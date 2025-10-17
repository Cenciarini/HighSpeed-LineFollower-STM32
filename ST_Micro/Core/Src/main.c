/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stddef.h>
#include "stdint.h"
#include "usbd_cdc_if.h"
#include "comunicaciones.h"
#include "ESP01.h"
#include "stm32f1xx_hal.h"
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	ALIVE=0xF0,      // Identificador para mensaje de "vivo"
	DATA=0xF1,       // Identificador para transmisión de datos
	DEBUGGER=0xE3,   // Identificador para mensajes de depuración
	AT_TRANSMIT=0xA0,// Identificador para transmisión AT
	AT_CONFIG=0xA1,  // Identificador para configuración AT
	MPU_READ = 0xAC, // Identificador para lectura del sensor MPU
	IR_READ = 0xAB,  // Identificador para lectura de sensores IR
	MOTOR = 0XA5,    // Identificador para control de motores
	PID_CONFIG = 0xA6,// Identificador para configuración PID
	ADC_VALUES = 0XA7,// Identificador para graficar valores ADC
	OTHERS           // Identificador para otros comandos
}_eID;

// Definición de una unión para almacenar banderas de estado
static union{
	struct{
		uint8_t tranparentDebug: 1; // Bandera para modo de depuración transparente
		uint8_t at_transmit: 1;     // Bandera para transmisión AT
		uint8_t comillas: 1;        // Bandera para manejo de comillas en configuración AT
		uint8_t mpu_read: 1;        // Bandera para indicar lectura del sensor MPU
		uint8_t mpu_read_ok: 1;     // Bandera para indicar que la lectura del MPU fue exitosa
		uint8_t ir_read: 1;         // Bandera para indicar lectura de sensores IR
		uint8_t adc_dma_start: 1;   // Bandera para indicar inicio de DMA en ADC
		uint8_t motor_prueba: 1;    // Bandera para prueba de motores
	}bit;
	uint8_t byte;  // Variable de 8 bits para acceder a todas las banderas como un solo byte
}flags;

typedef struct{
	uint8_t bufferRx[256];
	uint8_t bufferTx[256];
	uint8_t bufferPx[256];
	uint8_t Rxir;
	uint8_t Rxiw;
	uint8_t Txir;
	uint8_t Txiw;
	uint8_t Pxir;
	uint8_t Pxiw;
}bufferST;

// Variables para configuración y almacenamiento de datos

SSD1306_COLOR_t color; // Color de la pantalla OLED
//_sESP01_CONFIG myESP01_Config; // Configuración del módulo ESP01
_eESP01STATUS currentESP01states;
bufferST bufferESP01;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDRESS 0xD0 // Dirección I2C del sensor MPU6050
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Variables de uso general
uint8_t tempRxUART[2];
//char dataRXESP01; // Variable para almacenar datos recibidos por UART
uint32_t elapsedHeartBeat = 0, elapsedESP01Receive = 0, elapsedMPU = 0, elapsedMPU_tx = 0, elapseIR = 0, elapsedADC_draw = 0;
uint32_t elapsedESP01TimeOut = 0;
int16_t pwmIzq = 0, pwmDer = 0, pwmIzqInv = 0, pwmDerInv = 0, pwmBase = 999; // Variables de control de PWM
int error = 0, lastError = 0;
int cumError = 0, rateError = 0, correccionDer = 0, correccionIzq = 0; // Variables para control PID
uint8_t KpIzq = 0, KpDer = 0,  Vel = 100; // Coeficientes PID
uint16_t KdIzq = 0, KdDer = 0;
uint8_t raceStart = 0, adcDraw = 0; // Banderas de control

// Buffers para almacenamiento y transmisión de datos
char bufPayloadRx[32]; // Buffer para recepción de datos USB
char bufPayloadTx[32]; // Buffer para transmisión de datos USB
char payloadBck[256];  // Buffer de respaldo para datos USB
char bufDebug[32];      // Buffer para mensajes de depuración
char ipBuf[16];           // Almacenamiento de la dirección IP del ESP01
char *ip;
uint8_t TxLen = 0;

// Variables para lectura del sensor MPU6050
uint8_t MPU_rx[4], accelCount = 0;
int16_t accelXProm = 0, accelYProm = 0, gyroZProm = 0, gyroYProm = 0, gyroZAux[20], gyroYAux[20], accelXAux[20], accelYAux[20];

// Variables para manejo de ADC con DMA
uint32_t adc_dma_value[7], dma_result[7][20], ir_prom_value[7]; // Almacenamiento de valores ADC
uint8_t adc_dma_count = 0;
int8_t line_pos;

// Variables para la detección de la linea
uint8_t max_value_ir = 0;  // Índice del sensor IR con el valor máximo
uint8_t last_ir = 0;
uint32_t max_value = 0;    // Valor máximo encontrado en ir_prom_value

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//void mypinSet();
uint8_t myI2CInit();
void myI2CWriteMulti(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);
void myI2CWrite(uint8_t address, uint8_t reg, uint8_t data);
void decode(uint8_t *payloadBuf, uint8_t lenPx);
void onUSBRx(char *myBuf, int length);
//uint8_t onWiFiTx(char Txdata);
//void onWiFiRx(char *Rxdata, int RxLength);
//void TxCOM(uint8_t *TxBuf, int length);
void display_debug(char txt[],uint8_t X,uint8_t Y);
void MPU6050_Read_Data(void);
uint8_t MPU6050_Init(void);
void IR_PROMEDIO();
void MPU_PROMEDIO();
int8_t Line_detection();
void Pwm_Step();

void ESP01_CH_PD_Set(uint8_t value);
void ESP01_UART_Receive(uint8_t value);
int ESP01_UART_Send(uint8_t value);
void ESP01_ChangeState(_eESP01STATUS statusESP01);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Muestra un texto en un display OLED SSD1306 en una posición específica.
 *
 * Esta función borra la pantalla OLED, posiciona el cursor en las coordenadas (x, y)
 * indicadas y muestra el texto deseado utilizando la fuente `Font_7x10`. Finalmente,
 * actualiza la pantalla para reflejar los cambios.
 *
 * @param text Puntero a la cadena de texto que se desea mostrar.
 * @param x Coordenada horizontal (en píxeles) donde se colocará el texto.
 * @param y Coordenada vertical (en píxeles) donde se colocará el texto.
 */
void display_debug(char txt[],uint8_t x,uint8_t y) {
    //SSD1306_Clear(); // Borra la pantalla OLED
    SSD1306_GotoXY(x, y); // Posiciona el cursor en las coordenadas especificadas
    SSD1306_Puts(txt, &Font_7x10, 1); // Muestra el texto en la pantalla
    SSD1306_UpdateScreen(); // Actualiza la pantalla con el nuevo contenido
}

/**
 * @brief Callback que se ejecuta cuando finaliza una conversión ADC mediante DMA.
 *
 * Esta función se activa automáticamente al completarse la conversión ADC. Se encarga de
 * almacenar los valores convertidos en un buffer, manteniendo un historial de las últimas 8 muestras.
 *
 * @param hadc Puntero a la estructura de manejo del ADC.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	dma_result[0][adc_dma_count] = adc_dma_value[0];
	dma_result[1][adc_dma_count] = adc_dma_value[1];
	dma_result[2][adc_dma_count] = adc_dma_value[2];
	dma_result[3][adc_dma_count] = adc_dma_value[3];
	dma_result[4][adc_dma_count] = adc_dma_value[4];
	dma_result[5][adc_dma_count] = adc_dma_value[5];
	dma_result[6][adc_dma_count++] = adc_dma_value[6];

	if(adc_dma_count > 19)
		adc_dma_count = 0;
}

/**
 * @brief Callback de interrupción de recepción UART.
 *
 * Esta función se ejecuta automáticamente cuando se recibe un byte a través de UART.
 * Si la recepción proviene de USART1, se almacena el dato en `dataRXESP01`, se vuelve
 * a habilitar la recepción para recibir el siguiente byte y se procesa el dato recibido.
 * Además, si está activado el modo de depuración transparente, el dato se envía por USB.
 *
 * @param huart Puntero a la estructura de manejo de UART.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
    if(huart->Instance == USART1){ // Verifica que la interrupción proviene de USART1
    	ESP01_WriteRX(tempRxUART[0]);
    	HAL_UART_Receive_IT(&huart1, tempRxUART, 1);
    }
}

/**
 * @brief Cambia el estado del pin de salida del GPIO.
 *
 * Esta función modifica el estado del pin de salida configurado en `GPIOx_ODR`.
 * Se utiliza para alternar entre los estados alto y bajo
 *
 * @param value Valor alto o bajo que deseamos asignar a nuestro pin de salida
 */

void ESP01_CH_PD_Set(uint8_t value){
	HAL_GPIO_WritePin(ChipEnable_GPIO_Port, ChipEnable_Pin, value);
}

/**
 * @brief Maneja la recepción de datos desde el módulo WiFi ESP01.
 *
 * Esta función copia los datos recibidos en un buffer temporal
 *
 * @param value byte de gatagrama recibido a través de la ESP01
 */

void ESP01_UART_Receive(uint8_t value){
	bufferESP01.bufferRx[bufferESP01.Rxiw++] = value;
	elapsedESP01Receive = __HAL_TIM_GetCounter(&htim2);
}


/**
 * @brief Transmite un byte de datos a través de UART si el buffer de transmisión está vacío.
 *
 * La función verifica si el buffer de transmisión de USART1 está vacío. Si lo está,
 * se envía el byte `Txdata` a través del registro de datos `USART1->DR`. Además, si el
 * modo de depuración transparente está activado, se transmite el mismo dato por USB.
 *
 * @param value Byte de datos a transmitir.
 * @return uint8_t Devuelve 1 si el dato se transmitió exitosamente, 0 en caso contrario.
 */
int ESP01_UART_Send(uint8_t value){
	HAL_UART_Transmit(&huart1, &value, 1, 10);
	return  1;
}

void ESP01_ChangeState(_eESP01STATUS statusESP01){
	switch (statusESP01) {
		case ESP01_READY:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"READY");
			display_debug(bufPayloadTx, 1, 30);
			break;
		case ESP01_WIFI_CONNECTED:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"WIFI_CONNECTED");
			display_debug(bufPayloadTx, 1, 30);
			break;
		case ESP01_WIFI_NEW_IP:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"NEW_IP");
			display_debug(bufPayloadTx, 1, 30);
			break;
		case ESP01_UDPTCP_CONNECTED:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"UDP_CONNECTED");
			display_debug(bufPayloadTx, 1, 30);
			currentESP01states = ESP01_GET_IP;
			break;
		case ESP01_WIFI_NOT_SETED:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"WIFI_NOT_SET");
			display_debug(bufPayloadTx, 1, 30);
			break;
		case ESP01_WIFI_CONNECTING_WIFI:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"WIFI_CONNECTING");
			display_debug(bufPayloadTx, 1, 30);
			break;
		case ESP01_UDPTCP_DISCONNECTED:
			sprintf(bufPayloadTx,"UDP_DISCONNECTED");
			display_debug(bufPayloadTx, 1, 30);
			break;
		case ESP01_UDPTCP_CONNECTING:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"UDP_CONNECTING");
			display_debug(bufPayloadTx, 1, 30);
			break;
		case ESP01_SEND_BUSY:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"SEND_BUSY");
			display_debug(bufPayloadTx, 1, 30);
			break;
		case ESP01_SEND_READY:

			break;
		case ESP01_SEND_ERROR:
			SSD1306_Clear();
			sprintf(bufPayloadTx,"SEND_ERROR");
			display_debug(bufPayloadTx, 1, 30);
			break;
		default:
			break;
	}
}


//--------------------------------------------------------------



/**
 * @brief Maneja la recepción de datos desde el puerto USB.
 *
 * Esta función recibe datos desde la conexión USB y los transfiere a
 * `Comunicaciones_Recieve` para su procesamiento.
 *
 * @param myBuf Puntero al buffer que contiene los datos recibidos.
 * @param length Longitud de los datos recibidos.
 */
void onUSBRx(char *myBuf, int length) {
    //Comunicaciones_Recieve((uint8_t *)myBuf, length); // Envía los datos recibidos para su procesamiento
}


/**
 * @brief Decodifica el comando recibido y ejecuta la acción correspondiente.
 *
 * Esta función analiza el primer byte del buffer recibido y ejecuta
 * acciones específicas dependiendo del tipo de comando.
 *
 * @param payBuf Puntero al buffer de datos recibido.
 * @param payLength Longitud del buffer de datos recibido.
 */
void decode(uint8_t *payloadBuf, uint8_t lenPx) {

    // Analiza el primer byte del buffer para determinar la acción a ejecutar
    switch(payloadBuf[0]) {
        case ALIVE:
            // Responde con un mensaje de confirmación
            sprintf(bufDebug, "ALIVE OK");
            display_debug(bufDebug, 1, 15);
            bufferESP01.bufferPx[bufferESP01.Pxiw++] = 0xF0;
            flags.bit.at_transmit = 1;
            break;

        case MPU_READ:
            // Activa la bandera para leer datos del sensor MPU
            flags.bit.mpu_read = !flags.bit.mpu_read;
            flags.bit.mpu_read_ok = !flags.bit.mpu_read_ok;
            accelCount = 0;
            break;
        case IR_READ:
            // Activa o desactiva la lectura del sensor infrarrojo e indica el inicio de la carrera
            sprintf(bufDebug, "RACE START");
            SSD1306_Clear();
            display_debug(bufDebug, 1, 15);
            raceStart = !raceStart;
            if(!raceStart){
    			TIM4->CCR1 = 0;
    			TIM4->CCR2 = 0;
    			TIM4->CCR3 = 0;
    			TIM4->CCR4 = 0;
            }
            break;

        case DATA:
            // Envía los datos de los sensores ADC almacenados en `adc_dma_value`
            sprintf(bufPayloadTx, "0 = %ld\n1 = %ld", adc_dma_value[0], adc_dma_value[1]);
            break;

        case DEBUGGER:
            // Activa o desactiva el modo de depuración transparente
            flags.bit.tranparentDebug = !flags.bit.tranparentDebug;
            sprintf(bufPayloadTx, "Transparent: %i", flags.bit.tranparentDebug);
            break;
		case MOTOR:
			flags.bit.motor_prueba = !flags.bit.motor_prueba;
			sprintf(bufDebug, "STOP");
			SSD1306_Clear();
			display_debug(bufDebug, 1, 15);
			raceStart = 0;
			flags.bit.ir_read = 0;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = 0;
			TIM4->CCR4 = 0;
			break;
		case PID_CONFIG:
			KpIzq = payloadBuf[1];
			KdIzq = (uint16_t)((payloadBuf[2] << 8) | payloadBuf[3]);
			KpDer = payloadBuf[4];
			KdDer = (uint16_t)((payloadBuf[5] << 8) | payloadBuf[6]);
			Vel = payloadBuf[7];
			pwmBase = (999 * Vel) / 100;

			sprintf(bufDebug, "PID Updated");
			SSD1306_Clear();
			display_debug(bufDebug, 1, 15);
			break;
		case ADC_VALUES:
			// Activa o desactiva la bandera para leer datos del sensor MPU
			adcDraw = !adcDraw;
			break;
	default:
		sprintf(bufPayloadTx,"ERROR");
		display_debug(bufPayloadTx, 1, 30);
		break;
	}

    // Verifica si es necesario transmitir una respuesta y envía los datos si es necesario
    if (flags.bit.at_transmit) {
        flags.bit.at_transmit = 0;
        //TxPay = strlen(bufPayloadTx);
        Comunicaciones_encode(bufferESP01.bufferTx, &bufferESP01.Txiw, bufferESP01.bufferPx, bufferESP01.Pxiw, &bufferESP01.Pxir);
        if(bufferESP01.Txiw < bufferESP01.Txir){
        	TxLen = (256 - bufferESP01.Txir) + bufferESP01.Txiw;
        }else{
        	TxLen = bufferESP01.Txiw - bufferESP01.Txir;
        }
        ESP01_Send(bufferESP01.bufferTx, bufferESP01.Txir, TxLen, 256);
        bufferESP01.Txir = bufferESP01.Txiw;
    }

}

/**
 * @brief Inicializa la comunicación I2C verificando la disponibilidad del dispositivo.
 *
 * Esta función comprueba si el dispositivo en la dirección `SSD1306_I2C_ADDR`
 * está listo para la comunicación mediante el bus I2C.
 *
 * @return 1 si el dispositivo está listo, 0 en caso contrario.
 */
uint8_t myI2CInit(){
    // Comprueba si el dispositivo responde en la dirección especificada
    if(HAL_I2C_IsDeviceReady(&hi2c2, SSD1306_I2C_ADDR, 1, 20000) != HAL_OK){
        return 0; // Dispositivo no disponible
    }else{
        return 1; // Dispositivo listo
    }
}

/**
 * @brief Escribe múltiples bytes en un dispositivo I2C.
 *
 * Envía un conjunto de datos a un dispositivo en la dirección `address`,
 * comenzando desde el registro `reg`.
 *
 * @param address Dirección del dispositivo I2C.
 * @param reg Registro de inicio donde se almacenarán los datos.
 * @param data Puntero al buffer de datos a enviar.
 * @param count Cantidad de bytes a enviar.
 */
void myI2CWriteMulti(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count){
    uint8_t dt[256]; // Buffer temporal para la transmisión
    dt[0] = reg; // Primer byte es la dirección del registro

    uint8_t i;
    for(i = 0; i < count; i++) // Copia los datos al buffer temporal
        dt[i+1] = data[i];

    // Enviar los datos a través de I2C
    HAL_I2C_Master_Transmit(&hi2c2, address, dt, count+1, 10);
}

/**
 * @brief Escribe un solo byte en un dispositivo I2C.
 *
 * Envía un byte de datos a un dispositivo en la dirección `address`,
 * en el registro `reg`.
 *
 * @param address Dirección del dispositivo I2C.
 * @param reg Registro donde se escribirá el dato.
 * @param data Dato a escribir.
 */
void myI2CWrite(uint8_t address, uint8_t reg, uint8_t data){
    uint8_t dt[2]; // Buffer para el registro y el dato
    dt[0] = reg;  // Dirección del registro
    dt[1] = data;  // Dato a escribir

    // Enviar los datos a través de I2C
    HAL_I2C_Master_Transmit(&hi2c2, address, dt, 2, 10);
}

/**
  * @brief Inicializa el sensor MPU6050 configurando los registros necesarios
  *        para la comunicación y funcionamiento del dispositivo.
  *
  * Esta función configura el registro de encendido y el registro de configuración
  * de la tasa de muestreo del MPU6050 para permitir su funcionamiento correcto.
  *
  * @param None
  * @return None
  */
uint8_t MPU6050_Init(void){
	uint8_t check, data;

	HAL_I2C_Mem_Read(&hi2c2, (uint16_t)MPU6050_ADDRESS, 0x75, 1, &check, 1, 1000);

	if(check != 0x68)
		return 0;

	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, (uint16_t)MPU6050_ADDRESS, 0x6B, 1, &data, 1, 1000);

	data = 0x09;
	HAL_I2C_Mem_Write(&hi2c2, (uint16_t)MPU6050_ADDRESS, 0x19, 1, &data, 1, 1000);

	data = 0x02;
	HAL_I2C_Mem_Write(&hi2c2, (uint16_t)MPU6050_ADDRESS, 0x1A, 1, &data, 1, 1000);

	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, (uint16_t)MPU6050_ADDRESS, 0x1C, 1, &data, 1, 1000);

	return 1;
}

/**
  * @brief Lee los datos del sensor MPU6050.
  *
  * Esta función lee 14 bytes de datos desde el MPU6050, comenzando en el registro 0x3B
  * que contiene los datos de aceleración y giroscopio. La función utiliza la transmisión
  * I2C para obtener los datos y devuelve 1 si la operación es exitosa, 0 si falla.
  *
  * @param None
  * @return uint8_t: 1 si la lectura fue exitosa, 0 si hubo un error.
  */
void MPU6050_Read_Data(void){
	HAL_I2C_Mem_Read(&hi2c2, (uint16_t)MPU6050_ADDRESS, 0x3B, 1, MPU_rx, 4, 1000);

	accelXAux[accelCount] = (int16_t)((MPU_rx[0] << 8) | MPU_rx[1]);
	accelYAux[accelCount] = (int16_t)((MPU_rx[2] << 8) | MPU_rx[3]);

	HAL_I2C_Mem_Read(&hi2c2, (uint16_t)MPU6050_ADDRESS, 0x47, 1, MPU_rx, 2, 1000);

	gyroZAux[accelCount] = (int16_t)((MPU_rx[0] << 8) | MPU_rx[1]);

	if(accelCount > 19){
	  flags.bit.mpu_read_ok = 1;
	  accelCount = 0;
	}
}

/**
  * @brief Calcula el promedio de los valores IR almacenados en la matriz dma_result.
  *
  * Esta función recorre los 7 elementos de ir_prom_value y, para cada uno,
  * suma los valores de 8 lecturas almacenadas en dma_result. Luego,
  * divide la suma entre 8 para obtener el promedio.
  *
  * @param None
  * @return None
  */
void IR_PROMEDIO(){
	for (uint8_t index_ir = 0; index_ir < 7; index_ir++) {
		ir_prom_value[index_ir] = 0;  // Inicializa el acumulador de promedio en 0

		for (uint8_t index_read = 0; index_read < 20; index_read++) {
			ir_prom_value[index_ir] += dma_result[index_ir][index_read];  // Suma los valores de dma_result
		}

		ir_prom_value[index_ir] = ir_prom_value[index_ir] / 20;  // Calcula el promedio dividiendo por 8
	}
}

/**
  * @brief Detecta la posición de la línea en función de los valores IR promediados.
  *
  * Esta función analiza los valores de ir_prom_value para determinar la posición de la línea
  * en un rango de sensores IR. Identifica el sensor con el valor máximo, ajusta su posición
  * relativa y devuelve un resultado escalado.
  *
  * @param None
  * @return int8_t: Valor procesado que representa la posición de la línea detectada.
  */
int8_t Line_detection(){
	int8_t x_result;           // Resultado final de la detección de línea

	max_value = ir_prom_value[0];  // Se inicializa con el primer valor del array

	/** Recorre los sensores IR para encontrar el de mayor valor */
	for (uint8_t index_ir = 1; index_ir < 7; index_ir++) {
		if(max_value < ir_prom_value[index_ir]){
			max_value = ir_prom_value[index_ir];  // Actualiza el valor máximo
			max_value_ir = index_ir;  // Guarda el índice del sensor con mayor valor
		}
	}

	if(max_value > 500)
		last_ir = max_value_ir;

	x_result = max_value_ir * 3;  // Asigna una posición base en función del índice

	/** Ajusta la posición si el sensor máximo no es el primero (0) ni el último (6) */
	if((max_value_ir != 0) || (max_value_ir != 6)){

		/** Verifica si alguno de los sensores adyacentes tiene un valor significativo */
		if( (ir_prom_value[max_value_ir - 1] > (max_value / 2)) || (ir_prom_value[max_value_ir + 1] > (max_value / 2)) ) {

			/** Ajusta la posición según el sensor adyacente con mayor valor */
			if(ir_prom_value[max_value_ir - 1] >= ir_prom_value[max_value_ir + 1]){
				x_result--;  // Se ajusta la posición hacia la izquierda
			}else{
				x_result++;  // Se ajusta la posición hacia la derecha
			}

		}

	}else{  // Caso donde el sensor máximo es el primero (0) o el último (6)

		/** Ajusta la posición si el sensor 1 tiene un valor significativo */
		if( (max_value_ir == 0) && (ir_prom_value[1] > (max_value / 2)) ){
			x_result++;
		}

		/** Ajusta la posición si el sensor 5 tiene un valor significativo */
		if( (max_value_ir == 6) && (ir_prom_value[5] > (max_value / 2)) ){
			x_result--;
		}

	}

	x_result = x_result * 4;  // Escala el resultado
	x_result -= 36;  // Ajusta el valor final

	return x_result;  // Devuelve la posición detectada de la línea
}

void MPU_PROMEDIO(){
	accelXProm = 0;
	accelYProm = 0;
	gyroZProm = 0;

	for(uint8_t counter = 0; counter < 20; counter++) {
		accelXProm += accelXAux[counter];
		accelYProm += accelYAux[counter];
		gyroZProm += gyroZAux[counter];
	}

	gyroZProm = gyroZProm / 20;
	accelXProm = accelXProm / 20;
	accelYProm = accelYProm / 20;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	_sESP01Handle handleESP01;
	handleESP01.DoCHPD = ESP01_CH_PD_Set;
	handleESP01.WriteUSARTByte = ESP01_UART_Send;
	handleESP01.WriteByteToBufRX = ESP01_UART_Receive;

	flags.bit.mpu_read = 0;
	flags.bit.ir_read = 0;
	flags.bit.adc_dma_start = 0;
	flags.bit.motor_prueba = 0;

	bufferESP01.Txiw = 0;
	bufferESP01.Txir = 0;
	bufferESP01.Rxiw = 0;
	bufferESP01.Rxir = 0;
	bufferESP01.Pxiw = 0;
	bufferESP01.Pxir = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  ESP01_Init(&handleESP01);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  Attach_I2CInit(myI2CInit);
  Attach_I2CWriteMulti(myI2CWriteMulti);
  Attach_I2CWrite(myI2CWrite);

  CDC_Attach_RxFun(onUSBRx);
  Comunicaciones_Decode(decode);

  SSD1306_Init();
  if(MPU6050_Init()){
	  display_debug("MPU_OK", 1, 30);
  }else{
	  display_debug("MPU_DISCONNECT", 1, 30);
  }

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  TIM4->CCR2 = 0;
  TIM4->CCR3 = 0;
  TIM4->CCR1 = 0;
  TIM4->CCR4 = 0;
  pwmDer = 0;
  pwmIzq = 0;
  HAL_UART_Receive_IT(&huart1,tempRxUART,1);
  HAL_GPIO_WritePin(ChipEnable_GPIO_Port, ChipEnable_Pin, 0);
  HAL_ADC_Start_DMA(&hadc1, adc_dma_value, 7);  // Inicia la conversión ADC con DMA

  ESP01_AttachChangeState(ESP01_ChangeState);
  ESP01_SetWIFI("LabPrototip", "labproto");
  ESP01_StartUDP("192.168.2.114", 30001, 30010);

  currentESP01states = ESP01_WIFI_DISCONNECTED;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if((__HAL_TIM_GetCounter(&htim2) - elapsedHeartBeat) >= 1000 ){
		  elapsedHeartBeat = __HAL_TIM_GetCounter(&htim2);
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }

	  if((__HAL_TIM_GetCounter(&htim2) - elapsedESP01TimeOut) > 20){
		  elapsedESP01TimeOut = __HAL_TIM_GetCounter(&htim2);
		  ESP01_Timeout10ms();
	  }

	  if((__HAL_TIM_GetCounter(&htim2) - elapsedESP01Receive) > 100 && elapsedESP01Receive > 0){
		  elapsedESP01Receive = 0;
		  Comunicaciones_Recieve(bufferESP01.bufferRx, bufferESP01.Rxiw, &bufferESP01.Rxir);
	  }

	  switch (currentESP01states) {
		case ESP01_GET_IP:
			ip = ESP01_GetLocalIP();

			if (ip != NULL) {
				strncpy(ipBuf,ip,15);
				ipBuf[15] = '\0';
				currentESP01states = ESP01_READY;
				display_debug(ip, 1, 45);
			}
			break;
		case ESP01_READY:

			break;
		default:
			break;
	  }

	  /**
	   * Verifica si la carrera ha comenzado y si han transcurrido al menos 100 unidades de tiempo
	   * desde la última ejecución de la rutina de procesamiento de sensores IR.
	   */
	  if (raceStart && ((__HAL_TIM_GetCounter(&htim2) - elapseIR) >= 20)){

		  elapseIR = __HAL_TIM_GetCounter(&htim2);  // Actualiza el tiempo de referencia

		  IR_PROMEDIO();  // Calcula el promedio de las lecturas de los sensores IR

		  line_pos = Line_detection();  // Determina la posición de la línea detectada

		  error = line_pos;  // Asigna la posición detectada como error de control

		  if(max_value < 700){
			  if(last_ir > 3){
				  pwmDer = 200;
				  pwmDerInv = 0;
				  pwmIzqInv = 200;
				  pwmIzq = 0;
			  }else{
				  pwmDer = 0;
				  pwmDerInv = 200;
				  pwmIzqInv = 0;
				  pwmIzq = 200;
			  }
		  }else{
			  pwmDerInv = 0;
			  pwmIzqInv = 0;

			  cumError += error * 10;  // Acumula el error para el control integral
			  rateError = error - lastError;  // Calcula la derivada del error

			  /** Calcula la corrección del sistema usando el control PID */
			  correccionDer = KpDer * error + KdDer * rateError;
			  correccionIzq = KpIzq * error + KdIzq * rateError;

			  /** Restablece los valores de PWM a la base antes de aplicar la corrección */
			  pwmDer = pwmBase + correccionDer;
			  pwmIzq = pwmBase - correccionIzq;
			  pwmDerInv = 0;
			  pwmIzqInv = 0;

			  if(pwmDer > 999)
				  pwmDer = 999;

			  if(pwmDer < 0){
				  pwmDerInv = pwmDer * -1;
				  pwmDer = 0;
			  }

			  if(pwmIzq > 999)
				  pwmIzq = 999;

			  if(pwmIzq < 0){
				  pwmIzqInv = pwmIzq * -1;
				  pwmIzq = 0;
			  }

		  }

		  lastError = error;

		  TIM4->CCR1 = pwmDerInv;
		  TIM4->CCR2 = pwmDer;
		  TIM4->CCR3 = pwmIzq;
		  TIM4->CCR4 = pwmIzqInv;

	  }

	  /**
	   * Verifica si la bandera de lectura del MPU6050 está activada y si han transcurrido al menos 10 ms
	   * desde la última lectura del sensor.
	   */
	  if(flags.bit.mpu_read && ((__HAL_TIM_GetCounter(&htim2) - elapsedMPU) >= 20)){
		  elapsedMPU = __HAL_TIM_GetCounter(&htim2);  //Actualiza el tiempo de referencia

		  /** Intenta leer los datos del MPU6050 */
		  MPU6050_Read_Data();
	  }

	  /**
	   * Verifica si la lectura del MPU6050 fue exitosa y si han transcurrido al menos 200 ms
	   * desde la última transmisión de datos del MPU6050.
	   */
	  if(flags.bit.mpu_read_ok && ((__HAL_TIM_GetCounter(&htim2) - elapsedMPU_tx) > 400)){

		  elapsedMPU_tx = __HAL_TIM_GetCounter(&htim2);  // Actualiza el tiempo de referencia para la transmisión

		  MPU_PROMEDIO();

		  // Prepara el buffer de datos a transmitir
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = 0xAC;  // Código de inicio de trama
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = accelXProm >> 8;  // Ajusta y almacena el primer dato del MPU6050 (Parte alta aceleración X)
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = accelXProm;  // Ajusta y almacena el segundo dato del MPU6050 (Parte baja aceleración X)
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = accelYProm >> 8;  // Ajusta y almacena el tercer dato del MPU6050 (Parte alta aceleración Y)
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = accelYProm;  // Ajusta y almacena el cuarto dato del MPU6050 (Parte baja aceleración Y)
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = gyroZProm >> 8;  // Ajusta y almacena el quinto dato del MPU6050 (Parte alta giroscopio Z)
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = gyroZProm;  // Ajusta y almacena el sexto dato del MPU6050 (Parte baja giroscopio Z)

		  // Envía los datos almacenados en el buffer de transmisión
		  Comunicaciones_encode(bufferESP01.bufferTx, &bufferESP01.Txiw, bufferESP01.bufferPx, bufferESP01.Pxiw, &bufferESP01.Pxir);
		  if(bufferESP01.Txiw < bufferESP01.Txir){
			  TxLen = (256 - bufferESP01.Txir) + bufferESP01.Txiw;
		  }else{
			  TxLen = bufferESP01.Txiw - bufferESP01.Txir;
		  }
		  ESP01_Send(bufferESP01.bufferTx, bufferESP01.Txir, TxLen, 256);
		  bufferESP01.Txir = bufferESP01.Txiw;

	  }

	  /*
	   * Verifica si esta activada la bandera para dibujar el valor ADC y si han transcurrido al menos 500 unidades de tiempo
	   * desde la última vez que se transmitieron estos datos
	   */
	  if(adcDraw && ((__HAL_TIM_GetCounter(&htim2) - elapsedADC_draw) > 300)){
		  elapsedADC_draw = __HAL_TIM_GetCounter(&htim2);

		  if(!raceStart){
			  IR_PROMEDIO();
			  line_pos = Line_detection();
		  }

		  display_debug("ADC", 15, 45);

		  /** Prepara el buffer de datos a transmitir */
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = 0xA7;  // Código de inicio de trama
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = line_pos;
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = pwmIzq >> 8;
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = pwmIzq;
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = pwmDer >> 8;
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = pwmDer;
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = rateError >> 8;
		  bufferESP01.bufferPx[bufferESP01.Pxiw++] = rateError;

		  /** Envía los datos almacenados en el buffer de transmisión */
		  Comunicaciones_encode(bufferESP01.bufferTx, &bufferESP01.Txiw, bufferESP01.bufferPx, bufferESP01.Pxiw, &bufferESP01.Pxir);
		  if(bufferESP01.Txiw < bufferESP01.Txir){
			  TxLen = (256 - bufferESP01.Txir) + bufferESP01.Txiw;
		  }else{
			  TxLen = bufferESP01.Txiw - bufferESP01.Txir;
		  }
		  ESP01_Send(bufferESP01.bufferTx, bufferESP01.Txir, TxLen, 256);
		  bufferESP01.Txir = bufferESP01.Txiw;
	  }

	  ESP01_Task();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 249;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 719;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ChipEnable_GPIO_Port, ChipEnable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ChipEnable_Pin */
  GPIO_InitStruct.Pin = ChipEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ChipEnable_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

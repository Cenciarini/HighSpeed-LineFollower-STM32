# 🚀 High-Speed Line Follower Robot — STM32 Bluepill

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-STM32-blue)
![Language](https://img.shields.io/badge/language-C%2B%2B%20%26%20C-green)
![Software](https://img.shields.io/badge/tools-KiCAD%20%7C%20Qt%20Creator%20%7C%20SolidWorks%20%7C%20STM32CubeIDE-orange)

---

## 📖 Descripción general

Este proyecto presenta el desarrollo completo de un **robot móvil seguidor de línea de alta velocidad**, basado en un **microcontrolador STM32F103C8T6 (Bluepill)**.  
El sistema fue diseñado para lograr **gran precisión y estabilidad a altas velocidades** mediante el uso de **controladores PID independientes para cada rueda** y una arquitectura modular que integra hardware, firmware, modelado 3D y una interfaz de control desde PC.

El robot combina múltiples disciplinas de ingeniería:  
- ⚙️ **Diseño electrónico y PCB**  
- 💻 **Programación embebida en C**  
- 🧠 **Control en tiempo real (PID)**  
- 🌐 **Comunicación inalámbrica vía ESP32 / UDP**  
- 🖥️ **Interfaz gráfica en C++ con Qt Creator**  
- 🧩 **Diseño mecánico en SolidWorks**

---

## 🧰 Estructura del repositorio

```
📦 RobotLineFollower_STM32
│
├── PCB_Micro/
│   ├── schematics/
│   ├── gerbers/
│   └── README.md
│   📘 Contiene el diseño electrónico completo en KiCAD.
│
├── QT_Micro/
│   ├── mainwindow.cpp
│   ├── resources/
│   └── README.md
│   💻 Interfaz de usuario en Qt Creator (C++) para control remoto y monitoreo.
│
├── SOLID_Micro/
│   ├── ensamblajes/
│   ├── piezas/
│   └── archivos_stl/
│   🧩 Modelos 3D y partes imprimibles diseñadas en SolidWorks.
│
├── ST_Micro/
│   ├── Core/
│   ├── Drivers/
│   ├── Src/
│   └── Inc/
│   ⚙️ Proyecto en STM32CubeIDE con firmware completo para el Bluepill.
│
└── README.md
```

---

## ⚙️ ST_Micro — Firmware embebido

El firmware principal está implementado en **C** dentro de `STM32CubeIDE`, con soporte para múltiples periféricos del microcontrolador.  
Entre sus módulos más relevantes se incluyen:

### 🔧 Periféricos utilizados
- **I2C:** Lectura de sensor **MPU6050** (acelerómetro + giroscopio).  
- **DMA:** Adquisición continua de señales **ADC** para los sensores IR.  
- **PWM:** Control preciso de la velocidad de cada motor DC.  
- **UART:** Comunicación con módulo **ESP01 (WiFi)** para envío de datos vía UDP.  
- **Timers:** Sincronización de tareas y control de tiempo de muestreo.  
- **ADC:** Lectura analógica de los sensores de línea.  

### 🧠 Control PID
El control PID ajusta dinámicamente las velocidades de ambas ruedas para mantener el robot centrado sobre la línea incluso en curvas pronunciadas o a altas velocidades.

El algoritmo se ejecuta en tiempo real y los parámetros **Kp, Ki, Kd** pueden modificarse desde la interfaz gráfica mediante comunicación **UDP**.

### 🧾 Librerías implementadas
- `comunicaciones.c`: Implementa un protocolo propio con chequeo de errores y estructura de tramas (`HEADER`, `NBYTES`, `PAYLOAD`, `CKS`).  
- `ESP01.c`: Gestión completa del módulo WiFi ESP01 con comandos AT y control de estados.  
- `ssd1306.c` y `fonts.c`: Control del display OLED (visualización de estados del robot).  
- `main.c`: Orquesta la lógica principal, incluyendo adquisición de datos, control PID, comunicación, y visualización en pantalla.

---

## 💻 QT_Micro — Interfaz gráfica de control

Desarrollada en **Qt Creator (C++)**, esta interfaz permite:

- Conexión con el robot a través de **UDP (ESP32 o ESP01)**.  
- Ajuste dinámico de parámetros PID (Kp, Ki, Kd) para cada rueda.  
- Visualización en tiempo real de:
  - Estado del robot y sensores IR.
  - Gráficos del recorrido.
  - Datos del MPU6050 (ángulos, aceleraciones).  
- Control remoto del robot (inicio/parada, velocidad base, direcciones, etc.).

---

## ⚡ PCB_Micro — Electrónica y PCB

Diseñada completamente en **KiCAD**, esta placa integra:

- Microcontrolador **STM32F103C8T6** (Bluepill).  
- Etapa de potencia con control PWM para motores DC.  
- Sensores IR analógicos para detección de línea.  
- Reguladores de tensión y protección de alimentación.  
- Conectores para display OLED, ESP01, MPU6050 y ST-Link.  

Incluye:
- Archivos **.sch** (esquemáticos)
- Archivos **.kicad_pcb**
- Archivos **Gerber** listos para fabricación

---

## 🧩 SOLID_Micro — Diseño mecánico

El diseño mecánico del robot fue modelado íntegramente en **SolidWorks**, con foco en minimizar peso y tamaño manteniendo la rigidez estructural.  
Se incluyen:
- Ensambles del chasis y soporte de sensores.  
- Piezas STL listas para impresión 3D.  
- Montaje optimizado del PCB y baterías.

---

## 📡 Comunicación y protocolo

El robot y la interfaz de PC se comunican mediante **UDP**, usando un **protocolo propio de mensajes binarios** desarrollado en C.  
Cada mensaje posee estructura de seguridad y verificación de checksum, garantizando robustez en entornos de alta velocidad.

---

## 🧠 Principales características técnicas

| Componente | Descripción |
|-------------|-------------|
| **Microcontrolador** | STM32F103C8T6 (Cortex-M3, 72 MHz) |
| **Sensores** | IR analógicos + MPU6050 (I2C) |
| **Pantalla** | OLED SSD1306 (I2C) |
| **Comunicación** | ESP01 (UART → UDP) / Interfaz Qt |
| **Control** | PID doble canal (rueda izquierda / derecha) |
| **Alimentación** | Batería Li-Po 7.4V con regulación interna |
| **Software principal** | STM32CubeIDE, KiCAD, Qt Creator, SolidWorks |

---

## 🧠 Habilidades demostradas

Este proyecto refleja competencias avanzadas en:

- Diseño y fabricación de **PCBs profesionales** con KiCAD.  
- Programación de **firmware embebido en C para microcontroladores STM32**.  
- Implementación de **controladores PID en tiempo real**.  
- Desarrollo de **protocolos de comunicación personalizados**.  
- Integración de **periféricos complejos (ADC, DMA, I2C, UART, PWM)**.  
- Diseño de **interfaces gráficas multiplataforma (Qt Creator)**.  
- Diseño **mecánico 3D y prototipado rápido (SolidWorks)**.  

---


## 📄 Licencia

Este proyecto está licenciado bajo los términos de la **MIT License**.  
Consulta el archivo [LICENSE](LICENSE) para más información.

---

## 👤 Autor

**Gabriel Cenciarini**  
Ingeniero Mecatrónico
📧 [LinkedIn](https://www.linkedin.com/in/cenciarinigabriel/) | 💻 [GitHub](https://github.com/Cenciarini)

---

> “La integración entre electrónica, software y control no solo construye robots, sino ingenieros capaces de pensar en sistemas completos.”

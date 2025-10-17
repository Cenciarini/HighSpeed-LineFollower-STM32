#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtNetwork/QUdpSocket>
#include <QElapsedTimer>
#include <QPixmap>
#include <qdebug.h>

#define PI 3.14159162
#define H 54.0
#define Alpha 0.97738

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Initialize the track visualization widget
    Qpista1 = new qpista(1348,561,ui->QPista);

    // Initialize UDP sockets for communication
    mSocket_send = new QUdpSocket(this);
    mSocket_recive = new QUdpSocket(this);

    // Bind receiving socket to local IP and port for listening
    mSocket_recive->bind(QHostAddress("192.168.2.114"), 30001, QUdpSocket::ShareAddress);

    // Connect signal for incoming data to read handler
    connect(mSocket_recive, &QUdpSocket::readyRead, this, &MainWindow::readDatagrams);
}

MainWindow::~MainWindow()
{
    // Clean up dynamically allocated memory
    delete Qpista1;
    delete ui;
}

//-------------------------------------------------------------------------------------------------
// Resize event handler: updates the track widget size and UI labels when the main window resizes.
//-------------------------------------------------------------------------------------------------
void MainWindow::resizeEvent(QResizeEvent */*event*/)
{
    Qpista1->resize(ui->QPista->size());
    ui->label_5->setText(QString().number(ui->QPista->width()));
    ui->label_6->setText(QString().number(ui->QPista->height()));
}

//-------------------------------------------------------------------------------------------------
// Paint event: ensures the track is resized only once during initial rendering.
//-------------------------------------------------------------------------------------------------
void MainWindow::paintEvent(QPaintEvent */*event*/)
{
    static bool first = false;

    if(!first){
        Qpista1->resize(ui->QPista->size());
        first = true;
    }
}

//-------------------------------------------------------------------------------------------------
// Sends the currently selected datagram command over UDP when the push button is clicked.
//-------------------------------------------------------------------------------------------------
void MainWindow::on_pushButton_clicked()
{
    datagramaWrite.resize(seleccion.size());
    datagramaWrite = seleccion;

    if(commandSend == 172){
        firstRead = 1;
    }
    if(commandSend == 171){
        raceStart = !raceStart;
    }

    qDebug() << commandSend;

    // Transmit datagram to target device (local or lab)
    mSocket_send->writeDatagram(datagramaWrite, QHostAddress("192.168.2.115"), 30010);  // Home
    //mSocket_send->writeDatagram(datagramaWrite, QHostAddress("192.168.2.108"), 30001); // Lab prototype
}

//-------------------------------------------------------------------------------------------------
// Updates the datagram structure according to the selected command in the combo box.
//-------------------------------------------------------------------------------------------------
void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    seleccion.resize(9);

    switch (index) {
    case 0:
        break;
    case 1:
        // ALIVE command
        seleccion[0] = 'U';
        seleccion[1] = 'N';
        seleccion[2] = 'E';
        seleccion[3] = 'R';
        seleccion[4] = 0x02;
        seleccion[5] = 0x3A;
        seleccion[6] = 0xF0;
        seleccion[7] = 0xC4;
        seleccion[8] = '\0';
        break;
    case 2:
        // Accelerometer/Gyroscope data request
        seleccion[0] = 'U';
        seleccion[1] = 'N';
        seleccion[2] = 'E';
        seleccion[3] = 'R';
        seleccion[4] = 0x02;
        seleccion[5] = 0x3A;
        seleccion[6] = 0xAC;
        seleccion[7] = 0x98;
        seleccion[8] = '\0';
        break;
    case 3:
        // Command for another operation
        seleccion[0] = 'U';
        seleccion[1] = 'N';
        seleccion[2] = 'E';
        seleccion[3] = 'R';
        seleccion[4] = 0x02;
        seleccion[5] = 0x3A;
        seleccion[6] = 0xAB;
        seleccion[7] = 0x9F;
        seleccion[8] = '\0';
        break;
    case 4:
        // Alternative system command
        seleccion[0] = 'U';
        seleccion[1] = 'N';
        seleccion[2] = 'E';
        seleccion[3] = 'R';
        seleccion[4] = 0x02;
        seleccion[5] = 0x3A;
        seleccion[6] = 0xA5;
        seleccion[7] = 0x91;
        seleccion[8] = '\0';
        break;
    default:
        break;
    }

    commandSend = seleccion[6];
}

//-------------------------------------------------------------------------------------------------
// Reads available UDP datagrams and triggers the decoding routine.
//-------------------------------------------------------------------------------------------------
void MainWindow::readDatagrams()
{
    if(mSocket_recive->hasPendingDatagrams()) {
        datagramaRead.resize(int(mSocket_recive->pendingDatagramSize()));
        mSocket_recive->readDatagram(datagramaRead.data(),datagramaRead.size());
    }
    decodeDatagram();
}

//-------------------------------------------------------------------------------------------------
// Decodes incoming datagrams using a finite state machine to verify header, payload, and checksum.
//-------------------------------------------------------------------------------------------------
void MainWindow::decodeDatagram()
{
    uint8_t index = 0, indexPayload = 0;

    while (index != datagramaRead.size()) {
        switch (stateDecode)
        {
        case 0:
            // Expect header byte 'U'
            if (datagramaRead[index++] == 'U') {
                stateDecode = 1;
                cksRx = 0;
            }else{
                stateDecode = 0;
                command = ERROR;
                return;
            }
            break;
        case 1:
            // Expect 'N'
            if (datagramaRead[index++] == 'N') {
                stateDecode = 2;
            }else{
                stateDecode = 0;
                command = ERROR;
                return;
            }
            break;
        case 2:
            // Expect 'E'
            if (datagramaRead[index++] == 'E'){
                stateDecode = 3;
            }else{
                stateDecode = 0;
                command = ERROR;
                return;
            }
            break;
        case 3:
            // Expect 'R'
            if (datagramaRead[index++] == 'R'){
                stateDecode = 4;
            }else{
                stateDecode = 0;
                command = ERROR;
                return;
            }
            break;
        case 4:
            // Payload size
            nBytes = datagramaRead[index++];
            stateDecode = 5;
            break;
        case 5:
            // Expect ':' separator and compute checksum seed
            if (datagramaRead[index++] == ':'){
                stateDecode = 6;
                cksRx = 'U' ^ 'N' ^ 'E' ^ 'R' ^ nBytes ^ ':';
                indexPayload = 0;
            }else{
                stateDecode = 0;
                command = ERROR;
                return;
            }
            break;
        case 6:
            // Read payload and update checksum
            if(nBytes > 1){
                buff[indexPayload++] = datagramaRead[index];
                cksRx ^= datagramaRead[index++];
            }
            nBytes--;
            // Validate checksum once payload is complete
            if(nBytes == 0){
                stateDecode = 0;
                uint8_t cksRxAux = datagramaRead[index];
                if(cksRx == cksRxAux){
                    analisisData();
                }
            }
            break;
        default:
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------
// Processes valid decoded data depending on the command type and updates the GUI accordingly.
//-------------------------------------------------------------------------------------------------
void MainWindow::analisisData()
{
    switch (buff[0]) {
    case 0xF0:
        ui->label_7->setText("Estoy vivo");
        break;

    case 0xAC:  {
        // Parse IMU data (accelerometer and gyroscope)
        accelX = (buff[1] << 8) | buff[2];
        accelY = (buff[3] << 8) | buff[4];
        gyroZ = (buff[5] << 8) | buff[6];

        // Initialize variables
        gZ = 0.0;
        aX = 0.0;
        aY = 0.0;
        theta = 0.0;

        // Convert raw data to physical units
        gZ = ((float)gyroZ / 131.0);
        aX = ((float)accelX / 16384.0) * 9810.0;
        aY = ((float)accelY / 16384.0) * 9810.0;

        aX *= 2;
        aY *= 2;

        // Offset calibration at first reading
        if(firstRead == 1){
            timer.start();
            aYOffset = aY;
            aXOffset = aX;
            gZOffset = gZ;
            firstRead = 0;
        }else{
            // Apply offset compensation
            aX -= aXOffset;
            aY -= aYOffset;
            gZ -= gZOffset;

            // Deadband filtering for noise reduction
            if(aY > -50.0 && aY < 50.0) aY = 0.0;
            if(aX > -50.0 && aX < 50.0) aX = 0.0;

            dT = timer.elapsed() / 1000.0;
            timer.restart();

            // Integrate angular rate to compute orientation
            if(gZ > -1.0 && gZ < 1.0){
                gZ = 0.0;
            }else{
                gZ = (gZ * dT)/ 3.1415916;
                gT += gZ;
            }

            // Keep orientation within 0–2π range
            if(gT > (2*PI)) gT -= (2 * PI);
            if(gT < 0.0) gT = (2 * PI) - gT;

            QString qvalor;
            ui->label->setText(qvalor.number(aX));
            ui->label_2->setText(qvalor.number(aY));
            ui->label_3->setText(qvalor.number(gZ));

            // Compute displacement based on acceleration and time
            aT = sqrt((aX * aX) + (aY * aY));
            sT = 0.5 * aT * dT * dT;

            sXo = sT * sin(gT);
            sYo = sT * cos(gT);

            sXf += sXo;
            sYf += sYo;

            ui->label_5->setText(qvalor.number(gT));
            ui->label_6->setText(qvalor.number(gyroDerCount));

            // Draw circuit path if enabled and race started
            if(circuitDraw == 1 && raceStart == 1){
                updateCircuitDraw();
            }
        }
        break;
    }

    case 0xAA:  {
        // Debug or status message
        char valor = buff[1];
        int ivalor = (int)valor;
        QString qvalor;
        ui->label_7->setText(qvalor.number(ivalor));
        break;
    }

    case 0xA7: {
        // Parse ADC and PWM data for IR sensors or motors
        adc_value = buff[1];
        pwmIzq = (buff[2] << 8) | buff[3];
        pwmDer = (buff[4] << 8) | buff[5];
        rateError = (int)(buff[6] << 8) | buff[7];
        QString qvalor;
        ui->label_5->setText(qvalor.number(pwmIzq));
        ui->label_6->setText(qvalor.number(pwmDer));
        ui->label_7->setText(qvalor.number(rateError));
        updateADCread();
    }
    default:
        break;
    }
}

//-------------------------------------------------------------------------------------------------
// Visualizes ADC sensor data as a graphical bar indicator on the track canvas.
//-------------------------------------------------------------------------------------------------
void MainWindow::updateADCread()
{
    QPen pen;
    QPainter paint(Qpista1->getCanvas());
    QPoint point, point_adc;
    QPoint puntoIzq, puntoDer;

    puntoIzq.setX(404);
    puntoIzq.setY(200);
    puntoDer.setX(944);
    puntoDer.setY(200);

    Qpista1->getCanvas()->fill(Qt::white);
    pen.setWidth(3);
    pen.setColor(Qt::darkBlue);
    paint.setPen(pen);
    paint.drawLine(puntoIzq,puntoDer);

    adc_value += 36;
    adc_value = adc_value/4;

    point.setX(944 - adc_value * 30);
    point.setY(170);
    point_adc.setX(944 - adc_value * 30);
    point_adc.setY(230);

    pen.setWidth(20);
    pen.setColor(Qt::darkGray);
    paint.setPen(pen);
    paint.drawLine(point,point_adc);

    Qpista1->update();
}

//-------------------------------------------------------------------------------------------------
// Draws the current trajectory of the system on the virtual track canvas.
//-------------------------------------------------------------------------------------------------
void MainWindow::updateCircuitDraw()
{
    QPen pen;
    QPainter paint(Qpista1->getCanvas());

    if(firstUpdate == 1){
        pointAnt.setX(700);
        pointAnt.setY(300);
        firstUpdate = 0;
    }

    pActX = sXf + 700;
    pActY = sYf + 300;

    pointAct.setX(pActX);
    pointAct.setY(pActY);

    pAntX = pActX;
    pAntY = pActY;

    pen.setWidth(2);
    pen.setColor(Qt::darkBlue);
    paint.setPen(pen);
    paint.drawLine(pointAnt,pointAct);
    Qpista1->update();

    pointAnt.setX(pAntX);
    pointAnt.setY(pActY);
}

//-------------------------------------------------------------------------------------------------
// Sends PID configuration parameters to the remote system via UDP.
//-------------------------------------------------------------------------------------------------
void MainWindow::on_pushButton_2_clicked()
{
    PIDdatagram.resize(16);

    PIDdatagram[0] = 'U';
    PIDdatagram[1] = 'N';
    PIDdatagram[2] = 'E';
    PIDdatagram[3] = 'R';
    PIDdatagram[4] = 0x09;
    PIDdatagram[5] = 0x3A;
    PIDdatagram[6] = 0xA6;
    PIDdatagram[7] = (char)ui->KpIzq->value();
    PIDdatagram[8] = (char)(ui->KdIzq->value() >> 8);
    PIDdatagram[9] = (char)ui->KdIzq->value();
    PIDdatagram[10] = (char)ui->KpDer->value();
    PIDdatagram[11] = (char)(ui->KdDer->value() >> 8);
    PIDdatagram[12] = (char)ui->KdDer->value();
    PIDdatagram[13] = (char)ui->VelPorc->value();

    // Compute XOR-based checksum for datagram integrity
    PIDdatagram[14] = 'U' ^ 'N' ^ 'E' ^ 'R' ^ 0x09 ^ ':' ^ 0xA6 ^ PIDdatagram[7] ^ PIDdatagram[8]  ^ PIDdatagram[9] ^ PIDdatagram[10] ^ PIDdatagram[11] ^ PIDdatagram[12] ^ PIDdatagram[13];
    PIDdatagram[15] = '\0';

    datagramaWrite.resize(PIDdatagram.size());
    datagramaWrite = PIDdatagram;

    //mSocket_send->writeDatagram(datagramaWrite, QHostAddress("192.168.2.108"), 30001); // Lab prototype
    mSocket_send->writeDatagram(datagramaWrite, QHostAddress("192.168.2.115"), 30010);  // Home
}

//-------------------------------------------------------------------------------------------------
// Handles IR sensor visualization toggle and sends corresponding command to the remote system.
//-------------------------------------------------------------------------------------------------
void MainWindow::on_checkBox_stateChanged(int arg1)
{
    QPen pen;
    QPainter paint(Qpista1->getCanvas());
    QPoint puntoIzq, puntoDer;

    puntoIzq.setX(404);
    puntoIzq.setY(200);
    puntoDer.setX(944);
    puntoDer.setY(200);

    if(arg1){
        Qpista1->getCanvas()->fill(Qt::white);
        pen.setWidth(3);
        pen.setColor(Qt::darkBlue);
        paint.setPen(pen);
        paint.drawLine(puntoIzq,puntoDer);
    }else{
        Qpista1->getCanvas()->fill(Qt::white);
    }
    Qpista1->update();

    // Build and send IR command datagram
    IRdatagram.resize(9);
    IRdatagram[0] = 'U';
    IRdatagram[1] = 'N';
    IRdatagram[2] = 'E';
    IRdatagram[3] = 'R';
    IRdatagram[4] = 0x02;
    IRdatagram[5] = 0x3A;
    IRdatagram[6] = 0xA7;
    IRdatagram[7] = 'U' ^ 'N' ^ 'E' ^ 'R' ^ 0x02 ^ ':' ^ 0xA7;
    IRdatagram[8] = '\0';

    datagramaWrite.resize(IRdatagram.size());
    datagramaWrite = IRdatagram;

    //mSocket_send->writeDatagram(datagramaWrite, QHostAddress("192.168.2.108"), 30001); // Lab prototype
    mSocket_send->writeDatagram(datagramaWrite, QHostAddress("192.168.2.115"), 30010);  // Home
}

//-------------------------------------------------------------------------------------------------
// Handles circuit drawing checkbox: toggles visualization of trajectory plotting.
//-------------------------------------------------------------------------------------------------
void MainWindow::on_checkBox_2_stateChanged(int arg1)
{
    if(arg1){
        Qpista1->getCanvas()->fill(Qt::white);
        circuitDraw = 1;
        firstUpdate = 1;
    }else{
        Qpista1->getCanvas()->fill(Qt::white);
        circuitDraw = 0;
    }
    Qpista1->update();
}

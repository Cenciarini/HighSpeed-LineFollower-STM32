#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QElapsedTimer>
#include "qpista.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QUdpSocket;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();
    void resizeEvent(QResizeEvent *event);
    void paintEvent(QPaintEvent *event);
    void on_comboBox_currentIndexChanged(int index);
    void readDatagrams();
    void decodeDatagram();
    void analisisData();
    void updateADCread();
    void updateCircuitDraw();

    //uint8_t sizeOf(QByteArray &array);

    void on_pushButton_2_clicked();

    void on_checkBox_stateChanged(int arg1);

    void on_checkBox_2_stateChanged(int arg1);

private:
    Ui::MainWindow *ui;

    qpista *Qpista1;

    QPoint pointAnt, pointAct;

    QElapsedTimer timer;

    QUdpSocket *mSocket_send;
    QUdpSocket *mSocket_recive;

    QByteArray seleccion;
    QByteArray PIDdatagram;
    QByteArray IRdatagram;
    QByteArray datagramaWrite;
    QByteArray datagramaRead;

    uint8_t stateDecode = 0;
    uint8_t nBytes = 0;
    uint8_t cksRx;
    uint8_t cksRx_;
    uint8_t command = 0;
    uint8_t buff[256];
    int8_t adc_value;
    uint16_t pwmIzq = 0;
    uint16_t pwmDer = 0;

    uint8_t commandSend = 0;

    uint8_t gyroDerCount = 0;

    /*
    uint16_t IR0 = 0;
    uint16_t IR1 = 0;
    uint16_t IR2 = 0;
    uint16_t IR3 = 0;
    uint16_t IR4 = 0;
    uint16_t IR5 = 0;
    uint16_t IR6 = 0;
*/

    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;

    float gZ = 0.0;
    float gT = 0.0;
    float gZOffset = 0.0;

    float theta = 0.0;

    float dT = 0.0;

    int16_t accelX = 0;
    float aX = 0.0;
    float aXOffset = 0.0;
    float vX = 0.0;
    float vXi = 0.0;
    int sXf = 0;
    int sXaux_1 = 0;
    int sXaux_2 = 0;
    int sXo = 0;
    int16_t accelY = 0;
    float aY = 0.0;
    float aYOffset = 0.0;
    float vY = 0.0;
    float vYi = 0.0;
    int sYf = 0;
    int sYaux_1 = 0;
    int sYaux_2 = 0;
    int sYo = 0;

    int rateError = 0;

    float aT = 0.0;
    float sT = 0.0;

    uint8_t firstRead = 1;
    uint8_t firstUpdate = 1;
    uint8_t raceStart = 0;
    uint8_t circuitDraw = 0;

    int pActX = 0;
    int pActY = 0;
    int pAntX = 0;
    int pAntY = 0;

    typedef enum{
        NADA,
        ERROR,
        ALIVE,
        PISTA
    }_command;

};
#endif // MAINWINDOW_H

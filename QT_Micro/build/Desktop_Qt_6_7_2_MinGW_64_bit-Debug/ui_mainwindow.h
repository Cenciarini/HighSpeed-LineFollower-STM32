/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_3;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_5;
    QLabel *label_6;
    QComboBox *comboBox;
    QPushButton *pushButton;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout_3;
    QLabel *label;
    QSpinBox *KpIzq;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_2;
    QSpinBox *KdIzq;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_3;
    QSpinBox *KpDer;
    QVBoxLayout *verticalLayout_8;
    QLabel *label_8;
    QSpinBox *KdDer;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_4;
    QSpinBox *VelPorc;
    QVBoxLayout *verticalLayout;
    QLabel *label_7;
    QPushButton *pushButton_2;
    QGridLayout *gridLayout;
    QFrame *QPista;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(820, 724);
        QSizePolicy sizePolicy(QSizePolicy::Policy::Fixed, QSizePolicy::Policy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setStyleSheet(QString::fromUtf8("QWidget {\n"
"    background-color: #f8f9fa;\n"
"    color: #343a40;\n"
"    font-family: \"Verdana\";\n"
"}\n"
"\n"
"QPushButton {\n"
"    background-color: #007bff;\n"
"    border: none;\n"
"    color: white;\n"
"    padding: 6px;\n"
"    border-radius: 4px;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #0056b3;\n"
"}\n"
"\n"
"QComboBox, QSpinBox {\n"
"    background-color: white;\n"
"    border: 1px solid #ced4da;\n"
"    color: #495057;\n"
"}\n"
"\n"
"QFrame {\n"
"    border: 1px solid #adb5bd;\n"
"    border-radius: 5px;\n"
"}\n"
""));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        verticalLayout_7 = new QVBoxLayout(centralwidget);
        verticalLayout_7->setObjectName("verticalLayout_7");
        verticalLayout_7->setContentsMargins(-1, 20, -1, -1);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        label_5 = new QLabel(centralwidget);
        label_5->setObjectName("label_5");
        QFont font;
        font.setFamilies({QString::fromUtf8("Verdana")});
        font.setBold(false);
        label_5->setFont(font);
        label_5->setCursor(QCursor(Qt::CursorShape::ArrowCursor));

        horizontalLayout_4->addWidget(label_5);

        label_6 = new QLabel(centralwidget);
        label_6->setObjectName("label_6");

        horizontalLayout_4->addWidget(label_6);

        comboBox = new QComboBox(centralwidget);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName("comboBox");
        comboBox->setEnabled(true);
        comboBox->setEditable(false);

        horizontalLayout_4->addWidget(comboBox);

        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName("pushButton");
        pushButton->setCheckable(false);

        horizontalLayout_4->addWidget(pushButton);

        horizontalLayout_4->setStretch(2, 1);

        horizontalLayout_3->addLayout(horizontalLayout_4);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        checkBox = new QCheckBox(centralwidget);
        checkBox->setObjectName("checkBox");

        verticalLayout_2->addWidget(checkBox);

        checkBox_2 = new QCheckBox(centralwidget);
        checkBox_2->setObjectName("checkBox_2");

        verticalLayout_2->addWidget(checkBox_2);


        horizontalLayout_3->addLayout(verticalLayout_2);


        verticalLayout_7->addLayout(horizontalLayout_3);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(11);
        verticalLayout_3->setObjectName("verticalLayout_3");
        label = new QLabel(centralwidget);
        label->setObjectName("label");
        label->setTextFormat(Qt::TextFormat::AutoText);
        label->setScaledContents(false);
        label->setWordWrap(true);

        verticalLayout_3->addWidget(label, 0, Qt::AlignmentFlag::AlignHCenter);

        KpIzq = new QSpinBox(centralwidget);
        KpIzq->setObjectName("KpIzq");
        KpIzq->setMaximum(256);

        verticalLayout_3->addWidget(KpIzq);


        horizontalLayout_5->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(11);
        verticalLayout_4->setObjectName("verticalLayout_4");
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName("label_2");
        label_2->setTextFormat(Qt::TextFormat::AutoText);
        label_2->setScaledContents(false);
        label_2->setWordWrap(true);

        verticalLayout_4->addWidget(label_2, 0, Qt::AlignmentFlag::AlignHCenter);

        KdIzq = new QSpinBox(centralwidget);
        KdIzq->setObjectName("KdIzq");
        KdIzq->setMaximum(65000);

        verticalLayout_4->addWidget(KdIzq);


        horizontalLayout_5->addLayout(verticalLayout_4);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(11);
        verticalLayout_5->setObjectName("verticalLayout_5");
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName("label_3");
        label_3->setTextFormat(Qt::TextFormat::AutoText);
        label_3->setScaledContents(false);
        label_3->setWordWrap(true);

        verticalLayout_5->addWidget(label_3, 0, Qt::AlignmentFlag::AlignHCenter);

        KpDer = new QSpinBox(centralwidget);
        KpDer->setObjectName("KpDer");
        KpDer->setMaximum(256);
        KpDer->setSingleStep(1);
        KpDer->setStepType(QAbstractSpinBox::StepType::DefaultStepType);

        verticalLayout_5->addWidget(KpDer);


        horizontalLayout_5->addLayout(verticalLayout_5);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName("verticalLayout_8");
        label_8 = new QLabel(centralwidget);
        label_8->setObjectName("label_8");
        label_8->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_8->setWordWrap(true);

        verticalLayout_8->addWidget(label_8);

        KdDer = new QSpinBox(centralwidget);
        KdDer->setObjectName("KdDer");
        KdDer->setMaximum(65000);

        verticalLayout_8->addWidget(KdDer);


        horizontalLayout_5->addLayout(verticalLayout_8);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(11);
        verticalLayout_6->setObjectName("verticalLayout_6");
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName("label_4");
        label_4->setTextFormat(Qt::TextFormat::AutoText);
        label_4->setScaledContents(false);
        label_4->setWordWrap(true);

        verticalLayout_6->addWidget(label_4, 0, Qt::AlignmentFlag::AlignHCenter);

        VelPorc = new QSpinBox(centralwidget);
        VelPorc->setObjectName("VelPorc");
        VelPorc->setMaximum(100);
        VelPorc->setSingleStep(5);

        verticalLayout_6->addWidget(VelPorc);


        horizontalLayout_5->addLayout(verticalLayout_6);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName("label_7");

        verticalLayout->addWidget(label_7);

        pushButton_2 = new QPushButton(centralwidget);
        pushButton_2->setObjectName("pushButton_2");

        verticalLayout->addWidget(pushButton_2);


        horizontalLayout_5->addLayout(verticalLayout);


        verticalLayout_7->addLayout(horizontalLayout_5);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        gridLayout->setContentsMargins(0, -1, -1, -1);
        QPista = new QFrame(centralwidget);
        QPista->setObjectName("QPista");
        QPista->setFrameShape(QFrame::Shape::StyledPanel);
        QPista->setFrameShadow(QFrame::Shadow::Raised);

        gridLayout->addWidget(QPista, 0, 0, 1, 1);


        verticalLayout_7->addLayout(gridLayout);

        verticalLayout_7->setStretch(2, 1);
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        comboBox->setItemText(0, QCoreApplication::translate("MainWindow", "Seleccione un comando", nullptr));
        comboBox->setItemText(1, QCoreApplication::translate("MainWindow", "ALIVE", nullptr));
        comboBox->setItemText(2, QCoreApplication::translate("MainWindow", "ACELEROMETRO", nullptr));
        comboBox->setItemText(3, QCoreApplication::translate("MainWindow", "INFRAROJOS", nullptr));
        comboBox->setItemText(4, QCoreApplication::translate("MainWindow", "MOTOR", nullptr));

        pushButton->setText(QCoreApplication::translate("MainWindow", "Enviar", nullptr));
        checkBox->setText(QCoreApplication::translate("MainWindow", "Visualizar lectura de sensores IR", nullptr));
        checkBox_2->setText(QCoreApplication::translate("MainWindow", "Graficar pista de pruebas", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Kp Izquierdo", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Kd Izquierdo", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Kp Derecho", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "Kd Derecho", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Velocidad (%)", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "Actualizar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

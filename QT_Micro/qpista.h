#ifndef QPISTA_H
#define QPISTA_H

#include <QWidget>
#include <QPainter>

class qpista : public QWidget
{

    Q_OBJECT

    public:
        qpista(int aWidth = 800, int aHeight = 561, QWidget *parent = nullptr);

        void setWidth(int aWidth);
        void setHeight(int aHeight);
        QPixmap *getCanvas(void);

    signals:

    public slots:

    protected:
        void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
        void resizeEvent(QResizeEvent *event)Q_DECL_OVERRIDE;

    private:
        QPixmap *pixelCanvas;
};

#endif // QPISTA_H

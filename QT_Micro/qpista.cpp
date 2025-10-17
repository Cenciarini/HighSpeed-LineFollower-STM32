#include "qpista.h"

/**
 * @brief Constructs a qpista widget that contains a drawable QPixmap.
 *
 * Initializes the internal pixmap (pixelCanvas) with the same size
 * as the parent widget if one exists, or with the provided width and height otherwise.
 */
qpista::qpista(int aWidth, int aHeight, QWidget *parent) : QWidget(parent)
{
    // If a parent widget is provided, inherit its dimensions.
    if (parent) {
        aWidth = parent->width();
        aHeight = parent->height();
    }

    // Initialize the drawing surface.
    pixelCanvas = new QPixmap(aWidth, aHeight);
    this->resize(aWidth, aHeight);
    pixelCanvas->fill(Qt::black);
}

/**
 * @brief Sets a new width for the canvas and resizes the widget accordingly.
 */
void qpista::setWidth(int aWidth)
{
    pixelCanvas->size().setWidth(aWidth);
    this->resize(aWidth, this->height());
}

/**
 * @brief Sets a new height for the canvas and resizes the widget accordingly.
 */
void qpista::setHeight(int aHeight)
{
    pixelCanvas->size().setHeight(aHeight);
    this->resize(this->width(), aHeight);
}

/**
 * @brief Returns a pointer to the current QPixmap used as the drawing canvas.
 *
 * @return QPixmap* Pointer to the internal pixel canvas.
 */
QPixmap *qpista::getCanvas()
{
    return pixelCanvas;
}

/**
 * @brief Handles the paint event and draws the current QPixmap content on the widget.
 *
 * This ensures that the latest state of the drawing canvas is rendered on screen.
 */
void qpista::paintEvent(QPaintEvent* /*event*/)
{
    QPainter Canvas(this);
    Canvas.drawPixmap(0, 0, *pixelCanvas);
}

/**
 * @brief Handles the resize event to maintain the current drawing after widget resizing.
 *
 * When the widget is resized, the existing pixmap is copied and scaled
 * to match the new dimensions while preserving previously drawn content.
 */
void qpista::resizeEvent(QResizeEvent* /*event*/)
{
    QPixmap aux(this->width(), this->height());

    // Copy the current content before scaling to new dimensions.
    aux.copy(pixelCanvas->rect());

    // Adjust the internal pixmap to fit the new widget size.
    pixelCanvas->scaled(aux.width(), aux.height());
    pixelCanvas->swap(aux);
}

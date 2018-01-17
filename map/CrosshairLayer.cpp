#include <marble/GeoPainter.h>
#include <marble/ViewportParams.h>
#include "CrosshairLayer.h"
#include "RoboticsMap.h"

using namespace MapAbstraction;
using namespace Marble;

CrosshairLayer::CrosshairLayer(RoboticsMap *rm) : MapLayerInterface(rm)
{
}

QStringList CrosshairLayer::renderPosition() const
{
    return QStringList() << "FLOAT_ITEM";
}

bool CrosshairLayer::render(GeoPainter *painter, ViewportParams *viewport, const QString &, GeoSceneLayer *)
{
    if (!visible())
        return true;

    int x = viewport->size().width() / 2;
    int y = viewport->size().height() / 2;
    const qreal wingSize = 25;

    QPen crosshairPen(Qt::red);
    crosshairPen.setWidth(1);
    painter->save();

    painter->setPen(crosshairPen);
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->drawLine(QLineF(x - wingSize, y, x + wingSize, y));
    painter->drawLine(QLineF(x, y - wingSize, x, y + wingSize));

    painter->restore();
    return true;
}

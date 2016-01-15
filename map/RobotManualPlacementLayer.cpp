#include <QMouseEvent>

#include <marble/GeoDataLineString.h>
#include <marble/GeoPainter.h>
#include <marble/MarbleMap.h>

#include "MapLibraryHelpers.h"
#include "RobotManualPlacementLayer.h"

using namespace Marble;

RobotManualPlacementLayer::RobotManualPlacementLayer(MarbleMap *map)
    : mVisible(false), mMap(map), mTracking(false)
{
}

QStringList RobotManualPlacementLayer::renderPosition() const
{
    return QStringList() << "FLOAT_ITEM";
}

bool RobotManualPlacementLayer::render(GeoPainter *painter, ViewportParams *, const QString &, GeoSceneLayer *)
{   //TODO - check if it is in viewport;
    if (!mVisible || !mReferenceCoords.isValid() || !mMouseCoords.isValid() || !mTracking)
        return true;

    QPen pen(Qt::red);
    pen.setStyle(Qt::DashDotLine);
    painter->save();

    painter->setPen(pen);
    painter->setRenderHint(QPainter::Antialiasing, true);
    GeoDataLineString string(Tessellate);
    string.append(mReferenceCoords);
    string.append(mMouseCoords);
    painter->drawPolyline(string);

    painter->restore();
    return true;
}

void RobotManualPlacementLayer::setVisibility(bool visible)
{
    mVisible = visible;
}

bool RobotManualPlacementLayer::visible() const
{
    return mVisible;
}

bool RobotManualPlacementLayer::handleEvent(QObject *, QEvent *e)
{
    if (!mVisible || !mReferenceCoords.isValid())
        return false;

    if (e->type() == QEvent::MouseButtonPress
        || e->type() == QEvent::MouseMove
        || e->type() == QEvent::MouseButtonRelease)
    {
        QMouseEvent *event = static_cast<QMouseEvent*>(e);
        qreal mouseLon, mouseLat;
        bool isAboveMap = mMap->geoCoordinates(event->x(), event->y(),
                                               mouseLon, mouseLat, GeoDataCoordinates::Radian);
        if (!isAboveMap)
        {
            mMouseCoords = GeoDataCoordinates();
            mTracking = false;
            return false;
        }

        bool consumed = false;
        if (event->type() == QEvent::MouseButtonPress && event->button() == Qt::LeftButton)
        {
            mTracking = true;
            consumed = true;
        }

        if (mTracking)
        {
            mMouseCoords.setLongitude(mouseLon);
            mMouseCoords.setLatitude(mouseLat);
            emit orientationUpdate(MapLibraryHelpers::transformCoords(mMouseCoords));
        }

        if (event->type() == QEvent::MouseButtonRelease && event->button() == Qt::LeftButton)
        {
            mMouseCoords = GeoDataCoordinates();
            mTracking = false;
            return true;
        }

        return consumed;
    }
    return false;
}

void RobotManualPlacementLayer::setReferencePoint(GeoDataCoordinates coords)
{
    mReferenceCoords = coords;
}


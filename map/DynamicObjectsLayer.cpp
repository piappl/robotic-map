#include <marble/ViewportParams.h>
#include <marble/GeoPainter.h>

#include "DynamicObjectsLayer.h"
#include "LocalMapLayer.h"
#include "MapLibraryHelpers.h"
#include "MapRobotObject.h"

using namespace MapAbstraction;
using namespace Marble;

namespace
{
    const int disappearTime = 5000;
}

DynamicObjectsLayer::DynamicObjectsLayer(LocalMapLayerPtr map)
    : mLocalMap(map), mVisible(false), mHasContent(false)
{
    connect(&mCleanupTimer, SIGNAL(timeout()), this, SLOT(cleanDynamicObjects()));
}

void DynamicObjectsLayer::setVisible(bool visible)
{
    mVisible = visible;
}

bool DynamicObjectsLayer::visible() const
{
    return mVisible;
}

void DynamicObjectsLayer::updateContent(MapAbstraction::DynamicObjects objects,
                                    MapAbstraction::MapRobotObjectConstPtr robot)
{
    mCleanupTimer.stop();
    mDynamicObjects = objects;
    mHasContent = true;
    mRobot = robot;
    mCleanupTimer.start(disappearTime);
    //qDebug("Content set, object count %d", objects.count());
}

void DynamicObjectsLayer::cleanDynamicObjects()
{
    mDynamicObjects.clear();
}

QStringList DynamicObjectsLayer::renderPosition() const
{
    return QStringList() << "HOVERS_ABOVE_SURFACE";
}

qreal DynamicObjectsLayer::zValue() const
{
    const qreal DynamicObjectsLayerZPosition = 3.0;
    return DynamicObjectsLayerZPosition;
}

bool DynamicObjectsLayer::render(GeoPainter *painter, ViewportParams*, const QString&, GeoSceneLayer*)
{
    if (!mVisible || !mHasContent)
        return true;

    if (!mRobot->connected())  //Only display objects for currently active robot
        return true;

    //painter->save();
    //painter->setOpacity(1);

    foreach(DynamicObject o, mDynamicObjects)
    {
        if (o.localizationType != MapObject::Global && (!mLocalMap->visible() || !mLocalMap->hasContent()))
            continue; //No local map to display local objects, skipping

        GeoDataCoordinates marbleCoords = mLocalMap->robotToGlobal(
                    QPointF(o.position.longitude(), o.position.latitude()), mRobot);
        //qDebug("Coords: %f %f", marbleCoords.longitude(), marbleCoords.latitude());
        painter->drawImage(marbleCoords, mIconProvider.getDynamicObjectIcon(DynamicObjectPedestrian));
    }

    //painter->restore();
    return true;
}

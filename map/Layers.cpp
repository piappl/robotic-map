#include "Layers.h"
#include "RegionsLayer.h"
#include "PathsLayer.h"
#include "CrosshairLayer.h"
#include "RobotManualPlacementLayer.h"
#include "DynamicObjectsLayer.h"
#include "SensorDataLayer.h"
#include "LaserCloudLayer.h"
#include "LocalMapLayer.h"
#include "RoboticsMap.h"
#include "WaypointsLayer.h"

using namespace Marble;
using namespace MapAbstraction;

Layers::Layers(RoboticsMap *rm)
{
    PathsLayerPtr paths(new PathsLayer(rm));
    RegionsLayerPtr regions(new RegionsLayer(rm));
    CrosshairLayerPtr crosshair(new CrosshairLayer(rm));
    LocalMapLayerPtr localMap(new LocalMapLayer(rm));
    LaserCloudLayerPtr laserCloud(new LaserCloudLayer(rm, localMap));
    RobotManualPlacementLayerPtr robotPlacement(new RobotManualPlacementLayer(rm));
    DynamicObjectsLayerPtr dynamicObjects(new DynamicObjectsLayer(rm, localMap));
    SensorDataLayerPtr sensorData(new SensorDataLayer(rm));
    WaypointsLayerPtr waypoints(new WaypointsLayer(rm));

    localMap->setVisible(true); //by default
    waypoints->setVisible(true);
    regions->setVisible(true);

    mLayers.insert(LayerPaths, paths);
    mLayers.insert(LayerRegions, regions);
    mLayers.insert(LayerCrosshair, crosshair);
    mLayers.insert(LayerLocalMap, localMap);
    mLayers.insert(LayerLaserCloud, laserCloud);
    mLayers.insert(LayerRobotManualPlacement, robotPlacement);
    mLayers.insert(LayerDynamicObjects, dynamicObjects);
    mLayers.insert(LayerSensorData, sensorData);
    mLayers.insert(LayerWaypoints, waypoints);

    for (auto layer : mLayers.values())
    {
        rm->map()->addLayer(layer.data());
    }
}

MapLayerInterfacePtr Layers::getLayer(LayerType type)
{
    if (!hasLayer(type))
        return MapLayerInterfacePtr();
    return mLayers.value(type);
}

void Layers::setVisibility(LayerType type, bool visible)
{
    if (hasLayer(type))
        mLayers.value(type)->setVisible(visible);
}

void Layers::toggleVisibility(LayerType type)
{
    if (hasLayer(type))
    {
        MapLayerInterfacePtr layer = mLayers.value(type);
        layer->setVisible(!layer->visible());
    }
}

bool Layers::hasLayer(LayerType type) const
{
    return mLayers.contains(type);
}



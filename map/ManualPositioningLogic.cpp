#include <marble/MarbleMap.h>
#include "ManualPositioningLogic.h"
#include "PlacemarkLogic.h"
#include "GeoObjectsManager.h"
#include "MapRobotObject.h"
#include "MapLibraryHelpers.h"
#include "RobotManualPlacementLayer.h"

using namespace MapAbstraction;

ManualPositioningLogic::ManualPositioningLogic(PlacemarkLogicPtr logic,
                 MapAbstraction::GeoObjectsManagerPtr geoManager, Marble::MarbleMap *map,
                 Marble::RobotManualPlacementLayerPtr placementLayer)
    : mPlacemarkLogic(logic), mGeoManager(geoManager), mMarbleMap(map),
      mPlacementLayer(placementLayer), mManualPlacementMode(false)
{
    connect(mPlacementLayer.data(), SIGNAL(orientationUpdate(MapAbstraction::GeoCoords)),
            this, SLOT(orientateTo(MapAbstraction::GeoCoords)));
    mPlacementLayer->setVisibility(false);
}

void ManualPositioningLogic::startPlacement()
{
    startOrContinueManualPlacement();
}

void ManualPositioningLogic::orientationEdit(bool enabled)
{
    mPlacementLayer->setVisibility(enabled);
}

bool ManualPositioningLogic::isInManualPlacementMode(GeoObjectID id) const
{
    return mManualPlacementMode && (mManipulatedObjectID == id);
}

MapRobotObjectPtr ManualPositioningLogic::positionedRobot()
{
    if (!mManipulatedObjectID.isNull())
    {
        MapRobotObjectConstPtr robot = mGeoManager->getMapObjectForID(mManipulatedObjectID)
                .staticCast<MapRobotObject>();
        MapObjectPtr cloned(robot->Clone());
        return cloned.staticCast<MapRobotObject>();
    }
    return MapRobotObjectPtr();
}

void ManualPositioningLogic::startOrContinueManualPlacement()
{   //TODO - change this to selection-based
    if (!mManualPlacementMode)
    {
        GeoObjectID id;
        MapRobotObjectPtr robotObject = mGeoManager->getConnectedRobot(id);
        if (!robotObject)
            return;

        if (robotObject->localizationMode() == MapObject::Manual ||
            robotObject->localizationMode() == MapObject::Assisted)
        {
            mManipulatedObjectID = id;
            MapRobotObjectPtr manipulatedRobot = positionedRobot();
            mOldCoords = manipulatedRobot->coords();
            mOldOrientation = manipulatedRobot->orientation();
            mPlacementLayer->setReferencePoint(MapLibraryHelpers::transformCoords(mOldCoords));
            mManualPlacementMode = true;
        }
    }
}

void ManualPositioningLogic::placeAtCrosshair()
{
    startOrContinueManualPlacement();
    if (!mManualPlacementMode)
        return; //no valid robot

    MapRobotObjectPtr robot = positionedRobot();
    qreal lat = mMarbleMap->centerLatitude();
    qreal lon = mMarbleMap->centerLongitude();
    MapAbstraction::GeoCoords coords(lon, lat);
    robot->setCoords(coords);
    mPlacementLayer->setReferencePoint(MapLibraryHelpers::transformCoords(coords));
    if (robot->localizationType() == MapObject::None)
    {
        robot->setLocalizationType(MapObject::LocalRelative);
    }

    mPlacemarkLogic->addOrUpdatePlacemark(mManipulatedObjectID, robot);
}

void ManualPositioningLogic::orientateToCrosshair()
{
    qreal lat = mMarbleMap->centerLatitude();
    qreal lon = mMarbleMap->centerLongitude();
    orientateTo(GeoCoords(lon, lat));
}

void ManualPositioningLogic::orientateTo(GeoCoords coords)
{
    startOrContinueManualPlacement();
    if (!mManualPlacementMode)
        return; //no valid robot

    MapRobotObjectPtr robot = positionedRobot();
    if (!robot->coords().valid())
        return; //No position yet

    qreal x = coords.longitude() - robot->coords().longitude();
    qreal y = coords.latitude() - robot->coords().latitude();
    robot->setOrientation(atan2(y,x));

    mPlacemarkLogic->addOrUpdatePlacemark(mManipulatedObjectID, robot);
}

void ManualPositioningLogic::finalizePlacement(bool persist)
{
    if (!mManualPlacementMode)
        return;

    mManualPlacementMode = false;
    mPlacementLayer->setVisibility(false);
    MapRobotObjectPtr robot = positionedRobot();
    robot->setOrientation(mOldOrientation);
    robot->setCoords(mOldCoords);

    if (persist)
    {
        emit robotPlacementComplete(mManipulatedObjectID, robot);
    }
    else
    {
        mPlacementLayer->setReferencePoint(MapLibraryHelpers::transformCoords(mOldCoords));
        mPlacemarkLogic->addOrUpdatePlacemark(mManipulatedObjectID, robot);
    }

    mOldOrientation = 0;
    mOldCoords.invalidate();
    GeoObjectID nullID;
    mManipulatedObjectID = nullID;
}

#ifndef MANUALPOSITIONINGLOGIC_H
#define MANUALPOSITIONINGLOGIC_H

#include <QObject>
#include "MapObjectsFwd.h"
#include "InternalTypesFwd.h"
#include "PlacemarkType.h"
#include "MapLibraryTypes.h"
#include "GeoCoords.h"
#include "GeoObjectID.h"
#include "Orientation.h"
#include "RobotManualPlacementLayer.h"

namespace Marble { class MarbleMap; }

class ManualPositioningLogic : public QObject
{
Q_OBJECT
signals:
    void robotPlacementComplete(GeoObjectID id, MapAbstraction::MapRobotObjectPtr robot);

public slots:
    void orientateToCrosshair();
    void orientateTo(MapAbstraction::GeoCoords coords);
    void placeAtCrosshair();
    void finalizePlacement(bool persist);
    void startPlacement();
    void orientationEdit(bool enabled);

public:
    ManualPositioningLogic(PlacemarkLogicPtr logic, MapAbstraction::GeoObjectsManagerPtr geoManager,
                           Marble::MarbleMap *map, MapAbstraction::RobotManualPlacementLayerPtr placementLayer);

    bool isInManualPlacementMode(GeoObjectID id) const;
    MapAbstraction::MapRobotObjectPtr positionedRobot(); //operates on a clone of robot object to use the comparison mechanism later

private:
    void startOrContinueManualPlacement();

    PlacemarkLogicPtr mPlacemarkLogic;
    MapAbstraction::GeoObjectsManagerPtr mGeoManager;
    Marble::MarbleMap *mMarbleMap;
    MapAbstraction::RobotManualPlacementLayerPtr mPlacementLayer;

    bool mManualPlacementMode;
    GeoObjectID mManipulatedObjectID;

    MapAbstraction::GeoCoords mOldCoords;
    MapAbstraction::Orientation mOldOrientation;
};


#endif // MANUALPOSITIONINGLOGIC_H

#ifndef MAPPLACESMANAGER_H
#define MAPPLACESMANAGER_H

#include <QObject>
#include "InternalTypesFwd.h"
#include "PlacemarkType.h"
#include "MapLibraryTypes.h"
#include "MapAbstractionFwd.h"
#include "GeoCoords.h"
#include "GeoObjectID.h"
#include "MapLayersFwd.h"

namespace Marble { class MarbleMap; }

class MapPlacesManager : public QObject
{
Q_OBJECT
signals:
    void selectionModeChanged(bool selectionMode);
    void mapPathCreated(int targetRobotID, MapAbstraction::MapPath path);

public slots:
    void addAtCenter(bool orderBeforeSelected);
    void removeAtCenter();
    void removeAll();
    void activeRobotPositionChanged(GeoObjectID robotID);
    void editModeChanged(PlacemarkType type, bool on);
    void selectPlacemark(int placemarkID);
    void finalizeMapEdit(bool accept);

public:
    MapPlacesManager(PlacemarkLogicPtr logic, MapAbstraction::GeoObjectsManagerPtr geoManager,
                     Marble::LocalMapLayerPtr localMapLayer, Marble::MarbleMap *map);
    PlacemarkType currentEditMode() const;
    void selectPlacemarkRequest(int x, int y);

private:
    PlacemarkType placemarkCategory(PlacemarkConstPtr placemark) const;
    int firstFreeNum(PlacemarkType type) const;
    void renumberForward(PlacemarkType type, int from);
    MapAbstraction::MapObjectPtr makePlace(MapAbstraction::GeoCoords coords, PlacemarkType type) const;
    bool leadingPlacemarkSelected() const;
    void recalculate(PlacemarkType type);
    void removeSelection();
    void selectPlacemark(PlacemarkPtr place);
    QVector<PlacemarkConstPtr> placemarksAtCenter(bool filterByType = true) const;
    QVector<PlacemarkConstPtr> placemarksAtPoint(int x, int y, bool filterByType = true) const;

    PlacemarkLogicPtr mPlacemarkLogic;
    MapAbstraction::GeoObjectsManagerPtr mGeoManager;
    Marble::LocalMapLayerPtr mLocalMapLayer;
    Marble::MarbleMap *mMarbleMap;
    PlacemarkType mCurrentEditMode;
};

#endif // MAPPLACESMANAGER_H

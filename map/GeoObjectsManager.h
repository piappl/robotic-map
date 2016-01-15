#ifndef GEOOBJECTSMANAGER_H
#define GEOOBJECTSMANAGER_H

#include <QSharedPointer>
#include <QMap>
#include "MapAbstractionFwd.h"
#include "MapLibraryTypes.h"
#include "GeoObjectID.h"
#include "PlacemarkType.h"
#include "InternalTypesFwd.h"

//Issues:
// 1) Placemarks are embedded in other documents, but held on placemark level here (inconsequent).
//
namespace MapAbstraction
{
    typedef QList<PlacemarkPtr> PlacemarkList;
    typedef QVector<PlacemarkConstPtr> PlacemarkVector;
    typedef QMap<int, PlacemarkPtr> OrderedPoints;
    typedef QMap<GeoObjectID, PlacemarkPtr> PlacemarkMap;

    typedef QPair<GeoObjectID, MapObjectPtr> PlacemarkObjectData;
    typedef QMap<PlacemarkConstPtr, PlacemarkObjectData> PlacemarkDict;

    class GeoObjectsManager
    {
    public:
        bool hasPlacemark(const GeoObjectID &id) const;
        PlacemarkPtr getPlacemark(const GeoObjectID &id) const;
        void removePlacemark(const GeoObjectID &id);
        GeoObjectID findPlacemark(PlacemarkConstPtr place) const;
        PlacemarkVector registeredRobots() const;
        OrderedPoints orderedPoints(PlacemarkType type) const;

        void addMapObject(const GeoObjectID &id, PlacemarkPtr place, MapObjectPtr object);
        void updateMapObject(PlacemarkPtr place, MapObjectPtr object);
        MapObjectPtr getMapObjectForPlacemark(PlacemarkConstPtr place);
        MapObjectPtr getMapObjectForID(const GeoObjectID &id);

        MapRobotObjectPtr getConnectedRobot(GeoObjectID &id) const;
        MapRobotObjectPtr getConnectedRobot() const;
        bool isAnyRobotConnected() const;
        int connectedRobotID() const;
        void setConnectedRobotID(int id);
        void setNoRobotConnected();

        MapRobotObjectPtr getRobot(int robotID) const;
        MapRobotObjectPtr getRobot(int robotID, GeoObjectID &id) const;

        //Lists to iterate
        PlacemarkList placemarks() const;

        void setSelectedPlacemark(PlacemarkConstPtr place);
        PlacemarkConstPtr selectedPlacemark() const;
        bool isSelected(PlacemarkConstPtr place) const;

    private:
        int mConnectedRobotID;
        PlacemarkConstPtr mSelectedPlacemark;
        PlacemarkVector mRegisteredRobots;
        PlacemarkMap mPlacemarkMap;
        PlacemarkDict mPlacemarkDict;
    };
 }

#endif // GEOOBJECTSMANAGER_H

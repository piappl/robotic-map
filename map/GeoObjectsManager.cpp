#include "MapRobotObject.h"
#include "MapWaypointObject.h"
#include "GeoObjectsManager.h"

namespace MapAbstraction
{
    const int NoConnectedRobotID = -1;

    GeoObjectsManager::GeoObjectsManager() : mConnectedRobotID(NoConnectedRobotID), mSelectedPlacemark(0)
    {

    }

    bool GeoObjectsManager::hasPlacemark(const GeoObjectID &id) const
    {
        return mPlacemarkMap.contains(id);
    }

    PlacemarkList GeoObjectsManager::placemarks() const
    {
        return mPlacemarkMap.values();
    }

    GeoObjectID GeoObjectsManager::findPlacemark(PlacemarkConstPtr place) const
    {
        if (mPlacemarkDict.contains(place))
        {
            return mPlacemarkDict.value(place).first;
        }
        GeoObjectID empty;
        return empty;
    }

    PlacemarkPtr GeoObjectsManager::getPlacemark(const GeoObjectID &id) const
    {
        Q_ASSERT(hasPlacemark(id));
        return mPlacemarkMap.value(id);
    }

    void GeoObjectsManager::removePlacemark(const GeoObjectID &id)
    {
        Q_ASSERT(hasPlacemark(id));
        if (!hasPlacemark(id))
            return;

        PlacemarkPtr placemark = getPlacemark(id);
        mPlacemarkDict.remove(placemark);
        mPlacemarkMap.remove(id);

        if (mRegisteredRobots.contains(placemark))
            mRegisteredRobots.remove(mRegisteredRobots.indexOf(placemark));
    }

    void GeoObjectsManager::addMapObject(const GeoObjectID &id,
                                         PlacemarkPtr place,
                                         MapObjectPtr object)
    {
        Q_ASSERT(!hasPlacemark(id));
        mPlacemarkMap.insert(id, place);

        //We maintain a copy of data for comparision purposes
        MapObjectPtr snapshot(object->Clone());
        mPlacemarkDict.insert(place, PlacemarkObjectData(id, snapshot));

        if (object->category() == PlacemarkRobot)
            mRegisteredRobots.push_back(place);
    }

    MapObjectPtr GeoObjectsManager::getMapObjectForPlacemark(PlacemarkConstPtr place)
    {
        Q_ASSERT(mPlacemarkDict.contains(place));
        return mPlacemarkDict.value(place).second;
    }

    MapObjectPtr GeoObjectsManager::getMapObjectForID(const GeoObjectID &id)
    {
        return (getMapObjectForPlacemark(getPlacemark(id)));
    }

    void GeoObjectsManager::updateMapObject(PlacemarkPtr place, MapObjectPtr object)
    {
        GeoObjectID id = findPlacemark(place);
        if (id.isNull())
            return;

        MapObjectPtr data(object->Clone());
        mPlacemarkDict.insert(place, PlacemarkObjectData(id, data));
    }

    PlacemarkVector GeoObjectsManager::registeredRobots() const
    {
        return mRegisteredRobots;
    }

    OrderedPoints GeoObjectsManager::orderedPoints(PlacemarkType type) const
    {
        QMap<int, PlacemarkPtr> out;
        foreach(PlacemarkPtr placemark, placemarks())
        {
            MapObjectPtr mapObject = mPlacemarkDict.value(placemark).second;
            if (mapObject->category() == type)
            {
                MapOrderedObjectPtr orderedPoint = mapObject.staticCast<MapOrderedObject>();
                out.insert(orderedPoint->number(), placemark);
            }
        }
        return out;
    }

    void GeoObjectsManager::setSelectedPlacemark(PlacemarkConstPtr place)
    {
        mSelectedPlacemark = place;
    }

    PlacemarkConstPtr GeoObjectsManager::selectedPlacemark() const
    {
        return mSelectedPlacemark;
    }

    bool GeoObjectsManager::isSelected(PlacemarkConstPtr place) const
    {
        return place == mSelectedPlacemark;
    }

    MapRobotObjectPtr GeoObjectsManager::getConnectedRobot() const
    {
        GeoObjectID unneeded;
        return getConnectedRobot(unneeded);
    }

    MapRobotObjectPtr GeoObjectsManager::getConnectedRobot(GeoObjectID &id) const
    {
        if (!isAnyRobotConnected())
            return MapRobotObjectPtr();

        return getRobot(connectedRobotID(), id);
    }

    MapRobotObjectPtr GeoObjectsManager::getRobot(int robotID) const
    {
        GeoObjectID unused;
        return getRobot(robotID, unused);
    }

    MapRobotObjectPtr GeoObjectsManager::getRobot(int robotID, GeoObjectID &id) const
    {   //TODO - optimize!
        foreach(PlacemarkConstPtr placemark, mRegisteredRobots)
        {
            MapObjectPtr mapObject = mPlacemarkDict.value(placemark).second;
            if (mapObject->category() == PlacemarkRobot)
            {
                MapRobotObjectPtr robot = mapObject.staticCast<MapRobotObject>();
                if (robot->robotID() == robotID)
                {
                    id = findPlacemark(placemark);
                    return robot;
                }
            }
        }
        return MapRobotObjectPtr();
    }

    bool GeoObjectsManager::isAnyRobotConnected() const
    {
        return connectedRobotID() != NoConnectedRobotID;
    }

    int GeoObjectsManager::connectedRobotID() const
    {
        return mConnectedRobotID;
    }

    void GeoObjectsManager::setConnectedRobotID(int id)
    {
        mConnectedRobotID = id;
    }

    void GeoObjectsManager::setNoRobotConnected()
    {
        mConnectedRobotID = NoConnectedRobotID;
    }
}

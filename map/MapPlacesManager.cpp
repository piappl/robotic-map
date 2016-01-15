#include <QVector>
#include <marble/MarbleMap.h>
#include <marble/ViewportParams.h>
#include <marble/GeoDataLineString.h>
#include "LocalMapLayer.h"
#include "MapPlaceObject.h"
#include "MapWaypointObject.h"
#include "MapPolygonNodeObject.h"
#include "MapRobotObject.h"
#include "MapPlacesManager.h"
#include "MapLibraryHelpers.h"
#include "PlacemarkLogic.h"
#include "GeoObjectID.h"
#include "GeoObjectsManager.h"

using namespace MapAbstraction;

//TODO Selection logic should be separated, maybe use marble model (?)
namespace
{
    const bool deleteWaypointOnReach = true;
}

MapPlacesManager::MapPlacesManager(PlacemarkLogicPtr logic, MapAbstraction::GeoObjectsManagerPtr geoManager,
                                   Marble::LocalMapLayerPtr localMapLayer, Marble::MarbleMap *map)
    : mPlacemarkLogic(logic), mGeoManager(geoManager), mLocalMapLayer(localMapLayer), mMarbleMap(map),
      mCurrentEditMode(PlacemarkNone)
{
}

bool MapPlacesManager::leadingPlacemarkSelected() const
{
    if (mCurrentEditMode != PlacemarkWaypoint
            && mCurrentEditMode != PlacemarkPolygonNode)
    {
        return false;
    }

    PlacemarkConstPtr selectedOne = mGeoManager->selectedPlacemark();
    if (!selectedOne)
        return false;

    MapObjectPtr object = mGeoManager->getMapObjectForPlacemark(selectedOne);
    if (!object)
        return false;

    if (object->category() != mCurrentEditMode)
        return false;

    return true;
}

QVector<PlacemarkConstPtr> MapPlacesManager::placemarksAtCenter(bool filterByType) const
{
    QSize viewport = mMarbleMap->viewport()->size();
    return placemarksAtPoint(viewport.width()/2, viewport.height()/2, filterByType);
}

QVector<PlacemarkConstPtr> MapPlacesManager::placemarksAtPoint(int x, int y, bool filterByType) const
{
    QVector<const Marble::GeoDataFeature*> allPlacemarksAtCenter = mMarbleMap->whichFeatureAt(QPoint(x, y));
    QVector<PlacemarkConstPtr> placemarks;
    foreach (const Marble::GeoDataFeature* feature, allPlacemarksAtCenter)
    {
        PlacemarkConstPtr placemark = dynamic_cast<PlacemarkConstPtr>(feature);
        if (placemark)
        {
            MapObjectPtr mapObject = mGeoManager->getMapObjectForPlacemark(placemark);
            if (mapObject && (!filterByType ||
                              mCurrentEditMode == PlacemarkNone ||
                              mapObject->category() == mCurrentEditMode))
            {
                placemarks.append(placemark);
            }
        }
    }
    return placemarks;
}

void MapPlacesManager::addAtCenter(bool orderBeforeSelected)
{
    if (placemarksAtCenter(mMarbleMap).empty())
    {
        if (orderBeforeSelected && leadingPlacemarkSelected())
        {
            PlacemarkConstPtr selectedOne = mGeoManager->selectedPlacemark();
            MapObjectPtr object = mGeoManager->getMapObjectForPlacemark(selectedOne);
            if (object->category() == PlacemarkWaypoint
                    || object->category() == PlacemarkPolygonNode)
            {
                MapOrderedObjectPtr ordered = object.staticCast<MapOrderedObject>();
                renumberForward(mCurrentEditMode, ordered->number());
            }
        }

        qreal lat = mMarbleMap->centerLatitude();
        qreal lon = mMarbleMap->centerLongitude();
        MapAbstraction::GeoCoords coords(lon, lat);
        MapObjectPtr place = makePlace(coords, mCurrentEditMode);
        GeoObjectID id = GeoReferenceFactory::createGeoObjectId();
        mPlacemarkLogic->addOrUpdatePlacemark(id, place);
    }
}

void MapPlacesManager::removeAtCenter()
{
    QVector<PlacemarkConstPtr> placemarks = placemarksAtCenter();
    foreach (PlacemarkConstPtr placemark, placemarks)
    {
        if (placemark == mGeoManager->selectedPlacemark())
            removeSelection();
        mPlacemarkLogic->removePlacemark(placemark);
        recalculate(mCurrentEditMode);
        return;
    }
}

void MapPlacesManager::removeAll()
{
    removeSelection();
    mPlacemarkLogic->removeAllPlacemarks(mCurrentEditMode);
}

void MapPlacesManager::finalizeMapEdit(bool accept)
{
    if (accept)
    {   //Send waypoints
        MapRobotObjectPtr activeRobot = mGeoManager->getConnectedRobot();

        if (!activeRobot)
            return;

        MapPath path;
        OrderedPoints currentPoints = mGeoManager->orderedPoints(PlacemarkWaypoint);
        foreach(PlacemarkPtr placemark, currentPoints.values())
        {
            MapObjectPtr mapObject = mGeoManager->getMapObjectForPlacemark(placemark);
            if (mapObject->category() == PlacemarkWaypoint)
            {
                MapObject *cloned = mapObject->Clone();
                MapWaypointObjectPtr clonedWaypoint(dynamic_cast<MapWaypointObject*>(cloned));
                if (mLocalMapLayer->hasContent())
                {  //TODO - cut off nodes outside
                   QPointF localCoords;
                   /*bool inMap = */

                   if (activeRobot->localizationType() == MapObject::LocalAbsolute)
                   {
                       mLocalMapLayer->globalToLocal(MapLibraryHelpers::transformCoords(clonedWaypoint->coords()), localCoords);
                   }
                   else //Local Relative
                   {
                       mLocalMapLayer->globalToRobot(MapLibraryHelpers::transformCoords(
                                                     clonedWaypoint->coords()), activeRobot, localCoords);
                   }

                   clonedWaypoint->setCoords(GeoCoords(localCoords.x(), localCoords.y()));
                   //qDebug("Waypoint coords [%f, %f]", localCoords.x(), localCoords.y());
                }
                path.append(clonedWaypoint);
            }
        }

        if (path.size() > 0)
        {
            qDebug("emit path - waypoints are in robot frame of reference coords");
            emit mapPathCreated(activeRobot->robotID(), path);
        }
    }
}

PlacemarkType MapPlacesManager::placemarkCategory(PlacemarkConstPtr placemark) const
{
   MapObjectPtr mapObject = mGeoManager->getMapObjectForPlacemark(placemark);
   Q_ASSERT(mapObject);
   if (mapObject)
   {
       return mapObject->category();
   }
   qWarning("Placemark data not found");
   return PlacemarkNone;
}

void MapPlacesManager::recalculate(PlacemarkType type)
{
    if (type == PlacemarkWaypoint || type == PlacemarkPolygonNode)
    {
        QMap<int, PlacemarkPtr> orderedPointsMap = mGeoManager->orderedPoints(type);
        QList<int> numbers = orderedPointsMap.keys();
        int order = 1;
        foreach (int num, numbers)
        {
            PlacemarkPtr placemark = orderedPointsMap.value(num);
            MapOrderedObjectPtr current = mGeoManager->getMapObjectForPlacemark(placemark)
                    .staticCast<MapOrderedObject>();
            if (num != order)
            {
                MapObject *cloned = current->Clone();
                MapOrderedObjectPtr clonedOrdered(dynamic_cast<MapOrderedObject*>(cloned));
                clonedOrdered->setNumber(order);
                mPlacemarkLogic->updatePlacemark(clonedOrdered, placemark);
            }
            order++;
        }
    }
}

void MapPlacesManager::renumberForward(PlacemarkType type, int from)
{
    if (type != PlacemarkWaypoint && type != PlacemarkPolygonNode)
        return;

    OrderedPoints currentPoints = mGeoManager->orderedPoints(type);
    foreach(PlacemarkPtr placemark, currentPoints.values())
    {
        MapObjectPtr mapObject = mGeoManager->getMapObjectForPlacemark(placemark);
        if (mapObject->category() == type)
        {
            MapOrderedObjectPtr orderedPoint = mapObject.staticCast<MapOrderedObject>();
            if (orderedPoint->number() >= from)
            {
                MapObject *cloned = orderedPoint->Clone();
                MapOrderedObjectPtr clonedOrdered(dynamic_cast<MapOrderedObject*>(cloned));
                clonedOrdered->setNumber(orderedPoint->number()+1);
                mPlacemarkLogic->updatePlacemark(clonedOrdered, placemark);
            }
        }
    }
}

int MapPlacesManager::firstFreeNum(PlacemarkType type) const
{
    if (type != PlacemarkWaypoint && type != PlacemarkPolygonNode)
        return 0;

    QList<int> used = mGeoManager->orderedPoints(type).keys();
    if (used.isEmpty())
        return 1;

    int free = 1;
    foreach (int num, used)
    {
        if (num != free)
            return free;

        free++;
    }
    return free;
}

MapAbstraction::MapObjectPtr MapPlacesManager::makePlace(MapAbstraction::GeoCoords coords,
                                                         PlacemarkType type) const
{
    switch (type)
    {
        case PlacemarkPlace:
            return MapPlaceObjectPtr(new MapPlaceObject(coords, QString("place"), QString()));
        case PlacemarkWaypoint:
            return MapWaypointObjectPtr(new MapWaypointObject(coords, firstFreeNum(PlacemarkWaypoint)));
        case PlacemarkPolygonNode:
            return MapPolygonNodeObjectPtr(new MapPolygonNodeObject(coords, firstFreeNum(PlacemarkPolygonNode)));
        default:
            qWarning("Bad type");
            return MapPlaceObjectPtr();
    }
}

void MapPlacesManager::activeRobotPositionChanged(GeoObjectID robotID)
{
    const int reachedTreshold = 1; //meters

    PlacemarkPtr placemark = mGeoManager->getPlacemark(robotID);
    MapObjectPtr mapObject = mGeoManager->getMapObjectForPlacemark(placemark);
    Marble::GeoDataCoordinates robotCoords = MapLibraryHelpers::transformCoords(mapObject->coords());

    OrderedPoints waypoints = mGeoManager->orderedPoints(PlacemarkWaypoint);
    QList<PlacemarkPtr> values = waypoints.values();
    foreach (PlacemarkPtr waypoint, values)
    {
        MapWaypointObjectPtr current = mGeoManager->getMapObjectForPlacemark(waypoint)
                .staticCast<MapWaypointObject>();
        if (current->reached())
            continue;

        Marble::GeoDataLineString line;
        line.append(waypoint->coordinate());
        line.append(robotCoords);
        qreal distance = line.length(Marble::EARTH_RADIUS);
        if (distance <= reachedTreshold)
        {
            if (deleteWaypointOnReach)
            {
                mPlacemarkLogic->removePlacemark(waypoint);
                continue;
            }
            else
            {
                MapObject *cloned = current->Clone();
                MapWaypointObjectPtr clonedWaypoint(dynamic_cast<MapWaypointObject*>(cloned));
                clonedWaypoint->setReached(true);
                mPlacemarkLogic->updatePlacemark(clonedWaypoint, waypoint);
            }
        }
    }
}

void MapPlacesManager::editModeChanged(PlacemarkType type, bool on)
{
    if (mCurrentEditMode == type && on)
        return;

    mCurrentEditMode = PlacemarkNone;
    removeSelection();
    if (on)
    {
        mCurrentEditMode = type;
    }
}

PlacemarkType MapPlacesManager::currentEditMode() const
{
    return mCurrentEditMode;
}

void MapPlacesManager::removeSelection()
{
    PlacemarkConstPtr oldSelected = mGeoManager->selectedPlacemark();
    if (oldSelected)
    {
        mGeoManager->setSelectedPlacemark(PlacemarkConstPtr());
        mPlacemarkLogic->updatePlacemarkIcon(const_cast<PlacemarkPtr>(oldSelected));
        emit selectionModeChanged(false);
    }
}

void MapPlacesManager::selectPlacemarkRequest(int x, int y)
{
    QVector<PlacemarkConstPtr> placemarks = placemarksAtPoint(x, y);
    removeSelection();

    if (placemarks.isEmpty())
        return;

    PlacemarkPtr place = const_cast<PlacemarkPtr>(placemarks.first());
    selectPlacemark(place);
}

void MapPlacesManager::selectPlacemark(int placemarkID)
{
    GeoObjectID id = GeoObjectID::fromInt(placemarkID);
    PlacemarkPtr placemark = mGeoManager->getPlacemark(id);
    if (placemark)
        selectPlacemark(placemark);
}

void MapPlacesManager::selectPlacemark(PlacemarkPtr place)
{
    removeSelection();

    if (mCurrentEditMode != PlacemarkWaypoint && mCurrentEditMode != PlacemarkPolygonNode)
        return;

    MapObjectPtr object = mGeoManager->getMapObjectForPlacemark(place);
    if (!object)
        return;

    if (mCurrentEditMode == object->category())
    {
        mGeoManager->setSelectedPlacemark(place);
        mPlacemarkLogic->updatePlacemarkIcon(place);
        emit selectionModeChanged(true);
    }
}






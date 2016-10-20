#include <QQuickItem>
#include <marble/MarbleModel.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/GeoDataStyle.h>
#include <marble/GeoDataIconStyle.h>
#include <marble/GeoDataLabelStyle.h>
#include <QPainter>
#include "GeoObjectsManager.h"
#include "MapRobotObject.h"
#include "MapWaypointObject.h"
#include "MapPolygonNodeObject.h"
#include "MapLibraryHelpers.h"
#include "MapIconProvider.h"
#include "MapLogPlacemarkData.h"
#include "PlacemarkLogic.h"

using namespace Marble;
using namespace MapAbstraction;

namespace
{
    QString iconPath(PlacemarkPtr placemark)
    {
        return placemark->style()->iconStyle().iconPath();
    }

    GeoDataLabelStyle::Alignment getAlignment()
    {
        return GeoDataLabelStyle::Right;
    }

    SinglePointDataPtr createDataPoint(const MapObjectPtr object)
    {
        SinglePointData *d = new SinglePointData(object->coords());
        SinglePointDataPtr timePointData(d);
        return timePointData;
    }

    bool isLoggedElement(const MapObjectPtr object)
    {
        return object->category() == PlacemarkRobot;
    }

    bool isPartOfOverview(const MapObjectPtr object)
    {
        return object->category() == PlacemarkRobot ||
               object->category() == PlacemarkPlace;
    }

    bool hasIconChanged(const MapObjectPtr newObject, const MapObjectPtr oldObject, bool simplified)
    {
        if (newObject->category() == PlacemarkRobot)
        {
            MapRobotObjectPtr newRobot = newObject.staticCast<MapRobotObject>();
            MapRobotObjectPtr oldRobot = oldObject.staticCast<MapRobotObject>();
            if (newRobot->state() != oldRobot->state())
                return true;

            if (simplified)
            {
                return (oldRobot->orientation() != newRobot->orientation());
            }
        }

        if (newObject->category() == PlacemarkWaypoint)
        {
            MapWaypointObjectPtr newWaypoint = newObject.staticCast<MapWaypointObject>();
            MapWaypointObjectPtr oldWaypoint = oldObject.staticCast<MapWaypointObject>();
            return (oldWaypoint->number() != newWaypoint->number()
                    || oldWaypoint->reached() != newWaypoint->reached());
        }

        if (newObject->category() == PlacemarkPolygonNode)
        {
            MapPolygonNodeObjectPtr newNode = newObject.staticCast<MapPolygonNodeObject>();
            MapPolygonNodeObjectPtr oldNode = oldObject.staticCast<MapPolygonNodeObject>();
            return oldNode->number() != newNode->number();
        }

        return false;
    }
}

PlacemarkLogic::PlacemarkLogic(MarbleModel *model, GeoObjectsManagerPtr manager)
    : mPlacemarksVisible(true), mIconsSimplified(false),
      mMapLog(new MapLogPlacemarkData()),
      mMarbleModel(model), mGeoManager(manager)
{
    mPlacemarksVisibilityMap.insert(PlacemarkRobot, true);
    mPlacemarksVisibilityMap.insert(PlacemarkPlace, true);
    mPlacemarksVisibilityMap.insert(PlacemarkWaypoint, true);
    mPlacemarksVisibilityMap.insert(PlacemarkPolygonNode, false);
}

void PlacemarkLogic::addOrUpdatePlacemark(const GeoObjectID &id, MapObjectPtr localizedData)
{
    if (mGeoManager->hasPlacemark(id))
    {
        PlacemarkPtr placemark = mGeoManager->getPlacemark(id);
        updatePlacemark(localizedData, placemark);
    }
    else
    {
        PlacemarkPtr placemark = createPlacemark(id, localizedData);
        updatePlacemark(localizedData, placemark, false);
    }

    if (isLoggedElement(localizedData))
    {
        mMapLog->addData(id, createDataPoint(localizedData));
    }
}

PlacemarkPtr PlacemarkLogic::createPlacemark(const GeoObjectID &id, MapObjectPtr localizedData)
{
    PlacemarkPtr placemark = new GeoDataPlacemark();
    //it will be later shown in updatePlacemark, to have consistency with updates
    placemark->setVisible(false);

    GeoDataDocument *doc = new GeoDataDocument();
    doc->append(placemark);
    mMarbleModel->treeModel()->addDocument(doc);

    mGeoManager->addMapObject(id, placemark, localizedData);
    createPlacemarkStyle(placemark);
    return placemark;
}

void PlacemarkLogic::updatePlacemark(MapObjectPtr localizedData,
                                     PlacemarkPtr placemark,
                                     bool checkChanges)
{
    bool iconChanged = true;
    bool positionChanged = true;
    bool displayNameChanged = true;
    if (checkChanges)
    {
        MapObjectPtr oldData = mGeoManager->getMapObjectForPlacemark(placemark);
        if (*oldData == *localizedData)
            return;

        bool baseIconSame = mIconProvider.getIconPath(oldData) == mIconProvider.getIconPath(localizedData);
        iconChanged = baseIconSame ? hasIconChanged(localizedData, oldData, mIconsSimplified) : true;
        positionChanged = !(oldData->coords() == localizedData->coords());
        displayNameChanged = !(oldData->displayText() == localizedData->displayText());
    }

    if (localizedData->category() == PlacemarkRobot)
    {
        MapRobotObjectPtr robot = localizedData.staticCast<MapRobotObject>();
        bool updateConnection = true;
        if (checkChanges)
        {
            MapRobotObjectPtr oldRobot = mGeoManager->getMapObjectForPlacemark(placemark)
                    .staticCast<MapRobotObject>();
            if (oldRobot)
            {
                updateConnection = oldRobot->connected() != robot->connected();
            }
        }

        if (updateConnection)
        {
            QSharedPointer<GeoDataStyle> style = placemark->style().constCast<GeoDataStyle>();
            style->labelStyle().setGlow(robot->connected());
        }
    }

    if (displayNameChanged)
        updatePlacemarkName(placemark, localizedData);

    if (positionChanged)
        updatePlacemarkPosition(placemark, localizedData);

    if (iconChanged)
        updatePlacemarkIcon(placemark, localizedData);

    updatePlacemarkVisibility(placemark, localizedData);

    //qDebug("PlacemarkLogic - updatePlacemarkIcon (update placemark)");
    mGeoManager->updateMapObject(placemark, localizedData);
    updateMarblePlacemark(placemark);

    QString nonSimplifiedGuiIcon = mIconProvider.getIconPath(localizedData);
    emit placemarkUpdated(mGeoManager->findPlacemark(placemark), localizedData, nonSimplifiedGuiIcon);
}

bool PlacemarkLogic::shouldPlaceBeVisibleOnList(MapObjectPtr place) const
{
    bool basicVisibility = mPlacemarksVisibilityMap.value(place->category()) && mPlacemarksVisible && place->visible();
    if (place->category() == PlacemarkRobot)
    {
        MapRobotObjectPtr robot = place.staticCast<MapRobotObject>();
        return robot->active() && basicVisibility;
    }
    else
        return basicVisibility;
}

bool PlacemarkLogic::shouldPlaceBeVisibleOnMap(MapObjectPtr place) const
{
    bool should = shouldPlaceBeVisibleOnList(place);
    return should && place->coords().valid();
}

void PlacemarkLogic::removePlacemark(PlacemarkConstPtr placemark)
{   //if not found, it will return a null id so nothing will be removed
    return removePlacemark(mGeoManager->findPlacemark(placemark));
}

void PlacemarkLogic::removePlacemark(const GeoObjectID &id)
{
    if (!mGeoManager->hasPlacemark(id))
        return;

    PlacemarkPtr placemark = mGeoManager->getPlacemark(id);

    if (mGeoManager->selectedPlacemark() == placemark)
    {
        mGeoManager->setSelectedPlacemark(PlacemarkConstPtr());
    }
    mGeoManager->removePlacemark(id);
    mMarbleModel->treeModel()->removeFeature(placemark);
    emit placemarkRemoved(id);
}

void PlacemarkLogic::removeAllPlacemarks(PlacemarkType category)
{
    foreach(PlacemarkPtr placemark, mGeoManager->placemarks())
    {
        MapObjectPtr mapObject = mGeoManager->getMapObjectForPlacemark(placemark);
        if (mapObject->category() == category)
        {
            removePlacemark(placemark);
        }
    }
}

void PlacemarkLogic::updatePlacemarkIcon(PlacemarkPtr placemark)
{
    if (!placemark)
        return;

    MapObjectPtr object = mGeoManager->getMapObjectForPlacemark(placemark);
    if (!object)
        return;

    updatePlacemarkIcon(placemark, object);
    //qDebug("PlacemarkLogic - updatePlacemarkIcon");
    updateMarblePlacemark(placemark);

    QString nonSimplifiedGuiIcon = mIconProvider.getIconPath(object);
    emit placemarkUpdated(mGeoManager->findPlacemark(placemark), object, nonSimplifiedGuiIcon);
}

void PlacemarkLogic::updatePlacemarkIcon(PlacemarkPtr placemark, MapObjectPtr newInfo)
{
    bool isSelected = mGeoManager->isSelected(placemark);
    QImage newIcon = mIconProvider.getIcon(newInfo, mIconsSimplified, isSelected);
    QString newIconPath = mIconProvider.getIconPath(newInfo, mIconsSimplified, isSelected);
    updatePlacemarkIcon(placemark, newIconPath, newIcon);
}

void PlacemarkLogic::updatePlacemarkIcon(PlacemarkPtr placemark, QString newIconPath, QImage newIcon)
{
    QSharedPointer<GeoDataStyle> style = placemark->style().constCast<GeoDataStyle>();
    QString oldPath = style->iconStyle().iconPath();
    style->iconStyle().setIconPath(newIconPath);
    style->iconStyle().setIcon(newIcon);
    style->labelStyle().setAlignment(getAlignment());

    //qDebug("Style path icon: %s", qPrintable(placemark->style()->iconStyle().iconPath()));
}

void PlacemarkLogic::updatePlacemarkPosition(PlacemarkPtr placemark, MapObjectPtr newInfo)
{
    GeoCoords coords = newInfo->coords();
    GeoDataCoordinates newPosition = MapLibraryHelpers::transformCoords(coords);
    placemark->setCoordinate(newPosition);

    if (newInfo->category() == PlacemarkRobot)
    {
        MapRobotObjectPtr robot = newInfo.staticCast<MapRobotObject>();
        if (robot->connected())
        {
            emit activeRobotPositionChanged(mGeoManager->findPlacemark(placemark));
        }
    }
}

void PlacemarkLogic::updatePlacemarkVisibility(PlacemarkPtr placemark, MapObjectPtr newInfo)
{
    bool listVisible = shouldPlaceBeVisibleOnList(newInfo);
    bool mapVisible = shouldPlaceBeVisibleOnMap(newInfo);
    newInfo->setVisible(listVisible);
    placemark->setVisible(mapVisible);
}

void PlacemarkLogic::updatePlacemarkName(PlacemarkPtr placemark, MapObjectPtr newInfo)
{
    placemark->setName(newInfo->displayText());
    placemark->setDescription(newInfo->description());
}

void PlacemarkLogic::createPlacemarkStyle(PlacemarkPtr placemark)
{   //initial style settings. In Marble, styles are independent of placemark,
    //their memory allocation and release is external to placemark class.
    QSharedPointer<GeoDataStyle> style(new GeoDataStyle(*placemark->style()));
    mStyles.append(style);

    QFont font;
    font.setBold(true);
    style->labelStyle().setColor(Qt::red);
    style->labelStyle().setGlow(false);
    style->labelStyle().setFont(font);
    placemark->setStyle(style);
}

void PlacemarkLogic::placemarkTypeVisibilityRequest(PlacemarkType type, bool makeVisible)
{
    if (makeVisible || type == PlacemarkPolygonNode)
    {
        changeVisibility(type, makeVisible);
    }
}

void PlacemarkLogic::changeVisibility(bool visible)
{
    changeVisibility(visible, mPlacemarksVisible);
}

void PlacemarkLogic::changeVisibility(PlacemarkType type, bool visible)
{
    mPlacemarksVisibilityMap[type] = visible;
    applyVisibility();
}

void PlacemarkLogic::applyVisibility()
{
    foreach (PlacemarkPtr placemark, mGeoManager->placemarks())
    {
        MapObjectPtr object = mGeoManager->getMapObjectForPlacemark(placemark);
        if (object.isNull())
            continue;

        object->setVisible(true); //I am visible until proven otherwise
        if (object->category() == PlacemarkRobot)
        {
            MapRobotObjectPtr data = object.staticCast<MapRobotObject>();

            QString path = mIconProvider.getIconPath(data, mIconsSimplified);
            QImage iconImage = mIconProvider.getIcon(data, mIconsSimplified, mGeoManager->isSelected(placemark));

            updatePlacemarkIcon(placemark, path, iconImage);
            updatePlacemarkVisibility(placemark, object);
            QString nonSimplifiedGuiIcon = mIconProvider.getIconPath(object);
            emit placemarkUpdated(mGeoManager->findPlacemark(placemark), object, nonSimplifiedGuiIcon);
        }
        else
        {
            updatePlacemarkVisibility(placemark, object);
            emit placemarkUpdated(mGeoManager->findPlacemark(placemark), object, QString());
        }
        updateMarblePlacemark(placemark);
    }

    emit updateMarble();
    emit robotsVisibilityChanged(mPlacemarksVisible, mIconsSimplified);
}

void PlacemarkLogic::changeVisibility(bool visible, bool simplified)
{
    if (visible == mPlacemarksVisible && simplified == mIconsSimplified)
        return;

    mPlacemarksVisible = visible;
    mIconsSimplified = simplified;

    applyVisibility();
}

void PlacemarkLogic::togglePlacemarksVisibility()
{
    bool newVisibility = !mIconsSimplified;
    bool newIconsSimplified = (mPlacemarksVisible && !mIconsSimplified);
    changeVisibility(newVisibility, newIconsSimplified);
}

bool PlacemarkLogic::placemarksVisible() const
{
    return mPlacemarksVisible;
}

MapLogPlacemarkDataPtr PlacemarkLogic::mapLog() const
{
    return mMapLog;
}

GeoDataLatLonBox PlacemarkLogic::allPlacemarksGeoRect() const
{   //cache if needed
    qreal minlon = 180;
    qreal minlat = 90;
    qreal maxlon = -180;
    qreal maxlat = -90;

    int visiblePlacemarks = 0;
    foreach(PlacemarkPtr placemark, mGeoManager->placemarks())
    {
        if (placemark->isVisible()
                && isPartOfOverview(mGeoManager->getMapObjectForPlacemark(placemark)))
        {
            qreal lon, lat;
            placemark->coordinate().geoCoordinates(lon, lat, GeoDataCoordinates::Degree);
            if (lon < minlon)
                minlon = lon;
            if (lon > maxlon)
                maxlon = lon;
            if (lat < minlat)
                minlat = lat;
            if (lat > maxlat)
                maxlat = lat;
            visiblePlacemarks++;
        }
    }
    //TODO - refactor constants out
    const qreal minimumDim = 0.002;

    if (visiblePlacemarks > 1)
    {
        const qreal scaleHorizontal = 0.8; //How much context to show
        const qreal scaleVertical = 1;
        qreal latDelta = maxlat - minlat;
        qreal lonDelta = maxlon - minlon;
        latDelta = latDelta * (1.0 - scaleVertical);
        lonDelta = lonDelta * (1.0 - scaleHorizontal);
        minlat += latDelta/2;
        maxlat -= latDelta/2;
        minlon += lonDelta/2;
        maxlon -= lonDelta/2;
    }

    if (maxlat - minlat < minimumDim)
    {
        qreal center = (maxlat + minlat)/2;
        maxlat = center + minimumDim/2;
        minlat = center - minimumDim/2;
    }

    if (maxlon - minlon < minimumDim)
    {
        qreal center = (maxlon + minlon)/2;
        maxlon = center + minimumDim/2;
        minlon = center - minimumDim/2;
    }

    return GeoDataLatLonBox(maxlat, minlat, maxlon, minlon, GeoDataCoordinates::Degree);
}

void PlacemarkLogic::updateMarblePlacemark(PlacemarkPtr placemark)
{
    mMarbleModel->treeModel()->updateFeature(placemark);
}


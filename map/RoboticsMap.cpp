#include <QTimer>
#include <QKeyEvent>
#include <QQuickPaintedItem>
#include <QItemSelectionModel>
#include <QGestureEvent>
#include <QPinchGesture>
#include <QFileInfo>

#include <marble/MarbleMap.h>
#include <marble/MarbleModel.h>
#include <marble/GeoDataDocument.h>
#include <marble/MarbleGlobal.h>
#include <marble/MarbleInputHandler.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/GeoDataDocument.h>
#include <marble/GeoDataObject.h>
#include <marble/GeoDataFeature.h>
#include <marble/MarbleDebug.h>

#include "PathsLayer.h"
#include "RegionsLayer.h"
#include "CrosshairLayer.h"
#include "LocalMapLayer.h"
#include "LaserCloudLayer.h"
#include "RobotManualPlacementLayer.h"
#include "DynamicObjectsLayer.h"
#include "SensorDataLayer.h"

#include "RoboticsMap.h"
#include "GuiController.h"
#include "PlacemarkLogic.h"
#include "MapLibraryHelpers.h"
#include "TextureManager.h"
#include "MapKeyboardInputHandler.h"
#include "GeoObjectsManager.h"
#include "TrackingCamera.h"
#include "MapPlacesManager.h"
#include "ManualPositioningLogic.h"
#include "MapEditor.h"
#include "RobotEditor.h"
#include "MapRobotObject.h"
#include "MapWaypointObject.h"

using namespace Marble;
using namespace MapAbstraction;

namespace
{
    int getScaledZoomByParam(int oldZoom, int oldWindowParam, qreal newWindowParam) // *WindowParam can be eg. width, height etc.
    {
        return (int)(oldZoom-200*log(oldWindowParam/newWindowParam));
    }
}// unnamed namespace

void RoboticsMap::configure()
{
    map()->setSize(width(), height());
    map()->setProjection(Marble::Spherical);
    map()->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    map()->setShowCompass(false);
    map()->setShowOverviewMap(false);
    map()->setShowAtmosphere(false);
    map()->setShowBackground(false);
    map()->setShowClouds(false);
    map()->setShowSunShading(false);
    map()->setShowGrid(false);
    map()->setShowOtherPlaces(false);
    map()->setShowPlaces(false);
    map()->setShowCities(false);
    map()->setShowScaleBar(false);
    map()->setVolatileTileCacheLimit(300000); //300Mb memory cache
    model()->setWorkOffline(false); //TODO

    //model()->setHome(25.9722672,44.4349764, 2500);
    model()->setHome(20.91, 52.19, 2800);
    goHome();

    mPlacemarkLogic->togglePlacemarksVisibility();
    update();
}

void RoboticsMap::updateLicense()
{   //TODO - other licenses
    QString license;
    if (map()->mapThemeId() == "earth/openstreetmap/openstreetmap.dgml")
    {   //TODO - acquire from dgml?
        /* Full license: display in about section or as a tooltop
         * Source: © OpenStreetMap contributors, License: Creative Commons Attribution-ShareAlike 2.0 (CC BY-SA)
         * or link to this page: http://www.openstreetmap.org/copyright
        */
        license = "© OpenStreetMap contributors";
    }
    emit licenseChanged(license);
}

void RoboticsMap::centerUpdate()
{
    qreal lat = map()->centerLatitude();
    qreal lon = map()->centerLongitude();
    emit centerPositionChanged(MapLibraryHelpers::coordsString(lon, lat));
}

void RoboticsMap::connectSignals()
{
    connect(mInputHandler.data(), SIGNAL(toggleVisibility()),
            this, SLOT(toggleVisibility()));

    connect(mInputHandler.data(), SIGNAL(togglePaths()),
            this, SLOT(togglePaths()));

    connect(inputHandler(), SIGNAL(lmbRequest(int,int)),
            this, SLOT(selectPlacemarkAt(int, int)));

    connect(mTrackingCamera.data(), SIGNAL(cameraUpdate(Marble::GeoDataLatLonBox)),
            this, SLOT(center(Marble::GeoDataLatLonBox)));

    connect(mPlacemarkLogic.data(), SIGNAL(updateMarble()),
            this, SLOT(update()));

    connect(map(), SIGNAL(themeChanged(QString)),
            this, SLOT(updateLicense()));

    connect(map(), SIGNAL(visibleLatLonAltBoxChanged(GeoDataLatLonAltBox)),
            this, SLOT(centerUpdate()));

    connect(mPlacemarkLogic.data(), SIGNAL(activeRobotPositionChanged(GeoObjectID)),
            mMapPlacesManager.data(), SLOT(activeRobotPositionChanged(GeoObjectID)));

    connect(mLocalMapReceiver.data(), SIGNAL(localMapChanged(QString, qreal, MapAbstraction::GeoCoords, MapAbstraction::Orientation, QPointF)),
            this, SLOT(updateLocalMap(QString, qreal, MapAbstraction::GeoCoords, MapAbstraction::Orientation, QPointF)));

    qRegisterMetaType<LaserScanPoints>("LaserScanPoints");
    connect(mLocalMapReceiver.data(), SIGNAL(laserPointsCloudChanged(int, MapAbstraction::LaserScanPoints)),
            this, SLOT(updateLaserPointsCloud(int, MapAbstraction::LaserScanPoints)));

    qRegisterMetaType<MapPlaces>("MapPlaces");
    connect(mLocalMapReceiver.data(), SIGNAL(namedPlacesChanged(MapAbstraction::MapPlaces)),
            this, SLOT(updateMapPlaces(MapAbstraction::MapPlaces)));

    connect(mManualPositioningLogic.data(), SIGNAL(robotPlacementComplete(GeoObjectID,MapAbstraction::MapRobotObjectPtr)),
            this, SLOT(manualOverrideOfRobotLocation(GeoObjectID,MapAbstraction::MapRobotObjectPtr)));

    if (mLayers.hasLayer(LayerLocalMap))
    {
        LocalMapLayerPtr localMap = mLayers.getLayer(LayerLocalMap).staticCast<LocalMapLayer>();
        connect(localMap.data(), SIGNAL(localMapHasContent()),
                this, SIGNAL(localMapHasContent()));
        connect(localMap.data(), SIGNAL(localMapVisibilityChanged(bool)),
                this, SIGNAL(localMapVisibilityChanged(bool)));
        localMap->notifyState();
    }

    GuiController *guiController = findChild<GuiController*>("guiController");
    Q_ASSERT(guiController);
    if (guiController)
    {
        connect(mPlacemarkLogic.data(), SIGNAL(placemarkUpdated(GeoObjectID, MapAbstraction::MapObjectConstPtr, QString)),
                guiController, SLOT(updatePlacemarkGuiInfo(GeoObjectID, MapAbstraction::MapObjectConstPtr, QString)));
        connect(mPlacemarkLogic.data(), SIGNAL(placemarkRemoved(GeoObjectID)),
                guiController, SLOT(placemarkRemoved(GeoObjectID)));
        connect(mInputHandler.data(), SIGNAL(itemSelected(int)),
                guiController, SIGNAL(itemSelected(int)));
        connect(mPlacemarkLogic.data(), SIGNAL(robotsVisibilityChanged(bool,bool)),
                guiController, SIGNAL(robotsVisibilityChanged(bool,bool)));
        connect(this, SIGNAL(cameraChangePreventedWhenTracking()),
                guiController, SIGNAL(cameraChangePreventedWhenTracking()));
        connect(this, SIGNAL(licenseChanged(QString)),
                guiController, SIGNAL(licenseChanged(QString)));
        connect(this, SIGNAL(centerPositionChanged(QString)),
                guiController, SIGNAL(centerPositionChanged(QString)));
    }

    MapEditor *mapEditor = findChild<MapEditor*>("mapEditor");
    Q_ASSERT(mapEditor);
    if (mapEditor)
    {
        connect(mapEditor, SIGNAL(addAtCenter(bool)),
                mMapPlacesManager.data(), SLOT(addAtCenter(bool)));
        connect(mapEditor, SIGNAL(removeAtCenter()),
                mMapPlacesManager.data(), SLOT(removeAtCenter()));
        connect(mapEditor, SIGNAL(removeAll()),
                mMapPlacesManager.data(), SLOT(removeAll()));
        connect(mapEditor, SIGNAL(editModeChanged(PlacemarkType,bool)),
                mPlacemarkLogic.data(), SLOT(placemarkTypeVisibilityRequest(PlacemarkType,bool)));
        connect(mapEditor, SIGNAL(editModeChanged(PlacemarkType,bool)),
                mMapPlacesManager.data(), SLOT(editModeChanged(PlacemarkType,bool)));
        connect(mapEditor, SIGNAL(placemarkSelected(int)),
                mMapPlacesManager.data(), SLOT(selectPlacemark(int)));
        connect(mapEditor, SIGNAL(finalizeMapEdit(bool)),
                mMapPlacesManager.data(), SLOT(finalizeMapEdit(bool)));
        connect(mapEditor, SIGNAL(commandOrderParking()),
                mMapPlacesManager.data(), SLOT(orderParking()));
        connect(mapEditor, SIGNAL(commandOrderPath()),
                mMapPlacesManager.data(), SLOT(orderPath()));
        connect(mMapPlacesManager.data(), SIGNAL(selectionModeChanged(bool)),
                mapEditor, SIGNAL(selectionModeChanged(bool)));
    }

    qRegisterMetaType<MapWaypointObjectPtr>("MapWaypointObjectPtr");
    qRegisterMetaType<MapAbstraction::MapPath>("MapAbstraction::MapPath");
    connect(mMapPlacesManager.data(), SIGNAL(mapPathCreated(int,MapAbstraction::MapPath)),
            mSender.data(), SIGNAL(mapPathCreated(int,MapAbstraction::MapPath)));

    qRegisterMetaType<MapWaypointObjectPtr>("MapObjectPtr");
    connect(mMapPlacesManager.data(), SIGNAL(skillTriggered(int, QString)),
            mSender.data(), SIGNAL(skillTriggered(int, QString)));

    RobotEditor *robotEditor = findChild<RobotEditor*>("robotEditor");
    Q_ASSERT(robotEditor);
    if (robotEditor)
    {
        connect(robotEditor, SIGNAL(finalizePositionEdit(bool)),
                mManualPositioningLogic.data(), SLOT(finalizePlacement(bool)));
        connect(robotEditor, SIGNAL(startPositionEdit()),
                mManualPositioningLogic.data(), SLOT(startPlacement()));
        connect(robotEditor, SIGNAL(orientationEdit(bool)),
                mManualPositioningLogic.data(), SLOT(orientationEdit(bool)));
        connect(robotEditor, SIGNAL(placeAtCrosshair()),
                mManualPositioningLogic.data(), SLOT(placeAtCrosshair()));
        connect(robotEditor, SIGNAL(orientateToCrosshair()),
                mManualPositioningLogic.data(), SLOT(orientateToCrosshair()));
    }
}

void RoboticsMap::center(PlacemarkConstPtr placemark)
{
    if (!mGeoObjectsManager->getMapObjectForPlacemark(placemark)->coords().valid())
        return;

    const int minimumAcceptedZoom = 2700;
    centerOn(*placemark, true);
    if (zoom() < minimumAcceptedZoom)
        setZoom(minimumAcceptedZoom);
}

void RoboticsMap::center(int placemarkID)
{
    PlacemarkConstPtr place = mGeoObjectsManager->getPlacemark(GeoObjectID::fromInt(placemarkID));
    if (place)
    {
        center(place);
    }
}

MapLogPlacemarkDataPtr RoboticsMap::mapLog()
{
    return mPlacemarkLogic->mapLog();
}

RoboticsMap::RoboticsMap(QQuickItem *parent) : MarbleQuickItem(parent),
    mGeoObjectsManager(new GeoObjectsManager()),
    mInputHandler(new MapKeyboardInputHandler(mGeoObjectsManager)),
    mPlacemarkLogic(new PlacemarkLogic(model(), mGeoObjectsManager)),
    mTrackingCamera(new TrackingCamera()),
    mLayers(this),
    mTextureManager(new TextureManager(map())),
    mMapPlacesManager(new MapPlacesManager(mPlacemarkLogic, mGeoObjectsManager, mLayers.getLayer(LayerLocalMap).staticCast<LocalMapLayer>(), map())),
    mSender(new GeoMapSender()),
    mLocalMapReceiver(new GeoLocalMapReceiver()),
    mLocalMapLoader(new LocalMapLoader(this)),
    mManualPositioningLogic(new ManualPositioningLogic(mPlacemarkLogic, mGeoObjectsManager, map(),
                                                       mLayers.getLayer(LayerRobotManualPlacement).staticCast<RobotManualPlacementLayer>())),
    mSensorReadings(new SensorReadings(mLayers.getLayer(LayerSensorData).staticCast<SensorDataLayer>()))
{  
    installEventFilter(mInputHandler.data()); //add our specific filter on top
    configure();
}

GeoMapSenderPtr RoboticsMap::sender() const
{
    return mSender;
}

GeoLocalMapReceiverPtr RoboticsMap::localMapReceiver() const
{
    return mLocalMapReceiver;
}

ISensorReadingsPtr RoboticsMap::sensorReadingsInterface() const
{
    return mSensorReadings;
}

void RoboticsMap::processRobotConnectionStatus(MapRobotObjectPtr newObject)
{   //TODO - robot connection state responsibility should be just on console side.
    //This is here temporarily to solve compatibility issues with 2 different use cases.
    if (newObject->state() == RobotStateDisconnected || newObject->state() == RobotStateDisappeared)
        return;

    if (mGeoObjectsManager->isAnyRobotConnected())
    {
        if (mGeoObjectsManager->connectedRobotID() == newObject->robotID())
        {
            //qDebug("Setting connected for %d", newObject->robotID());
            newObject->setState(RobotStateConnected);
        }
        else if (newObject->state() == RobotStateConnected)
        {   //Only one should be connected at once, this one should not
            //qDebug("Setting normal for %d", newObject->robotID());
            newObject->setState(RobotStateNormal);
        }
    }
}

bool RoboticsMap::processRobotLocalizationStatus(GeoObjectID id, MapRobotObjectPtr robotObjectData)
{
    if (robotObjectData->localizationType() != MapObject::Global
            && robotObjectData->positionAvailable())
    {   //Has position but not global: compute from local map
        mLayers.getLayer(LayerLocalMap).staticCast<LocalMapLayer>()->localizeRobot(id, robotObjectData);
    }
    return true;
}

void RoboticsMap::updatePlacemark(GeoObjectID id, MapObjectPtr objectData)
{
    if (PlacemarkRobot == objectData->category())
    {
        MapRobotObjectPtr robotObjectData = objectData.staticCast<MapRobotObject>();
        processRobotConnectionStatus(robotObjectData);
        bool proceed = processRobotLocalizationStatus(id, robotObjectData);
        if (!proceed)
        {
            return;
        }

        if (mManualPositioningLogic->isInManualPlacementMode(id))
        {   //The robot is being manualy positioned on the map so don't interfere
            MapRobotObjectPtr manualOverride = mManualPositioningLogic->positionedRobot();
            robotObjectData->setCoords(manualOverride->coords());
            robotObjectData->setOrientation(manualOverride->orientation());
            return;  //TODO - alternatively, we might ditch only position and orientation components
        }

        broadcastRelativePosition(robotObjectData);
    }

    mPlacemarkLogic->addOrUpdatePlacemark(id, objectData);
}

void RoboticsMap::updateFileGeometry(const QString &fileSource)
{
    QFileInfo inputFile(fileSource);
    if (!inputFile.isFile())
    {
        qWarning("updateFileGeometry error: invalid filename provided (%s)", qPrintable(fileSource));
        return;
    }
    model()->addGeoDataFile(inputFile.absoluteFilePath());
}

void RoboticsMap::removeGeometry(const QString &keyName)
{
    model()->removeGeoData(keyName);
}

void RoboticsMap::broadcastRelativePosition(MapRobotObjectPtr robotObjectData)
{
    foreach (PlacemarkPtr place, mGeoObjectsManager->placemarks())
    {   //TODO - this is temporary and will be replaced (function: find operator placement if available)
        MapObjectPtr mapObject = mGeoObjectsManager->getMapObjectForPlacemark(place);
        if (PlacemarkPlace == mapObject->category() && "console" == mapObject->type()) //TODO-const
        {   //There is an operator placed, find relative robot location. Robot is already in global coords;
            GeoDataCoordinates robotCoords = MapLibraryHelpers::transformCoords(robotObjectData->coords());
            GeoDataCoordinates placeCoords = MapLibraryHelpers::transformCoords(mapObject->coords());

            qreal metersInRadianOfLongitude = model()->planetRadius();
            qreal metersInRadianOfLatitude = cos(placeCoords.latitude()) * model()->planetRadius();

            qreal y = placeCoords.latitude() * metersInRadianOfLongitude - robotCoords.latitude() * metersInRadianOfLongitude;
            qreal x = placeCoords.longitude() * metersInRadianOfLatitude - robotCoords.longitude() * metersInRadianOfLatitude;

            GeoCoords metersXYCoords(x, y);
            GeoCoords radiansGlobalCoords(placeCoords.longitude(), placeCoords.latitude());

            /*
            qDebug("Relative (to operator) robot position Available %f %f, in radians: %.8lf %.8lf",
                   x, y, placeCoords.longitude() - robotCoords.longitude(),
                   placeCoords.latitude() - robotCoords.latitude());
            */
            sender()->robotPositionRelativeToOperator(robotObjectData->robotID(), metersXYCoords,
                                                      radiansGlobalCoords);
            break;
        }
    }
}

void RoboticsMap::connectedToRobot(int robotID, bool connection)
{
    if (!connection && mGeoObjectsManager->connectedRobotID() == robotID)
    {
        mGeoObjectsManager->setNoRobotConnected();
    }
    else if (connection)
    {
        mGeoObjectsManager->setConnectedRobotID(robotID);
    }
}

void RoboticsMap::componentComplete()
{
    QQuickItem *pinch = findChild<QQuickItem*>("pinchArea");
    if (pinch)
    {
        pinch->installEventFilter(getEventFilter());
    }

    connectSignals();
    updateLicense();
}

void RoboticsMap::makePinch(QPointF center, Qt::GestureState state, qreal scale)
{
    scale = sqrt(sqrt(scale));
    scale = qBound(0.5, scale, 2.0);
    pinch(center, scale, state);
}

void RoboticsMap::geometryChanged(const QRectF &newGeometry, const QRectF &oldGeometry)
{
    QQuickItem::geometryChanged(newGeometry, oldGeometry);
    if(oldGeometry.width() != 0 && newGeometry.width() != 0)
        setZoom(getScaledZoomByParam(zoom(), oldGeometry.width(), newGeometry.width()));
}

int RoboticsMap::getScaledZoom(int oldZoom, int oldWindowWidth)
{
    return getScaledZoomByParam(oldZoom, oldWindowWidth, width());
}

void RoboticsMap::handlePinchStart(QPointF center)
{
    makePinch(center, Qt::GestureStarted);
}

void RoboticsMap::handlePinchUpdate(QPointF center, qreal scale)
{
    makePinch(center, Qt::GestureUpdated, scale);
}

void RoboticsMap::handlePinchEnd(QPointF center, bool canceled)
{
    makePinch(center, canceled ? Qt::GestureCanceled : Qt::GestureFinished);
}

void RoboticsMap::toggleVisibility()
{
    /*
    QVector<GeoDataFeature*> fl = model()->treeModel()->rootDocument()->featureList();
    for (GeoDataFeature *f : fl)
    {
        if (f->name() == "bucharest.kml")
        {
            qDebug() << "-" << f->name() << "-";
            f->setVisible(!f->isVisible());
            GeoDataDocument *d = geodata_cast<GeoDataDocument>(f);
            if (d)
            {
                for (auto ff : d->featureList())
                {
                    qDebug() << ":" << ff->name() << ":";
                    if (ff->name() == "path")
                    {
                        GeoDataPlacemark *p = geodata_cast<GeoDataPlacemark>(ff);
                        p->setVisible(!p->isVisible());
                        qDebug() << "::" << p->visualCategory();
                        qDebug() << ":::"  << p->categoryName();
                        GeoDataLinearRing *r = static_cast<GeoDataLinearRing *>(p->geometry());
                        qDebug() << "::::" << r->length(model()->planetRadius());
                        if (p->isGloballyVisible())
                            qDebug() << "Globally visible!";
                        update();
                    }
                }
            }
        }
    }
    */

    mPlacemarkLogic->togglePlacemarksVisibility();
    mLayers.getLayer(LayerPaths)->setVisible(mPathsVisible && mPlacemarkLogic->placemarksVisible());
}

void RoboticsMap::toggleLayer()
{
    mTextureManager->toggle();
}

void RoboticsMap::togglePaths()
{
    mPathsVisible = !mPathsVisible;
    mLayers.getLayer(LayerPaths)->setVisible(mPathsVisible && mPlacemarkLogic->placemarksVisible());
}

void RoboticsMap::manualOverrideOfRobotLocation(GeoObjectID id, MapRobotObjectPtr oldRobot)
{   //Old robot is a frame reference point for new location
    MapRobotObjectPtr newRobot = mGeoObjectsManager->getMapObjectForID(id).staticCast<MapRobotObject>();

    QPointF robotLocalPoint;
    qreal orientation;
    mLayers.getLayer(LayerLocalMap).staticCast<LocalMapLayer>()->calculateRelativePosition(id, oldRobot, newRobot,
                                                                                           robotLocalPoint, orientation);
    GeoCoords coords(robotLocalPoint.x(), robotLocalPoint.y());
    mSender->robotPositioned(newRobot->robotID(), coords, orientation);
}

void RoboticsMap::overviewPlacemarks()
{
     GeoDataLatLonBox box = mPlacemarkLogic->allPlacemarksGeoRect();
     centerOn(box);
}

void RoboticsMap::turnPlacemarksOverview()
{
    mTrackingCamera->TrackOverview(mPlacemarkLogic);
    mTrackingCamera->toggleTracking(true);
}

void RoboticsMap::turnFollowPlacemark(int placemarkID)
{
    GeoObjectID id = GeoObjectID::fromInt(placemarkID);
    if (!id.isNull())
    {
        PlacemarkConstPtr place = mGeoObjectsManager->getPlacemark(id);
        if (place)
            mTrackingCamera->TrackPlacemark(id, place);
        else
            qWarning("Place to follow not found");
    }
    mTrackingCamera->toggleTracking(true);
}

void RoboticsMap::stopCameraFollow()
{
    mTrackingCamera->toggleTracking(false);
}

void RoboticsMap::center(const GeoDataLatLonBox &box)
{
    int zoomBefore = zoom();
    bool allowLargerZoomOut = true;

    centerOn(box);  //this changes zoom to most fitting the box

    //Allow user to have a little zoom out from default zoom
    if (allowLargerZoomOut)
    {
        int defaultTrackingZoom = zoom();
        const qreal acceptedZoomRatio = 0.95;
        const int minZoomWhenTracking = defaultTrackingZoom*acceptedZoomRatio;
        int boundZoom = qBound(minZoomWhenTracking, zoomBefore, defaultTrackingZoom);
        if (boundZoom != defaultTrackingZoom)
        {
            setZoom(boundZoom);
        }
        if (boundZoom != zoomBefore)
        {
            emit cameraChangePreventedWhenTracking();
        }
    }
}

void RoboticsMap::displayCrosshair(bool display)
{
    mLayers.setVisibility(LayerCrosshair, display);
    update();
}

void RoboticsMap::selectPlacemarkAt(int x, int y)
{
    mMapPlacesManager->selectPlacemarkRequest(x, y);
}

void RoboticsMap::updateLocalMap(QString localMapPng, qreal resolution,
                                   GeoCoords coords, Orientation rotation, QPointF origin)
{
    Marble::GeoDataCoordinates marbleCoords;
    if (!coords.valid())
    {
        marbleCoords.set(20.9226, 52.1942, 0, Marble::GeoDataCoordinates::Degree);
    }
    else
    {
        marbleCoords = MapLibraryHelpers::transformCoords(coords);
    }

    /*
    qDebug("Received map: %s, %f, (%f, %f), %f, (%f, %f)", localMapPng.toStdString().c_str(),
           resolution, coords.longitude(), coords.latitude(), rotation, origin.x(), origin.y());
    */

    mLayers.setVisibility(LayerLaserCloud, true);
    mLayers.setVisibility(LayerLocalMap, true);
    mLayers.getLayer(LayerLocalMap).staticCast<LocalMapLayer>()->setLayerContent(localMapPng, resolution, marbleCoords, rotation, origin);
}

void RoboticsMap::updateRequested()
{
    update();
}

void RoboticsMap::updateLaserPointsCloud(int robotID, LaserScanPoints points)
{
    if (!mLayers.hasLayer(LayerLaserCloud))
        return;

    MapRobotObjectPtr localRobot = mGeoObjectsManager->getConnectedRobot();
    if (localRobot && (robotID == localRobot->robotID()))
    {
        //qDebug("Received laser cloud");
        LaserCloudLayerPtr lcl = mLayers.getLayer(LayerLaserCloud).staticCast<LaserCloudLayer>();
        lcl->updateContent(points, localRobot);
    }
}

void RoboticsMap::updateDynamicObjects(int robotID, DynamicObjects objects)
{
    MapRobotObjectPtr connectedRobot = mGeoObjectsManager->getConnectedRobot();

    if (!connectedRobot)
    {
        mLayers.setVisibility(LayerDynamicObjects, false);
        return;
    }

    if (connectedRobot && connectedRobot->robotID() == robotID)
    {
        mLayers.getLayer(LayerDynamicObjects).staticCast<DynamicObjectsLayer>()->updateContent(objects, connectedRobot);
        mLayers.setVisibility(LayerDynamicObjects, true);
    }
}

void RoboticsMap::updateMapPlaces(MapPlaces places)
{
    //TODO - for now, ignore this call
    return;

    foreach (MapPlaceObjectConstPtr place, places)
    {
        GeoObjectID id = mGeoObjectsManager->findPlace(place->name());
        if (id.isNull())
        {
            id = GeoReferenceFactory::createGeoObjectId();
        }

        MapObjectPtr clonedPlace(place->Clone());
        qWarning("PRE-Changes: %f %f %s", clonedPlace->coords().longitude(),
                 clonedPlace->coords().latitude(), qPrintable(clonedPlace->name()));
        if (clonedPlace->localizationType() != MapObject::Global)
        {
            QPointF coords(clonedPlace->coords().longitude(), clonedPlace->coords().latitude());
            LocalMapLayerPtr lml = mLayers.getLayer(LayerLocalMap).staticCast<LocalMapLayer>();
            GeoCoords transformed = MapLibraryHelpers::transformCoords(lml->localToGlobal(coords));
            clonedPlace->setCoords(transformed);
        }
        qWarning("POST-Changes: %f %f %s", clonedPlace->coords().longitude(),
                 clonedPlace->coords().latitude(), qPrintable(clonedPlace->name()));

        mPlacemarkLogic->addOrUpdatePlacemark(id, clonedPlace);
    }
}

bool RoboticsMap::layersEventFilter(QObject *o, QEvent *e)
{
    RobotManualPlacementLayerPtr rmpl = mLayers.getLayer(LayerRobotManualPlacement).staticCast<RobotManualPlacementLayer>();
    if (rmpl->handleEvent(o, e))
        return true;

    LocalMapLayerPtr lml = mLayers.getLayer(LayerLocalMap).staticCast<LocalMapLayer>();
    return lml->handleEvent(o, e);
}

void RoboticsMap::toggleLocalMapPositioning()
{
    LocalMapLayerPtr lml = mLayers.getLayer(LayerLocalMap).staticCast<LocalMapLayer>();
    lml->toggleManualGeolocalization();
}

void RoboticsMap::toggleLocalMapVisibility()
{
    mLayers.toggleVisibility(LayerLocalMap);
}

void RoboticsMap::onRobotConnectToggle(int robotID)
{
    bool doConnect = true;
    if (robotID == mGeoObjectsManager->connectedRobotID())
    { //already connected to this robot - this means disconnect request
        doConnect = false;
    }
    sender()->requestConnect(robotID, doConnect);
}

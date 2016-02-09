#include <QTimer>
#include <QKeyEvent>
#include <QQuickPaintedItem>
#include <QItemSelectionModel>
#include <QGestureEvent>
#include <QPinchGesture>

#include <marble/MarbleMap.h>
#include <marble/MarbleModel.h>
#include <marble/GeoDataDocument.h>
#include <marble/MarbleGlobal.h>
#include <marble/MarbleInputHandler.h>
#include <marble/GeoDataTreeModel.h>

#include "PathsLayer.h"
#include "RegionsLayer.h"
#include "CrosshairLayer.h"
#include "LocalMapLayer.h"
#include "LaserCloudLayer.h"
#include "RobotManualPlacementLayer.h"

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
    map()->setShowScaleBar(false);
    map()->setVolatileTileCacheLimit(300000); //300Mb memory cache
    model()->setWorkOffline(false); //TODO

    //model()->setPersistentTileCacheLimit(); Sets disk space limit
    model()->setHome(20.91, 52.19, 2800);
    goHome();
    mScaling=true;
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

    connect(mManualPositioningLogic.data(), SIGNAL(robotPlacementComplete(GeoObjectID,MapAbstraction::MapRobotObjectPtr)),
            this, SLOT(manualOverrideOfRobotLocation(GeoObjectID,MapAbstraction::MapRobotObjectPtr)));

    connect(mLocalMapLayer.data(), SIGNAL(localMapHasContent()),
            this, SIGNAL(localMapHasContent()));

    connect(mLocalMapLayer.data(), SIGNAL(localMapVisibilityChanged(bool)),
            this, SIGNAL(localMapVisibilityChanged(bool)));

    mLocalMapLayer->notifyState();

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
        connect(mMapPlacesManager.data(), SIGNAL(selectionModeChanged(bool)),
                mapEditor, SIGNAL(selectionModeChanged(bool)));
    }

    qRegisterMetaType<MapWaypointObjectPtr>("MapWaypointObjectPtr");
    qRegisterMetaType<MapAbstraction::MapPath>("MapAbstraction::MapPath");
    connect(mMapPlacesManager.data(), SIGNAL(mapPathCreated(int,MapAbstraction::MapPath)),
            mSender.data(), SIGNAL(mapPathCreated(int,MapAbstraction::MapPath)));

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

RoboticsMap::RoboticsMap(QQuickItem *parent) : MarbleQuickItem(parent),
    mGeoObjectsManager(new GeoObjectsManager()),
    mInputHandler(new MapKeyboardInputHandler(mGeoObjectsManager)),
    mPlacemarkLogic(new PlacemarkLogic(model(), mGeoObjectsManager)),
    mTrackingCamera(new TrackingCamera()),
    mPathsLayer(new PathsLayer(mPlacemarkLogic->mapLog(), mGeoObjectsManager)),
    mRegionsLayer(new RegionsLayer(mGeoObjectsManager)),
    mCrosshairLayer(new CrosshairLayer()),
    mLocalMapLayer(new LocalMapLayer(model(), map())),
    mLaserCloudLayer(new LaserCloudLayer(mLocalMapLayer)),
    mRobotPlacementLayer(new RobotManualPlacementLayer(map())),
    mPathsVisible(false),
    mScaling(true),
    mTextureManager(new TextureManager(map())),
    mMapPlacesManager(new MapPlacesManager(mPlacemarkLogic, mGeoObjectsManager, mLocalMapLayer, map())),
    mSender(new GeoMapSender()),
    mLocalMapReceiver(new GeoLocalMapReceiver()),
    mLocalMapLoader(new LocalMapLoader(this)),
    mManualPositioningLogic(new ManualPositioningLogic(mPlacemarkLogic, mGeoObjectsManager, map(), mRobotPlacementLayer))
{  
    map()->addLayer(mCrosshairLayer.data());
    map()->addLayer(mPathsLayer.data());
    map()->addLayer(mRegionsLayer.data());
    map()->addLayer(mLocalMapLayer.data());
    map()->addLayer(mLaserCloudLayer.data());
    map()->addLayer(mRobotPlacementLayer.data());
    mPathsLayer->setRobotPathsVisibility(mPathsVisible);

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

void RoboticsMap::processRobotConnectionStatus(MapRobotObjectPtr newObject)
{   //TODO - robot connection state responsibility should be just on console side.
    //This is here temporarily to solve compatibility issues with 2 different use cases.
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

        if (!mLocalMapLayer->hasContent())
            return false; //We can't do anything with such a robot - we don't have local map!

        GeoDataCoordinates newCoords;
        if (robotObjectData->localizationType() == MapObject::LocalAbsolute)
        {
            newCoords = mLocalMapLayer->localToGlobal(
                        QPointF(robotObjectData->coords().longitude(),
                                robotObjectData->coords().latitude()));
            robotObjectData->setOrientation(mLocalMapLayer->localToGlobalRotation(
                                              robotObjectData->orientation()));
            robotObjectData->setCoords(MapLibraryHelpers::transformCoords(newCoords));
        }
        else if (robotObjectData->localizationType() == MapObject::LocalRelative)
        {
            //qDebug("Incoming coords: %f, %f", robotObjectData->coords().latitude(), robotObjectData->coords().longitude());
            MapRobotObjectPtr referenceRobot;
            if (mLocalMapLayer->hasReferenceFrame(id))
            {
                referenceRobot = mLocalMapLayer->getReferenceFrame(id);
            }
            else
            {   //TODO - we are using the robot object as data object here. It's not too good
                GeoDataCoordinates localZeroInGlobal = mLocalMapLayer->localToGlobal(QPointF(0,0));
                QString e;
                referenceRobot.reset(new MapRobotObject(MapLibraryHelpers::transformCoords(localZeroInGlobal),
                                                        0, RobotStateNormal, e, e, e, 0, 0));
            }


            newCoords = mLocalMapLayer->robotToGlobal(
                        QPointF(robotObjectData->coords().longitude(),
                        robotObjectData->coords().latitude()), referenceRobot);

            //qDebug("Coords old: |%f,%f,%f|", robotObjectData->coords().longitude(), robotObjectData->coords().latitude(), robotObjectData->orientation());

            //qDebug("orientations: %f, %f, sum %f", robotObjectData->orientation(), referenceRobot->orientation(),
            //       robotObjectData->orientation() + referenceRobot->orientation());
            robotObjectData->setOrientation(mLocalMapLayer->localToGlobalRotation(
                                              robotObjectData->orientation()));
            robotObjectData->setOrientation(robotObjectData->orientation() + referenceRobot->orientation()); //TODO
        }

        robotObjectData->setCoords(MapLibraryHelpers::transformCoords(newCoords));
        robotObjectData->setLocalizationType(MapObject::Global);
    }
    return true;
}

void RoboticsMap::updatePlacemark(GeoObjectID id, MapRobotObjectPtr robotObjectData)
{
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

    foreach (PlacemarkPtr place, mGeoObjectsManager->placemarks())
    {   //TODO - this is temporary and will be replaced (function: find operator placement if available)
        MapObjectPtr mapObject = mGeoObjectsManager->getMapObjectForPlacemark(place);
        if (PlacemarkPlace == mapObject->category())
        {   //There is an operator placed, find relative robot location. Robot is already in global coords;
            GeoDataCoordinates robotCoords = MapLibraryHelpers::transformCoords(robotObjectData->coords());
            GeoDataCoordinates placeCoords = MapLibraryHelpers::transformCoords(mapObject->coords());

            qreal metersInRadianOfLongitude = model()->planetRadius();
            qreal metersInRadianOfLatitude = cos(placeCoords.latitude()) * model()->planetRadius();

            qreal y = (placeCoords.latitude() - robotCoords.latitude()) * metersInRadianOfLatitude;
            qreal x = (placeCoords.longitude() - robotCoords.longitude()) * metersInRadianOfLongitude;

            GeoCoords metersXYCoords(x, y);

            if (robotObjectData->connected())
            {   //TODO - no need to send only for connected. Send for all.
                qDebug("Position Available %lf %lf", x, y);
                sender()->robotPositionRelativeToOperator(robotObjectData->robotID(), metersXYCoords);
            }
            break;
        }
    }

    mPlacemarkLogic->addOrUpdatePlacemark(id, robotObjectData);
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
    if(mScaling && oldGeometry.width() != 0 && newGeometry.width() != 0)
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
    mPlacemarkLogic->togglePlacemarksVisibility();
    mPathsLayer->setRobotPathsVisibility(mPathsVisible && mPlacemarkLogic->placemarksVisible());
    mPathsLayer->setWaypointPathsVisibility(mPlacemarkLogic->placemarksVisible());
}

void RoboticsMap::toggleLayer()
{
    mTextureManager->toggle();
}

void RoboticsMap::togglePaths()
{
    mPathsVisible = !mPathsVisible;
    mPathsLayer->setRobotPathsVisibility(mPathsVisible && mPlacemarkLogic->placemarksVisible());
}

void RoboticsMap::manualOverrideOfRobotLocation(GeoObjectID id, MapRobotObjectPtr oldRobot)
{   //Old robot is a frame reference point for new location
    MapRobotObjectPtr newRobot = mGeoObjectsManager->getMapObjectForID(id).staticCast<MapRobotObject>();

    QPointF robotLocalPoint;
    mLocalMapLayer->globalToRobot(MapLibraryHelpers::transformCoords(newRobot->coords()),
                                  oldRobot, robotLocalPoint);
    GeoCoords coords(robotLocalPoint.x(), robotLocalPoint.y());
    mLocalMapLayer->updateRefereceRobotFrame(id, newRobot);
    mSender->robotPositioned(newRobot->robotID(), coords, newRobot->orientation());
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
    mCrosshairLayer->setVisibility(display);
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

    mLocalMapLayer->setLayerContent(localMapPng, resolution, marbleCoords, rotation, origin);
    mLocalMapLayer->setVisible(true);
    mLaserCloudLayer->setVisible(true);
    update();
}

void RoboticsMap::updateLaserPointsCloud(int robotID, LaserScanPoints points)
{
    MapRobotObjectPtr localRobot = mGeoObjectsManager->getConnectedRobot();
    if (mLocalMapLayer->visible() && localRobot && (robotID == localRobot->robotID()))
    {
        //qDebug("Received laser cloud");
        mLaserCloudLayer->setVisible(mLocalMapLayer->visible());
        mLaserCloudLayer->updateContent(points, localRobot);
        update();
    }
}

bool RoboticsMap::layersEventFilter(QObject *o, QEvent *e)
{
    bool handled = mRobotPlacementLayer->handleEvent(o, e);
    if (handled)
    {
        return true;
    }
    return mLocalMapLayer->handleEvent(o, e);
}

void RoboticsMap::toggleLocalMapPositioning()
{
    mLocalMapLayer->toggleManualGeolocalization();
    update();
}

void RoboticsMap::toggleLocalMapVisibility()
{
    mLocalMapLayer->setVisible(!mLocalMapLayer->visible());
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

#ifndef ROBOTICSMAP_H
#define ROBOTICSMAP_H

#include <QFileInfo>
#include <QSharedPointer>
#include <marble/MarbleQuickItem.h>
#include "MapLayersFwd.h"
#include "LocalMapLoader.h"
#include "MapLibraryTypes.h"
#include "InternalTypesFwd.h"
#include "IGeoMap.h"
#include "LaserScanPoint.h"
#include "GeoCoords.h"
#include "Orientation.h"

class RoboticsMap : public Marble::MarbleQuickItem, public MapAbstraction::IGeoMap
{
    Q_OBJECT

    signals:
        void cameraChangePreventedWhenTracking();
        void licenseChanged(QString license);
        void centerPositionChanged(QString position);
        void localMapHasContent();
        void localMapVisibilityChanged(bool aVisible);

    public:
        RoboticsMap(QQuickItem* parent = NULL);

    // QQmlParserStatus interface
    public:
        void componentComplete();

    // IGeoMap interface
    public:
        MapAbstraction::GeoMapSenderPtr sender() const;
        MapAbstraction::GeoLocalMapReceiverPtr localMapReceiver() const;

    // IMapSignalReceiver interface
    public slots:
        void updatePlacemark(GeoObjectID id, MapAbstraction::MapRobotObjectPtr newObject);
        void connectedToRobot(int robotID, bool connected);

    public slots:
        void center(int placemarkID);
        void center(const Marble::GeoDataLatLonBox& box);
        void handlePinchStart(QPointF center);
        void handlePinchUpdate(QPointF center, qreal scale);
        void handlePinchEnd(QPointF center, bool canceled);
        void toggleVisibility();
        void toggleLayer();
        void togglePaths();
        void toggleLocalMapVisibility();
        void toggleLocalMapPositioning();
        void stopCameraFollow();
        void turnPlacemarksOverview();
        void turnFollowPlacemark(int placemarkID);
        void overviewPlacemarks();
        void manualOverrideOfRobotLocation(GeoObjectID id, MapAbstraction::MapRobotObjectPtr robot);
        void displayCrosshair(bool display);
        void onRobotConnectToggle(int robotID);

    protected:
        bool layersEventFilter(QObject *o, QEvent *e);

    private slots:
        void updateLicense();
        void centerUpdate();
        void selectPlacemarkAt(int x, int y);
        void updateLocalMap(QString mapPng, qreal resolution, MapAbstraction::GeoCoords approxCenter,
                            MapAbstraction::Orientation rotation, QPointF origin);
        void updateLaserPointsCloud(int robotID, MapAbstraction::LaserScanPoints);

    private:
        void processRobotConnectionStatus(MapAbstraction::MapRobotObjectPtr newObject);
        bool processRobotLocalizationStatus(GeoObjectID id, MapAbstraction::MapRobotObjectPtr newObject);
        void configure();
        void connectSignals();
        void center(PlacemarkConstPtr placemark);
        void makePinch(QPointF center, Qt::GestureState state, qreal scale = 1);
        void showPath(const GeoObjectID &id);
        void geometryChanged(const QRectF & newGeometry, const QRectF & oldGeometry);
        int getScaledZoom(int oldZoom, int oldWindowWidth);

        MapAbstraction::GeoObjectsManagerPtr mGeoObjectsManager;
        MapKeyboardInputHandlerPtr mInputHandler;
        PlacemarkLogicPtr mPlacemarkLogic;
        TrackingCameraPtr mTrackingCamera;

        Marble::PathsLayerPtr mPathsLayer;
        Marble::RegionsLayerPtr mRegionsLayer;
        Marble::CrosshairLayerPtr mCrosshairLayer;
        Marble::LocalMapLayerPtr mLocalMapLayer;
        Marble::LaserCloudLayerPtr mLaserCloudLayer;
        Marble::RobotManualPlacementLayerPtr mRobotPlacementLayer;

        bool mPathsVisible;
        bool mScaling;
        TextureManagerPtr mTextureManager;
        MapPlacesManagerPtr mMapPlacesManager;
        MapAbstraction::GeoMapSenderPtr mSender;
        MapAbstraction::GeoLocalMapReceiverPtr mLocalMapReceiver;
        MapAbstraction::LocalMapLoaderPtr mLocalMapLoader;
        ManualPositioningLogicPtr mManualPositioningLogic;
};


#endif // ROBOTICSMAP_H

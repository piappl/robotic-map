#ifndef ROBOTICSMAP_H
#define ROBOTICSMAP_H

#include <QFileInfo>
#include <QSharedPointer>
#include <marble/MarbleQuickItem.h>
#include "LocalMapLoader.h"
#include "MapLibraryTypes.h"
#include "InternalTypesFwd.h"
#include "IGeoMap.h"
#include "LaserScanPoint.h"
#include "DynamicObject.h"
#include "GeoCoords.h"
#include "Orientation.h"
#include "SensorReadings.h"
#include "Layers.h"

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
        void componentComplete();

    // IGeoMap interface
        MapAbstraction::GeoMapSenderPtr sender() const;
        MapAbstraction::GeoLocalMapReceiverPtr localMapReceiver() const;
        MapAbstraction::ISensorReadingsPtr sensorReadingsInterface() const;

        MapAbstraction::GeoObjectsManagerPtr geoManager() { return mGeoObjectsManager; }
        MapAbstraction::MapLogPlacemarkDataPtr mapLog();

    public slots:
        void updatePlacemark(GeoObjectID id, MapAbstraction::MapObjectPtr newObject);
        void updateFileGeometry(const QString &fileSource);
        void removeGeometry(const QString &keyName);
        void connectedToRobot(int robotID, bool connected);
        void updateRequested();

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
        void updateDynamicObjects(int robotID, MapAbstraction::DynamicObjects);
        void updateMapPlaces(MapAbstraction::MapPlaces places);

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
        void broadcastRelativePosition(MapAbstraction::MapRobotObjectPtr robotObjectData);

        MapAbstraction::GeoObjectsManagerPtr mGeoObjectsManager;
        MapKeyboardInputHandlerPtr mInputHandler;
        PlacemarkLogicPtr mPlacemarkLogic;
        TrackingCameraPtr mTrackingCamera;

        MapAbstraction::Layers mLayers;

        bool mPathsVisible = false;
        TextureManagerPtr mTextureManager;
        MapPlacesManagerPtr mMapPlacesManager;
        MapAbstraction::GeoMapSenderPtr mSender;
        MapAbstraction::GeoLocalMapReceiverPtr mLocalMapReceiver;
        MapAbstraction::LocalMapLoaderPtr mLocalMapLoader;
        ManualPositioningLogicPtr mManualPositioningLogic;
        MapAbstraction::SensorReadingsPtr mSensorReadings;
};


#endif // ROBOTICSMAP_H

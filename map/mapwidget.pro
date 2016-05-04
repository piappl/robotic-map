QT += widgets qml quick

DESTDIR = bin

TEMPLATE = lib
QMAKE_LFLAGS+= -Wl,--no-undefined
#CONFIG += staticlib

SOURCES += \
    MapFactory.cpp \
    MapIconProvider.cpp \
    GeoObjectsManager.cpp \
    MapWrap.cpp \
    MapLibraryHelpers.cpp \
    RobotHeartState.cpp \
    RobotHeartbeat.cpp \
    RobotGeoReferences.cpp \
    PlacemarkLogic.cpp \
    GuiController.cpp \
    AbstractTracker.cpp \
    PlacemarkTracker.cpp \
    OverviewTracker.cpp \
    TrackingCamera.cpp \
    MapLogPlacemarkData.cpp \
    TextureManager.cpp \
    MapObject.cpp \
    GeoCoords.cpp \
    MapRobotObject.cpp \
    MapPlacesManager.cpp \
    MapPlaceObject.cpp \
    MapWaypointObject.cpp \
    MapEditor.cpp \
    MapPolygonNodeObject.cpp \
    MapOrderedObject.cpp \
    GeoObjectID.cpp \
    GeoMapSender.cpp \
    LocalMapLayer.cpp \
    GeoLocalMapReceiver.cpp \
    LocalMapLogic.cpp \
    LaserCloudLayer.cpp \
    ManualPositioningLogic.cpp \
    RobotEditor.cpp \
    LocalMapLoader.cpp \
    RobotManualPlacementLayer.cpp \
    RegionsLayer.cpp \
    CrosshairLayer.cpp \
    PathsLayer.cpp \
    MapKeyboardInputHandler.cpp \
    RoboticsMap.cpp \
    ConfigurationLoader.cpp \
    MapIconPainter.cpp \
    MapIcons.cpp

INSTALL_ROOT = /usr/local
INCLUDEPATH += $${INSTALL_ROOT}/include/marble
target.path = $${INSTALL_ROOT}/lib
INSTALLS += target

INSTALL_HEADERS += \
    IGeoMapFwd.h \
    MapFactory.h \
    MapAbstractionFwd.h \
    MapObjectsFwd.h \
    MapWrap.h \
    MapRobotObject.h \
    MapPlaceObject.h \
    MapOrderedObject.h \
    MapPolygonNodeObject.h \
    MapWaypointObject.h \
    MapObject.h \
    RobotStates.h \
    PlacemarkType.h \
    GeoCoords.h \
    GeoMapSender.h \
    GeoLocalMapReceiver.h \
    LaserScanPoint.h \
    Orientation.h

LIBS += -lmarbledeclarative \
        -lmarblewidget-qt5

HEADERS += $${INSTALL_HEADERS} \
    IMapSignalReceiver.h \
    IGeoMap.h \
    MapIconProvider.h \
    MapLibraryTypes.h \
    GeoObjectsManager.h \
    MapLibraryHelpers.h \
    RobotHeartState.h \
    RobotHeartbeat.h \
    RobotGeoReferences.h \
    PlacemarkLogic.h \
    GuiController.h \
    TrackingCamera.h \
    AbstractTracker.h \
    PlacemarkTracker.h \
    OverviewTracker.h \
    InternalTypesFwd.h \
    MapLogPlacemarkData.h \
    GeoObjectID.h \
    TextureManager.h \
    MapPlacesManager.h \
    MapEditor.h \
    LocalMapLayer.h \
    LocalMapLogic.h \
    LaserScanPoint.h \
    LaserCloudLayer.h \
    RobotEditor.h \
    ManualPositioningLogic.h \
    LocalMapLoader.h \
    RobotManualPlacementLayer.h \
    MapLayersFwd.h \
    RegionsLayer.h \
    CrosshairLayer.h \
    PathsLayer.h \
    Orientation.h \
    MapObjectsFwd.h \
    MapKeyboardInputHandler.h \
    RoboticsMap.h \
    ConfigurationLoader.h \
    IconConfiguration.h \
    MapIconPainter.h \
    MapIcons.h

headers.files = $$INSTALL_HEADERS
headers.path = $${INSTALL_ROOT}/include/map
INSTALLS += headers

RESOURCES += resources.qrc



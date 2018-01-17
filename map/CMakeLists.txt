cmake_minimum_required(VERSION 2.8.11)
project(mapwidget)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Marble REQUIRED)

set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
set(CMAKE_AUTOMOC TRUE)

add_definitions("--std=c++11")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MARBLE_INCLUDE_DIR}
)

set(SOURCES
    MapFactory.cpp
    MapIconProvider.cpp
    RegionsLayer.cpp
    GeoObjectsManager.cpp
    PathsLayer.cpp
    LocalMapLayer.cpp
    LocalMapLogic.cpp
    LocalMapLoader.cpp
    MapKeyboardInputHandler.cpp
    MapWrap.cpp
    RoboticsMap.cpp
    MapLibraryHelpers.cpp
    RobotHeartState.cpp
    RobotHeartbeat.cpp
    RobotGeoReferences.cpp
    PlacemarkLogic.cpp
    GuiController.cpp
    AbstractTracker.cpp
    PlacemarkTracker.cpp
    OverviewTracker.cpp
    TrackingCamera.cpp
    MapLogPlacemarkData.cpp
    TextureManager.cpp
    MapObject.cpp
    GeoCoords.cpp
    MapRobotObject.cpp
    CrosshairLayer.cpp
    MapPlacesManager.cpp
    MapPlaceObject.cpp
    MapWaypointObject.cpp
    MapEditor.cpp
    MapPolygonNodeObject.cpp
    MapOrderedObject.cpp
    GeoObjectID.cpp
    GeoMapSender.cpp
    GeoLocalMapReceiver.cpp
    LaserCloudLayer.cpp
    RobotEditor.cpp
    RobotManualPlacementLayer.cpp
    DynamicObjectsLayer.cpp
    ManualPositioningLogic.cpp
    MapIconPainter.cpp
    MapIcons.cpp
    ConfigurationLoader.cpp
    SensorReadings.cpp
    SensorDataLayer.cpp
    SensorLayerControlGui.cpp
    ISensorReadings.cpp
    Layers.cpp
    MapLayerInterface.cpp
    WaypointsLayer.cpp)

set(INSTALL_HEADERS
    IGeoMapFwd.h
    MapFactory.h
    MapAbstractionFwd.h
    MapObjectsFwd.h
    MapWrap.h
    MapRobotObject.h
    MapPlaceObject.h
    MapPlaces.h
    MapOrderedObject.h
    MapPolygonNodeObject.h
    MapWaypointObject.h
    MapObject.h
    RobotStates.h
    PlacemarkType.h
    GeoCoords.h
    GeoMapSender.h
    GeoLocalMapReceiver.h
    LaserScanPoint.h
    Orientation.h
    UncertainCoords.h
    SensorInfo.h
    SensorData.h
    Mission.h
    ISensorReadings.h
)

qt5_add_resources(RESOURCES resources.qrc)
add_library(mapwidget SHARED ${SOURCES} ${RESOURCES})
target_link_libraries(mapwidget Qt5::Widgets Qt5::Quick Qt5::Core Marble MarbleDeclarative)

install(FILES ${INSTALL_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/map)
install_targets(/lib RUNTIME_DIRECTORY /lib mapwidget)
set(localmap_parentdir /share/maps)
install(DIRECTORY localmap DESTINATION ${localmap_parentdir})

#ADD_CUSTOM_COMMAND(TARGET mapwidget
#          POST_BUILD
#          COMMAND ${CMAKE_COMMAND} -E make_directory /share/maps/localmap
#          COMMAND ${CMAKE_COMMAND} -E copy localmap/* /share/maps/localmap/.
#)

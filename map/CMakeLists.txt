cmake_minimum_required(VERSION 2.8.11)
project(mapwidget)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Marble REQUIRED)

SET(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
set(CMAKE_AUTOMOC TRUE)
message(Found marble ${MARBLE_LIBRARIES} ${MARBLE_PATH})

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_INSTALL_PREFIX}/include/marble #MARBLE_INCLUDE_DIRS is empty for some reason
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
    MapReceiver.cpp
    LaserCloudLayer.cpp
    RobotEditor.cpp
    RobotManualPlacementLayer.cpp
    ManualPositioningLogic.cpp
    MapIconPainter.cpp
    MapIcons.cpp
    ConfigurationLoader.cpp
    DynamicObjectsLayer.cpp)

set(INSTALL_HEADERS
    IGeoMapFwd.h
    MapFactory.h
    MapAbstractionFwd.h
    MapObjectsFwd.h
    MapWrap.h
    MapRobotObject.h
    MapPlaceObject.h
    MapOrderedObject.h
    MapPolygonNodeObject.h
    MapWaypointObject.h
    MapObject.h
    RobotStates.h
    PlacemarkType.h
    GeoCoords.h
    GeoMapSender.h
    MapReceiver.h
    LaserScanPoint.h
    Orientation.h
    DynamicObject.h)

qt5_add_resources(RESOURCES resources.qrc)
add_library(mapwidget SHARED ${SOURCES} ${RESOURCES})
target_link_libraries(mapwidget Qt5::Widgets Qt5::Quick Qt5::Core ${MARBLE_LIBRARIES} marbledeclarative)

INSTALL(FILES ${INSTALL_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/map)
INSTALL_TARGETS(/lib RUNTIME_DIRECTORY /lib mapwidget)




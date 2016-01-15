#ifndef INTERNALTYPESFWD_H
#define INTERNALTYPESFWD_H
#include <QSharedPointer>

class AbstractTracker;
typedef QSharedPointer<AbstractTracker> AbstractTrackerPtr;

class PlacemarkLogic;
typedef QSharedPointer<PlacemarkLogic> PlacemarkLogicPtr;

class TrackingCamera;
typedef QSharedPointer<TrackingCamera> TrackingCameraPtr;

class TextureManager;
typedef QSharedPointer<TextureManager> TextureManagerPtr;

class MapPlacesManager;
typedef QSharedPointer<MapPlacesManager> MapPlacesManagerPtr;

class ManualPositioningLogic;
typedef QSharedPointer<ManualPositioningLogic> ManualPositioningLogicPtr;

class MapKeyboardInputHandler;
typedef QSharedPointer<MapKeyboardInputHandler> MapKeyboardInputHandlerPtr;

namespace MapAbstraction
{
    class MapLogPlacemarkData;
    typedef QSharedPointer<MapLogPlacemarkData> MapLogPlacemarkDataPtr;

    class GeoObjectsManager;
    typedef QSharedPointer<GeoObjectsManager> GeoObjectsManagerPtr;
}

namespace Marble
{
    class LocalMapLogic;
    typedef QSharedPointer<LocalMapLogic> LocalMapLogicPtr;
}

#endif // INTERNALTYPESFWD_H

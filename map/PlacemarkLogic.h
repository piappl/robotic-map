#ifndef PLACEMARKLOGIC_H
#define PLACEMARKLOGIC_H

#include <QMap>
#include <marble/GeoDataLatLonBox.h>
#include "InternalTypesFwd.h"
#include "MapObjectsFwd.h"
#include "MapLibraryTypes.h"
#include "GeoCoords.h"
#include "GeoObjectID.h"
#include "PlacemarkType.h"
#include "MapIconProvider.h"

namespace Marble { class MarbleModel; }
class GuiController;

//Component class for the map that encapsulates placemark managing operations
class PlacemarkLogic : public QObject
{
Q_OBJECT

signals:
    void placemarkUpdated(GeoObjectID placemarkIndex, MapAbstraction::MapObjectConstPtr object,
                          QString path);
    void placemarkRemoved(GeoObjectID placemarkIndex);
    void activeRobotPositionChanged(GeoObjectID id);
    void robotsVisibilityChanged(bool aVisible, bool aSimplified); //emitted when a global visibility changes
    void updateMarble();

public slots:
    void placemarkTypeVisibilityRequest(PlacemarkType type, bool makeVisible);

public:
    PlacemarkLogic(Marble::MarbleModel *model, MapAbstraction::GeoObjectsManagerPtr manager);

    void addOrUpdatePlacemark(const GeoObjectID &id, MapAbstraction::MapObjectPtr localizedData);
    void updatePlacemark(MapAbstraction::MapObjectPtr localizedData,
                         PlacemarkPtr placemark, bool checkChanges = true);
    void addMapPlace(const GeoObjectID &id, MapAbstraction::MapObjectPtr place);
    void updatePlacemarkIcon(PlacemarkPtr placemark);
    void removePlacemark(const GeoObjectID &id);
    void removePlacemark(PlacemarkConstPtr placemark);
    void removeAllPlacemarks(PlacemarkType category);
    bool placemarksVisible() const;
    void changeVisibility(bool visible);
    void changeVisibility(PlacemarkType type, bool visible);
    void togglePlacemarksVisibility();
    QVector<PlacemarkPtr> placemarksOfType(PlacemarkType category) const;

    Marble::GeoDataLatLonBox allPlacemarksGeoRect() const;
    MapAbstraction::MapLogPlacemarkDataPtr mapLog() const;

private:
    PlacemarkPtr createPlacemark(const GeoObjectID &id, MapAbstraction::MapObjectPtr localizedData);
    void updatePlacemarkVisibility(PlacemarkPtr placemark, MapAbstraction::MapObjectPtr newInfo);
    void updatePlacemarkIcon(PlacemarkPtr placemark, MapAbstraction::MapObjectPtr newInfo);
    void updatePlacemarkIcon(PlacemarkPtr placemark, QString iconPath, QImage icon);
    void updatePlacemarkPosition(PlacemarkPtr placemark, MapAbstraction::MapObjectPtr newInfo);
    void updatePlacemarkName(PlacemarkPtr placemark, MapAbstraction::MapObjectPtr newInfo);
    void createPlacemarkStyle(PlacemarkPtr placemark);
    void changeVisibility(bool visible, bool simplified);
    void applyVisibility();
    bool shouldPlaceBeVisibleOnList(MapAbstraction::MapObjectPtr place) const;
    bool shouldPlaceBeVisibleOnMap(MapAbstraction::MapObjectPtr place) const;
    void updateMarblePlacemark(PlacemarkPtr placemark);

    bool mPlacemarksVisible;
    bool mIconsSimplified;
    QMap<PlacemarkType, bool> mPlacemarksVisibilityMap;
    MapAbstraction::MapLogPlacemarkDataPtr mMapLog;
    Marble::MarbleModel* mMarbleModel;
    MapAbstraction::GeoObjectsManagerPtr mGeoManager;
    MapAbstraction::MapIconProvider mIconProvider;
};

#endif // PLACEMARKLOGIC_H

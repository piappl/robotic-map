#include "MapObject.h"
#include "MapIconProvider.h"

using namespace MapAbstraction;

MapObject::MapObject(GeoCoords coords, QString type, QString name,
                     QString description, LocalizationType localizationType,
                     LocalizationMode localizationMode)
    : mCoords(coords), mType(type), mName(name),
      mDescription(description), mVisible(true),
      mLocalizationType(localizationType), mLocalizationMode(localizationMode)
{
}

MapObject::~MapObject() {}

GeoCoords MapObject::coords() const { return mCoords; }
void MapObject::setCoords(GeoCoords coords) { mCoords = coords; }

QString MapObject::type() const { return mType; }
void MapObject::setType(QString type) { mType = type; }

QString MapObject::name() const { return mName; }
void MapObject::setName(QString name) { mName = name; }

QString MapObject::description() const { return mDescription; }
void MapObject::setDescription(QString description) { mDescription = description; }

QString MapObject::displayText() const { return name(); }

bool MapObject::visible() const { return mVisible; }
void MapObject::setVisible(bool visible) { mVisible = visible; }

MapObject::LocalizationType MapObject::localizationType() const { return mLocalizationType; }
void MapObject::setLocalizationType(LocalizationType type) { mLocalizationType = type; }
MapObject::LocalizationMode MapObject::localizationMode() const { return mLocalizationMode; }
void MapObject::setLocalizationMode(LocalizationMode mode) { mLocalizationMode = mode; }

bool MapObject::operator==(const MapObject &other) const
{
    return  (coords() == other.coords()
            && name() == other.name()
            && type() == other.type()
            && description() == other.description()
            && localizationType() == other.localizationType()
            && localizationMode() == other.localizationMode()
            && compare(other));
}


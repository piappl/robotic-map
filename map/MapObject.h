#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include <QString>
#include "GeoCoords.h"
#include "PlacemarkType.h"

namespace MapAbstraction
{
    class MapObject
    {
    public:
        enum LocalizationType
        {
            None,
            LocalRelative, //Odometry
            LocalAbsolute, //Local map autonomous navigation
            Global //GPS
        };

        enum LocalizationMode
        {
            Manual, //Can overwrite localization
            Assisted, //Can overwrite localization but synchronizes with object each time it is done
            Automatic //Can't overwrite localization
        };

        MapObject(GeoCoords coords, QString type, QString name,
                  QString description = QString(),
                  LocalizationType localizationType = Global,
                  LocalizationMode localizationMode = Automatic);
        virtual ~MapObject();

        GeoCoords coords() const;
        void setCoords(GeoCoords coords);
        QString type() const;
        void setType(QString type);
        QString name() const;
        void setName(QString name);
        QString description() const;
        void setDescription(QString description);
        bool visible() const;
        void setVisible(bool visible);
        LocalizationType localizationType() const;
        void setLocalizationType(LocalizationType type);
        LocalizationMode localizationMode() const;
        void setLocalizationMode(LocalizationMode mode);

        virtual QString displayText() const;

        bool operator==(const MapObject &other) const;
        virtual MapObject* Clone() const = 0;
        virtual PlacemarkType category() const = 0;

    protected:
        virtual bool compare(const MapObject& other) const = 0;

    private:
        GeoCoords mCoords;
        QString mType;
        QString mName;
        QString mDescription;
        bool mVisible;
        LocalizationType mLocalizationType; //What type of coords the object understands
        LocalizationMode mLocalizationMode;
    };
}
#endif // MAPOBJECT_H

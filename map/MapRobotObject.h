#ifndef MAPDATAOBJECTS_H
#define MAPDATAOBJECTS_H

#include <QUuid>
#include <QVector>
#include "RobotStates.h"
#include "MapObject.h"
#include "Orientation.h"

namespace MapAbstraction
{
    class MapRobotObject : public MapObject
    {
    public:
        Orientation orientation() const;
        void setOrientation(Orientation orientation);
        RobotState state() const;
        void setState(RobotState state);
        QString displayText() const;
        int robotID() const ;
        int consoleID() const;
        void setConsoleID(int value);
        bool active() const;
        bool connected() const;
        bool orientationAvaliable() const;
        bool positionAvailable() const;
        void setOrientationAvailable(bool orientation);
        void setGPSAvailable(bool available);

        PlacemarkType category() const;

        MapRobotObject(GeoCoords coords, Orientation orientation, RobotState state, QString type,
                            QString name, QString description, int robotID, int consoleID,
                            LocalizationType localizationType = MapObject::Global,
                            LocalizationMode localizationMode = MapObject::Automatic);
        MapObject* Clone() const;

    private:
        bool compare(const MapObject &other) const;

        bool mOrientationAvailable;
        bool mGPSAvailable;
        Orientation mOrientation;
        RobotState mState;
        int mRobotID;
        int mConsoleID;
    };
}


#endif // MAPDATAOBJECTS_H

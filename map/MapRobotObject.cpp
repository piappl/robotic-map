#include "MapRobotObject.h"

namespace MapAbstraction
{
    Orientation MapRobotObject::orientation() const { return mOrientation; } //in radians from south
    void MapRobotObject::setOrientation(Orientation orientation) { mOrientation = orientation; mOrientationAvailable = true; }

    bool MapRobotObject::orientationAvaliable() const { return mOrientationAvailable; }
    void MapRobotObject::setOrientationAvailable(bool available) { mOrientationAvailable = available; }

    bool MapRobotObject::positionAvailable() const
    {
        return localizationType() != MapObject::None;
    }

    RobotState MapRobotObject::state() const { return mState; }
    void MapRobotObject::setState(RobotState state) { mState = state; }

    int MapRobotObject::robotID() const { return mRobotID; }

    int MapRobotObject::consoleID() const { return mConsoleID; }
    void MapRobotObject::setConsoleID(int value) { mConsoleID = value; }

    bool MapRobotObject::active() const { return mState != RobotStateDisappeared; }
    bool MapRobotObject::connected() const { return mState == RobotStateConnected; }

    QString MapRobotObject::displayText() const
    {
        return type() + " " + name() + " " + QString::number(mRobotID);
    }

    MapRobotObject::MapRobotObject(GeoCoords coords, Orientation orientation, RobotState state, QString type,
                        QString name, QString description, int robotID, int consoleID,
                        LocalizationType localizationType, LocalizationMode localizationMode)
        : MapObject(coords, type, name, description, localizationType, localizationMode),
          mOrientationAvailable(false), mGPSAvailable(false),
          mOrientation(orientation), mState(state), mRobotID(robotID), mConsoleID(consoleID)
    {
    }

    bool MapRobotObject::compare(const MapObject &other) const
    {
        bool ret = false;
        const MapRobotObject* lob = dynamic_cast<const MapRobotObject*>(&other);
        if (lob)
        {
            ret =   orientationAvaliable() == lob->orientationAvaliable()
                    && positionAvailable() == lob->positionAvailable()
                    && orientation() == lob->orientation()
                    && state() == lob->state()
                    && robotID() == lob->robotID()
                    && consoleID() == lob->consoleID();
        }
        return ret;
    }

    PlacemarkType MapRobotObject::category() const { return PlacemarkRobot; }
    MapObject* MapRobotObject::Clone() const { return new MapRobotObject(*this); }
}

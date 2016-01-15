#ifndef MAPWRAP_H
#define MAPWRAP_H

#include <QObject>
#include "IGeoMapFwd.h"
#include "MapAbstractionFwd.h"

class QQuickItem;

namespace MapAbstraction
{
    class MapWrap : public QObject
    {   //This class allows for smooth embedding (through parent QQuickItem) and interfacing
        Q_OBJECT
    public:
        MapWrap(QQuickItem *viewMap, int consoleID);
        GeoMapSenderPtr sender() const;
        GeoLocalMapReceiverPtr localMapReceiver() const;

    public slots:
        void updateRobot(MapRobotObjectPtr robotInfo);
        void robotConnected(int robotID, bool connected);
        void robotAvailabilityChanged(int robotId, bool available);

    private:
        int mConsoleID;
        IGeoMapPtr mMap;

        RobotGeoReferencesPtr mGeoRefs;
        RobotHeartbeatPtr mRobotHeartbeat;
    };
    typedef QSharedPointer<MapWrap> MapWrapPtr;
}

#endif // MAPWRAP_H

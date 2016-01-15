#ifndef ROBOTHEARTBEAT_H
#define ROBOTHEARTBEAT_H

#include <QMap>
#include "IGeoMapFwd.h"
#include "MapAbstractionFwd.h"
#include "InternalTypesFwd.h"
#include "GeoObjectID.h"

namespace MapAbstraction
{
    typedef QMap<GeoObjectID, RobotHeartStatePtr> RobotStateMap;

    class RobotHeartbeat : public QObject
    {
        Q_OBJECT
        public:
            RobotHeartbeat(IGeoMapPtr geoMap, RobotGeoReferencesPtr geoRefs,
                           int consoleID);
            void robotActive(const GeoObjectID& id);
            void robotDisappeared(const GeoObjectID& id);

        private slots:
            void updateRobot(const GeoObjectID& id);

        private:
            int mConsoleID;
            IGeoMapPtr mGeoMap;
            RobotGeoReferencesPtr mGeoRefs;
            RobotStateMap mRobotHeartState;
    };
}

#endif // ROBOTHEARTBEAT_H

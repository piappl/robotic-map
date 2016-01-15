#include "RobotHeartbeat.h"
#include "RobotHeartState.h"
#include "RobotGeoReferences.h"
#include "RobotStates.h"
#include "MapRobotObject.h"
#include "IGeoMap.h"

namespace MapAbstraction
{
    RobotHeartbeat::RobotHeartbeat(IGeoMapPtr geoMap, RobotGeoReferencesPtr geoRefs,
                                   int consoleID)
        : mConsoleID(consoleID), mGeoMap(geoMap), mGeoRefs(geoRefs)
    {
    }

    void RobotHeartbeat::robotActive(const GeoObjectID& id)
    {
        if (!mRobotHeartState.contains(id))
        {
            RobotHeartStatePtr newRobotState(new RobotHeartState(id));
            connect(newRobotState.data(), SIGNAL(robotStateChanged(GeoObjectID)),
                    this, SLOT(updateRobot(GeoObjectID)));
            mRobotHeartState.insert(id, newRobotState);
        }
        else
        {
            RobotHeartStatePtr robotState = mRobotHeartState[id];
            robotState->setCommunicating(true);
        }
    }

    void RobotHeartbeat::robotDisappeared(const GeoObjectID& id)
    {
        Q_ASSERT(mRobotHeartState.contains(id));
        if (!mRobotHeartState.contains(id))
            return; //nothing to do here

        /*  TODO - we don't want it to just vanish without warning
        RobotHeartStatePtr robotState = mRobotHeartState[id];
        robotState->setAlive(false);
        */
    }

    void RobotHeartbeat::updateRobot(const GeoObjectID &id)
    {
        Q_ASSERT(mRobotHeartState.contains(id));
        if (!mRobotHeartState.contains(id))
            return; //We are not observing this robot - it was not set as available

        MapRobotObjectPtr data = mGeoRefs->getLocalizedObject(id);
        RobotHeartStatePtr robotState = mRobotHeartState[id];
        RobotState state = robotState->isAlive() ?
                    (robotState->isCommunicating() ?
                          (data->consoleID() == mConsoleID ?
                               RobotStateConnected : RobotStateNormal)
                          : RobotStateDisconnected)
                  : RobotStateDisappeared;
        data->setState(state);
        mGeoMap->updatePlacemark(id, data);
    }
}

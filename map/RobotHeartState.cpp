#include "RobotHeartState.h"

namespace MapAbstraction
{
    const int heartbeatStopMs = 3000; //3 seconds
    const int deadMs = 7500; //7 seconds (after heartbeat stops - total 10 seconds)

    RobotHeartState::RobotHeartState(const GeoObjectID& id)
        : mId(id), mIsCommunicating(true), mIsAlive(true)
    {
        mRobotHeartbeatStoppedTimer.setInterval(heartbeatStopMs);
        mRemoveRobotTimer.setInterval(deadMs);

        connect(&mRobotHeartbeatStoppedTimer, SIGNAL(timeout()), this, SLOT(disconnected()));
        connect(&mRemoveRobotTimer, SIGNAL(timeout()), this, SLOT(died()));

        mRobotHeartbeatStoppedTimer.start();
    }

    void RobotHeartState::setCommunicating(bool communicating)
    {
        if (!communicating)
        {
            disconnected();
        }
        else
        {
            revived();
            stillConnected();
        }
    }

    void RobotHeartState::setAlive(bool alive)
    {
        if (!alive)
        {
            died();
        }
        else
        {
            revived();
            stillConnected();
        }
    }

    void RobotHeartState::disconnected()
    {
        if (mIsCommunicating)
        {
            mRobotHeartbeatStoppedTimer.stop();
            mRemoveRobotTimer.start();
            mIsCommunicating = false;
            emit robotStateChanged(mId);
        }
    }

    void RobotHeartState::stillConnected()
    {
        if (mRemoveRobotTimer.isActive())
        {
            mRemoveRobotTimer.stop();
        }

        mRobotHeartbeatStoppedTimer.start();
        if (!mIsCommunicating)
        {
            mIsCommunicating = true;
            emit robotStateChanged(mId);
        }
    }

    void RobotHeartState::revived()
    {
        if (!mIsAlive)
        {
            mIsAlive = true;
            emit robotStateChanged(mId);
        }
    }

    void RobotHeartState::died()
    {
        mIsCommunicating = false;
        if (mIsAlive)
        {
            mIsAlive = false;
            emit robotStateChanged(mId);
        }
    }

    bool RobotHeartState::isCommunicating() const
    {
        return mIsCommunicating;
    }

    bool RobotHeartState::isAlive() const
    {
        return mIsAlive;
    }
}

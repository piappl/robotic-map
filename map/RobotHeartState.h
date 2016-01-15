#ifndef ROBOTHEARTSTATE_H
#define ROBOTHEARTSTATE_H

#include <QTimer>
#include <QTime>
#include "InternalTypesFwd.h"
#include "GeoObjectID.h"

namespace MapAbstraction
{
    class RobotHeartState : public QObject
    {
        Q_OBJECT
        signals:
            void robotStateChanged(GeoObjectID id);

        public:
            RobotHeartState(const GeoObjectID &id);
            void setCommunicating(bool communicating);
            void setAlive(bool alive);
            bool isCommunicating() const;
            bool isAlive() const;

        private slots:
            void disconnected();
            void stillConnected();
            void died();
            void revived();

        private:
            GeoObjectID mId;

            bool mIsCommunicating;
            bool mIsAlive;

            //Timer that is independent of lower logic layers
            QTimer mRobotHeartbeatStoppedTimer;

            //Timer that fires when we want robot to no longer show on the map
            QTimer mRemoveRobotTimer;
    };
}

#endif // ROBOTHEARTSTATE_H

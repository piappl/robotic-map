#ifndef ISENSORREADINGS_H
#define ISENSORREADINGS_H

#include <QList>
#include <QObject>
#include "SensorData.h"
#include "Mission.h"

namespace MapAbstraction
{
    class ISensorReadings : public QObject
    {
        Q_OBJECT
        signals:
            //Ask for a list of missions. Mission is an entity that groups sets of sensor readings in time interval.
            //The list should then be connected to come periodically on the slot of updateMissionList(..)
            void requestMissionList();

            //Ask for a list of sensor types used in the mission of a given ID (received through updateMissionList(..)).
            //The list will come back on the slot of updateSensorTypesList(..), periodically as long as the mission is open
            //(that is, not finished) or until another requestSensorList is sent for another mission.
            void requestSensorList(MissionID missionID);

            //Request to receive all data produced by the sensors on the list in the context of the mission. For open missions, the data
            //will then be updated until a request with other missionID is sent or the mission terminates. Sending another request
            //with other list of sensors will cause received data to update in this regard as well
            void requestSensorData(MissionID missionID, SensorIDList sensors);

            //When we close the map interface or hide the entire layer, we would like to stop the updates with data from coming
            //Whem we need data again, the requests have to be sent again
            void stopUpdates();

        public slots:
            virtual void updateMissionList(MissionList missions) = 0;
            virtual void updateSensorList(MissionID missionID, SensorInfoList sensors) = 0;
            virtual void updateSensorData(MissionID missionID, SensorData data) = 0;

        public:
            ISensorReadings();
            virtual ~ISensorReadings();
    };
}
#endif // ISENSORREADINGS_H

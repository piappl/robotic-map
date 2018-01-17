#ifndef SENSORREADINGS_H
#define SENSORREADINGS_H

#include "ISensorReadings.h"
#include "SensorDataLayer.h"
#include <QMap>

namespace MapAbstraction
{
    class SensorReadings : public ISensorReadings
    {
        Q_OBJECT
        public:
            SensorReadings(SensorDataLayerPtr layer);

        public slots:
            void updateMissionList(MissionList missions);
            void updateSensorList(MissionID missionID, SensorInfoList sensors);
            void updateSensorData(MissionID missionID, SensorData data);

            void testCallback();
            void missionsPopulate();

        private:
            SensorDataLayerPtr mDisplayLayer;
            QMap<MissionID, Mission> mMissions;
    };
    typedef QSharedPointer<SensorReadings> SensorReadingsPtr;
}
#endif // SENSORREADINGS_H

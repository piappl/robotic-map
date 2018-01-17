#include "SensorLayerControlGui.h"

using namespace MapAbstraction;

SensorLayerControlGui::SensorLayerControlGui(QQuickItem *parent) : QQuickItem(parent)
{
}

void SensorLayerControlGui::populateMissionList(MissionList missions)
{
    QList<MissionID> missionsAlive;
    foreach (Mission m, missions)
    {
        emit addOrUpdateMission(m.id, m.start.toString() + m.end.toString(),
                                m.name, m.description);
        missionsAlive.append(m.id);
    }

    foreach (Mission m, mLastMissions)
    {   //They were here but not any more - remove
        if (!missionsAlive.contains(m.id))
        {
            emit removeMission(m.id);
        }
    }

    mLastMissions = missions;
}

void SensorLayerControlGui::populateSensorList(MapAbstraction::MissionID mission, MapAbstraction::SensorInfoList sensors)
{
    foreach (SensorInfo s, sensors)
    {
        emit addOrUpdateSensor(s.uniqueID, s.sensorName, s.sensorDescription);
    }

    if (mLastSensors.contains(mission))
    {
        foreach (auto s, mLastSensors.value(mission))
        {
            emit removeSensor(s.uniqueID);
        }
    }
    mLastSensors[mission] = sensors;
}


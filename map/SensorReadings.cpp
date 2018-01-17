#include "SensorReadings.h"
#include "SensorDataLayer.h"
#include <QTimer>

using namespace MapAbstraction;
using namespace Marble;

SensorReadings::SensorReadings(SensorDataLayerPtr layer) : mDisplayLayer(layer)
{
    QTimer::singleShot(1000, this, SLOT(testCallback()));
    QTimer::singleShot(500, this, SLOT(missionsPopulate()));
}

void SensorReadings::updateMissionList(MissionList missions)
{
    mMissions.clear();
    foreach (Mission m, missions)
    {
        mMissions.insert(m.id, m);
    }
    //emit missionList(mMissions.values());
}

void SensorReadings::updateSensorList(MissionID missionID, SensorInfoList sensors)
{
    if (!mMissions.contains(missionID))
        return; //Ignore data, unknown mission
    //TODO
}

void SensorReadings::updateSensorData(MissionID missionID, SensorData data)
{
    if (!mMissions.contains(missionID))
        return; //Ignore data, unknown mission
    mDisplayLayer->updateSensorData(mMissions.value(missionID), data);
}


namespace
{
    Mission makeTestMission(int i = 0)
    {
        static int id_counter = 1;
        Mission m;
        int id = i == 0 ? id_counter : i;

        m.id = id;
        QString idstring = QString::number(id);
        m.name = "Uwolnić orkę " + idstring;
        m.description = "Misja numer " + idstring;

        QString stringStart = "27 04 2017 12:12:12";
        QString stringEnd = "27 04 2017 18:14:44";
        QString format = "dd MM yyyy hh:mm:ss";
        m.start = QDateTime::fromString(stringStart, format);
        m.end = QDateTime::fromString(stringEnd, format);
        m.robots.append(666);

        id_counter++;
        return m;
    }
}

void SensorReadings::missionsPopulate()
{
    if (mMissions.isEmpty())
    {
        for (int i = 0; i < 3; ++i)
        {
            Mission m = makeTestMission();
            mMissions.insert(m.id, m);
        }
    }
    updateMissionList(mMissions.values());
    QTimer::singleShot(2000, this, SLOT(missionsPopulate()));
}

void SensorReadings::testCallback()
{
    if (mMissions.count() > 0)
    {
        Mission testMission = mMissions.first();
        SensorData data;
        for (int i = 0; i < 25; ++i)
        {
            SensorDataPoint p;
            p.coords.mCoords.setLatitude(52.1942 + 0.002*i);
            p.coords.mCoords.setLongitude(20.9226 + 0.001*i);
            p.coords.mUncertaintity = i+10;
            p.timeOfReading = testMission.start.addSecs(i*60);
            MeasuredValue v;
            v.normalizedValue = 1.0-i*0.04;
            v.typeOfReading = DetectionRadiationGamma;
            v.value = 6666;
            p.readings.append(v);
            data.append(p);
        }
        updateSensorData(testMission.id, data);
    }
    QTimer::singleShot(1000, this, SLOT(testCallback()));
}

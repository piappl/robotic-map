#ifndef SENSORLAYERCONTROLGUI_H
#define SENSORLAYERCONTROLGUI_H

#include <QQuickItem>
#include "Mission.h"
#include "SensorInfo.h"

class SensorLayerControlGui : public QQuickItem
{
    Q_OBJECT
public:
    SensorLayerControlGui(QQuickItem *parent = 0);

public slots:
    void populateMissionList(MapAbstraction::MissionList missions);
    void populateSensorList(MapAbstraction::MissionID mission, MapAbstraction::SensorInfoList sensors);

signals:
    //To qml
    void addOrUpdateMission(int id, QString timeString, QString name, QString description);
    void addOrUpdateSensor(int id, QString name, QString sensorDescription);
    void removeMission(int id);
    void removeSensor(int id);

    void missionSelected(int id, bool selected); //single selection
    void sensorSelected(int id, bool selected); //multiple selection

    //To logic
    void missionViewOpened();
    void missionSelectionRequest(int id, bool selected);
    void sensorSelectionRequest(int id, bool selected);
    void missionViewClosed();

private:
    MapAbstraction::MissionList mLastMissions;
    QMap<MapAbstraction::MissionID, MapAbstraction::SensorInfoList> mLastSensors;
};

#endif //SENSORLAYERCONTROLGUI_H

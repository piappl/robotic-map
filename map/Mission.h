#ifndef MISSION_H
#define MISSION_H

#include "SensorInfo.h"
#include "UncertainCoords.h"
#include <QDateTime>

namespace MapAbstraction
{
    typedef int MissionID;

    class Mission
    {   //TODO - think about the correct way to construct the object and encapsulate it
    public:
        MissionID id;
        QList<int> robots; //robots used in the mission
        QDateTime start;
        QDateTime end;  //If this date is invalid, the mission is still in progress (check isValid())
        QString name;
        QString description;
    };

    typedef QList<Mission> MissionList;
}

#endif // MISSION_H

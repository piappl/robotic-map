#ifndef ROBOTICONCONFIGURATION_H
#define ROBOTICONCONFIGURATION_H

#include <QMap>
#include <QString>

struct RobotIconConfiguration
{
    QString robotName;
    QString mainIconPath;
};
typedef QMap<QString, RobotIconConfiguration> RobotIconConfigurationMap;

#endif // ROBOTICONCONFIGURATION_H

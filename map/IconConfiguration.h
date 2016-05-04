#ifndef ICONCONFIGURATION_H
#define ICONCONFIGURATION_H

#include <QMap>
#include <QString>

struct IconConfiguration
{
    QString name;
    QString mainIconPath;
};
typedef QMap<QString, IconConfiguration> IconConfigurationMap;

#endif // ICONCONFIGURATION_H

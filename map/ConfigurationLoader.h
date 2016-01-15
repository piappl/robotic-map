#ifndef CONFIGURATIONLOADER_H
#define CONFIGURATIONLOADER_H

#include <QString>
#include "RobotIconConfiguration.h"

class ConfigurationLoader
{
public:
    ConfigurationLoader();
    void loadConfigurations();
    QString getIconPath(QString robotNameID) const;

private:
    RobotIconConfigurationMap mRobotIcons;
};

#endif // CONFIGURATIONLOADER_H

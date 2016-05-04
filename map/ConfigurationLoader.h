#ifndef CONFIGURATIONLOADER_H
#define CONFIGURATIONLOADER_H

#include <QString>
#include "IconConfiguration.h"

class ConfigurationLoader
{
public:
    ConfigurationLoader();
    void loadConfigurations();
    QString getIconPath(QString nameID) const;

private:
    IconConfigurationMap mIcons;
};

#endif // CONFIGURATIONLOADER_H

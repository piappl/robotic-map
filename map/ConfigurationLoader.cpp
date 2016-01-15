#include <QDir>
#include "ConfigurationLoader.h"

namespace
{
    const QString customIconsDir = "/share/maps/customicons/";
    const QString suffix = "png";
}

ConfigurationLoader::ConfigurationLoader()
{
    loadConfigurations();
}

void ConfigurationLoader::loadConfigurations()
{
    qDebug("Load configurations");
    QDir iconsDir(customIconsDir);
    QFileInfoList list = iconsDir.entryInfoList(QDir::Files);
    foreach (QFileInfo entry, list)
    {
        qDebug("One entry");
        if (entry.exists() && entry.isReadable() && entry.isFile() && entry.suffix() == suffix)
        {   //A png file
            qDebug("Found file %s, robot name %s", qPrintable(entry.absoluteFilePath()), qPrintable(entry.baseName()));
            QString robotName = entry.baseName();
            RobotIconConfiguration robotEntry;
            robotEntry.mainIconPath = entry.absoluteFilePath();
            robotEntry.robotName = robotName;
            mRobotIcons.insert(robotName, robotEntry);
        }
    }
}

QString ConfigurationLoader::getIconPath(QString robotNameID) const
{
    qDebug("/nGET ICON PATH -- %s/n", qPrintable(robotNameID));
    if (!mRobotIcons.contains(robotNameID))
        return QString();

    return mRobotIcons.value(robotNameID).mainIconPath;
}


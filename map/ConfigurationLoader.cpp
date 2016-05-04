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
    qDebug("Loading configurations");
    QDir iconsDir(customIconsDir);
    QFileInfoList list = iconsDir.entryInfoList(QDir::Files);
    foreach (QFileInfo entry, list)
    {
        if (entry.exists() && entry.isReadable() && entry.isFile() && entry.suffix() == suffix)
        {   //A png file
            //qDebug("Found file %s, robot name %s", qPrintable(entry.absoluteFilePath()), qPrintable(entry.baseName()));
            QString name = entry.baseName();
            IconConfiguration iconEntry;
            iconEntry.mainIconPath = entry.absoluteFilePath();
            iconEntry.name = name;
            mIcons.insert(name, iconEntry);
        }
    }
}

QString ConfigurationLoader::getIconPath(QString nameID) const
{
    if (!mIcons.contains(nameID))
        return QString();

    return mIcons.value(nameID).mainIconPath;
}


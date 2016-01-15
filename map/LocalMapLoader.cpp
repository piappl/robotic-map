#include "LocalMapLoader.h"
#include <QFileInfo>
#include <QDir>
#include <QSettings>
#include "RoboticsMap.h"

using namespace MapAbstraction;

namespace
{
    const QString localMapFirstPath = "/share/maps/localmap/";
    const QString localMapSecondPath = QDir::homePath() + "/mapwidget/map/localmap/";
    const QString mapName = "localMap";
    const QString localMapFile = mapName + ".png";
    const QString configFile = mapName + ".cfg";

    bool checkFile(QString path)
    {
        QFileInfo info(path);
        if (!info.exists() || !info.isFile() || !info.isReadable())
        {
            //qWarning("LocalMapLoader found no map at %s!", path.toStdString().c_str());
            return false;
        }
        return true;
    }

    QString getLocalMapFilePath(QString localFile)
    {
        QString firstSearch(localMapFirstPath + localFile);
        bool check = checkFile(firstSearch);
        if (check)
        {
            return firstSearch;
        }

        QString secondSearch(localMapSecondPath + localFile);
        check = checkFile(secondSearch);
        if (!check)
        {
            qWarning("LocalMapLoader: No local map available!");
            return QString();
        }
        return secondSearch;
    }
}

LocalMapLoader::LocalMapLoader(RoboticsMap *map)
{
    connect(this, SIGNAL(localMapLoaded(QString,qreal,MapAbstraction::GeoCoords,MapAbstraction::Orientation,QPointF)),
            map, SLOT(updateLocalMap(QString,qreal,MapAbstraction::GeoCoords,MapAbstraction::Orientation,QPointF)));
    readConfig();
}

void LocalMapLoader::readConfig()
{
    QString mapFilePath = getLocalMapFilePath(localMapFile);
    if (mapFilePath.isEmpty())
        return; //No local map

    QString configFilePath = getLocalMapFilePath(configFile); //TODO - this is prone to corrupt dirs

    QSettings config(configFilePath, QSettings::IniFormat);

    bool ok;
    qreal resolution = config.value("scale/pixelmeters").toFloat(&ok);
    if (!ok)
    {
        qWarning("LocalMapLoader couldn't read map scale from the file!");
        return;
    }

    qreal lat = config.value("location/latitude").toFloat(&ok);
    if (!ok)
    {
        qWarning("LocalMapLoader couldn't read map initial center latitude in decimal degrees!");
        return;
    }

    qreal lon = config.value("location/longitude").toFloat(&ok);
    if (!ok)
    {
        qWarning("LocalMapLoader couldn't read map initial center longitude in decimal degrees!");
        return;
    }

    qreal rotation = config.value("location/rotation").toFloat(&ok);
    if (!ok)
    {
        qWarning("LocalMapLoader wrong or no value for rotation, assuming 0");
        rotation = 0;
    }

    qreal originX = config.value("location/originX", 0.0).toFloat(&ok);
    if (!ok)
    {
        qWarning("LocalMapLoader wrong value of originX, assuming 0");
        originX = 0;
    }

    qreal originY = config.value("location/originY", 0.0).toFloat(&ok);
    if (!ok)
    {
        qWarning("LocalMapLoader wrong value of originY, assuming 0");
        originY = 0;
    }


    qDebug("Local map loaded");
    GeoCoords centerApprox(lon, lat);
    emit localMapLoaded(mapFilePath, resolution, centerApprox, rotation, QPointF(originX, originY));
}

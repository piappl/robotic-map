#ifndef MAPICONPROVIDER_H
#define MAPICONPROVIDER_H

#include <QImage>
#include "MapObjectsFwd.h"
#include "ConfigurationLoader.h"
#include "MapIconPainter.h"

namespace MapAbstraction
{
    class MapIconProvider
    {
    public:
        QImage getIcon(MapObjectConstPtr object, bool isSimplified = false, bool isSelected = false);
        QString getIconPath(MapObjectConstPtr object, bool isSimplified = false, bool isSelected = false);
    private:
        QString getRobotIconPath(MapRobotObjectConstPtr robot, bool isSimplified = false);

        ConfigurationLoader mRobotConfigurations;
        MapIconPainter mMapIconPainter;
    };
}

#endif // MAPICONPROVIDER_H

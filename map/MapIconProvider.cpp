#include "MapIconProvider.h"
#include "MapObject.h"
#include "MapRobotObject.h"
#include "MapWaypointObject.h"
#include "MapIcons.h"
#include "MapIconPainter.h"

using namespace MapAbstraction;

namespace
{
    MapIcons::DefaultMapIcon getPlaceIconType(MapObjectConstPtr object, bool selected)
    {
        switch (object->category())
        {
            case PlacemarkPlace:
                return MapIcons::PointIcon;
            case PlacemarkWaypoint:
                {
                    MapWaypointObjectConstPtr waypoint = object.staticCast<const MapWaypointObject>();
                    return waypoint->reached() ? MapIcons::ReachedWaypointIcon
                                               : (selected ? MapIcons::SelectedWaypointIcon
                                                           : MapIcons::WaypointIcon);
                }
            case PlacemarkPolygonNode:
                return MapIcons::PolygonNodeIcon;
            default:
                return MapIcons::NoIcon;
        }
    }
}


QString MapIconProvider::getRobotIconPath(MapRobotObjectConstPtr robot, bool isSimplified)
{
    if (isSimplified || robot->state() == RobotStateDisappeared)
    {   //Use generic icon
        MapIcons::DefaultMapIcon type;
        if (robot->state() == RobotStateDisappeared)
        {
            type = MapIcons::NoIcon;
        }
        else
        {
            if (robot->orientationAvaliable())
                type = MapIcons::TriangleIcon;
            else
                type = MapIcons::PointIcon;
        }
        return MapIcons::defaultIconPath(type, robot->state());
    }

    //Use specific icon if available
    QString specificIconPath = mRobotConfigurations.getIconPath(robot->type());
    if (!specificIconPath.isEmpty())
    {
        return specificIconPath;
    }

    //Return default, generic robot icon
    return MapIcons::defaultRobotIconPath();
}

QString MapIconProvider::getIconPath(MapObjectConstPtr object, bool isSimplified, bool isSelected)
{
    if (object->category() == PlacemarkRobot)
    {
        MapRobotObjectConstPtr robot = object.staticCast<const MapRobotObject>();
        return getRobotIconPath(robot, isSimplified);
    }
    else
    {
        MapIcons::DefaultMapIcon type = getPlaceIconType(object, isSelected);
        return MapIcons::defaultIconPath(type);
    }
}

QImage MapIconProvider::getIcon(MapObjectConstPtr object, bool isSimplified, bool isSelected)
{
    QImage icon(getIconPath(object, isSimplified, isSelected));
    return mMapIconPainter.paintIcon(icon, object, isSimplified);
}

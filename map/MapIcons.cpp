#include "MapIcons.h"
#include "RobotStates.h"

namespace
{
    QString prefix = ":mapwidget/icons/";
    QString extension = ".png";
}

QString MapIcons::iconName(DefaultMapIcon type)
{
    switch (type)
    {
        case ConnectedIcon: return "connected";
        case DisconnectedIcon: return "disconnected";
        case PointIcon: return "point";
        case WaypointIcon: return "waypoint";
        case ReachedWaypointIcon: return "waypointReached";
        case SelectedWaypointIcon: return "waypointSelected";
        case PolygonNodeIcon: return "polygonNode";
        case SelectedPolygonNodeIcon: return "polygonNodeSelected";
        case TriangleIcon: return "triangle";
        case PedestrianIcon: return "pedestrian";
        case NoIcon: return "noIcon"; 
        default: return "noIcon";
    }
}

QString MapIcons::defaultIconPath(DefaultMapIcon icon, RobotState state)
{
    if (icon == TriangleIcon || icon == PointIcon)
    {
        QString suffix;
        if (state == RobotStateConnected)
            suffix = "c";
        else if (state == RobotStateDisconnected)
            suffix = "dc";
        return prefix + iconName(icon) + suffix + extension;
    }
    return defaultIconPath(icon);
}

QString MapIcons::defaultIconPath(DefaultMapIcon icon)
{
    return prefix + iconName(icon) + extension;
}

QString MapIcons::defaultRobotIconPath()
{
    return prefix + "defaultRobot" + extension;
}

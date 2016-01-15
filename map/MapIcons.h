#ifndef MAPICONS_H
#define MAPICONS_H

#include <QString>
#include "RobotStates.h"

class MapIcons
{
    public:
        enum DefaultMapIcon
        {
            NoIcon,
            ConnectedIcon,
            DisconnectedIcon,
            WaypointIcon,
            ReachedWaypointIcon,
            SelectedWaypointIcon,
            PolygonNodeIcon,
            SelectedPolygonNodeIcon,
            TriangleIcon,
            PointIcon
        };

        static QString iconName(DefaultMapIcon type);
        static QString defaultIconPath(DefaultMapIcon icon);
        static QString defaultIconPath(DefaultMapIcon icon, RobotState state);
        static QString defaultRobotIconPath();
};

#endif // MAPICONS_H

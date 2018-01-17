#ifndef SENSORDATACOLORS_H
#define SENSORDATACOLORS_H

#include <QColor>
#include <QMap>
#include "SensorInfo.h"

namespace SensorDataColors
{
    QColor colorForSensorType(MapAbstraction::DetectionType type)
    {
        static const QMap<MapAbstraction::DetectionType, QColor> dict
        {
            {MapAbstraction::DetectionNoDetection, Qt::white},
            {MapAbstraction::DetectionRadiationAlpha, Qt::magenta},
            {MapAbstraction::DetectionRadiationBeta, Qt::darkMagenta},
            {MapAbstraction::DetectionRadiationGamma, Qt::yellow}
        };

        if (dict.contains(type))
            return dict.value(type);

        return Qt::gray;
    }
}

#endif //SENSORDATACOLORS_H

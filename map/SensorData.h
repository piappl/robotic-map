#ifndef SENSORDATA_H
#define SENSORDATA_H

#include "SensorInfo.h"
#include "UncertainCoords.h"
#include <QDateTime>

namespace MapAbstraction
{
    struct MeasuredValue
    {
        DetectionType typeOfReading;
        double value;          //What we display
        float normalizedValue; //0-1, what we use to guide visuals
    };

    class SensorDataPoint
    {
    public:
        UncertainCoords coords;
        QDateTime timeOfReading;
        QList<MeasuredValue> readings;
    };
    typedef QList<SensorDataPoint> SensorData;
}

#endif // SENSORDATA_H

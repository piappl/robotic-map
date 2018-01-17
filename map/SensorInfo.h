#ifndef SENSOR_H
#define SENSOR_H

#include <QString>

namespace MapAbstraction
{
    typedef int SensorID;
    typedef short DetectionType;

    enum RecognizedDetectionTypes
    {
        DetectionNoDetection,   //HelperValue
        DetectionRadiationAlpha,
        DetectionRadiationBeta,
        DetectionRadiationGamma
    };

    //A data type (i.e. radiological gamma contamination) with human-readable description
    struct SensorReadingInfo
    {
        DetectionType type;
        QString name;   //Don't want to hold dictionary type->name here, we support unknown sensors of known type
    };

    //Carries human-readable information about sensor. A single sensor can read several types of inputs
    struct SensorInfo
    {
        SensorID uniqueID;
        QList<SensorReadingInfo> typeIDs;
        QString sensorName;
        QString sensorDescription;
    };

    typedef QList<SensorID> SensorIDList;
    typedef QList<SensorInfo> SensorInfoList;
}
#endif // SENSOR_H

#include <marble/GeoPainter.h>
#include "SensorDataLayer.h"
#include "MapLibraryHelpers.h"
#include "SensorDataColors.h"
#include "RoboticsMap.h"

using namespace Marble;
using namespace MapAbstraction;

namespace
{
    struct GeoPaintPoint
    {
        GeoDataCoordinates coords;
        qreal opacityModifier;
        QColor centerColor;
        qreal normalizedValue;
    };

    qint64 getMissionDuration(MapAbstraction::Mission m)
    {
        QDateTime missionStart = m.start;
        QDateTime missionEnd = m.end.isValid() ? m.end : QDateTime::currentDateTime();
        qWarning("Start %s, End %s", qPrintable(missionStart.toString()), qPrintable(missionEnd.toString()));
        if (missionStart > missionEnd)
        {   //Corrupted data, but it's for transparency only, so just sanitize
            missionEnd = missionStart;
        }
        return missionStart.secsTo(missionEnd);
    }

    qreal getOpacityModifier(qint64 timeSinceStart, qint64 rangeSeconds)
    {
        if (rangeSeconds == 0)
            return 0;

        const qreal opacityRange = 0.1;
        qreal opacityFactor = timeSinceStart/rangeSeconds;
        //qWarning("getOpacityMod %lld %lld %lf", rangeSeconds, timeSinceStart, opacityFactor);
        qreal opacityModifier = qBound<double>(0, opacityRange * (1 - opacityFactor), 0.1);
        return opacityModifier;
    }
}

SensorDataLayer::SensorDataLayer(RoboticsMap *rm) : MapLayerInterface(rm)
{
}

void SensorDataLayer::updateSensorData(MapAbstraction::Mission mission, MapAbstraction::SensorData data)
{
    mSensorData = data;
    mMission = mission;
    mHasContent = true;
}

QStringList SensorDataLayer::renderPosition() const
{
    return QStringList() << "HOVERS_ABOVE_SURFACE";
}

qreal SensorDataLayer::zValue() const
{
    const qreal SensorDataLayerZPosition = 2.5;
    return SensorDataLayerZPosition;
}

bool SensorDataLayer::render(GeoPainter *painter, ViewportParams*, const QString&, GeoSceneLayer*)
{
    if (!visible() || !mHasContent) //TODO
        return true;

    bool drawGPScircles = false;

    painter->save();
    qint64 missionDuration = getMissionDuration(mMission);

    if (drawGPScircles)
    {
        //We will draw gps uncertaintity circles with this
        QPen pen(Oxygen::woodBrown1);
        pen.setWidth(2);
        QBrush brush(Oxygen::woodBrown2, Qt::SolidPattern);
        painter->setBrush(brush);
        painter->setOpacity(0.2);
        painter->setPen(pen);
    }

    QList<GeoPaintPoint> paintPoints;
    foreach (SensorDataPoint point, mSensorData)
    {
        QDateTime timeOfReading = qBound(mMission.start, point.timeOfReading,
                                         mMission.end.isValid() ? mMission.end : QDateTime::currentDateTime());
        qreal timeInMission = qMin(mMission.start.secsTo(timeOfReading), missionDuration);
        qreal opacityModifier = getOpacityModifier(timeInMission, missionDuration);

        GeoDataCoordinates marbleCoords = MapLibraryHelpers::transformCoords(point.coords.mCoords);
        foreach (MeasuredValue reading, point.readings)
        {
            GeoPaintPoint p;
            p.coords = marbleCoords;
            p.centerColor = SensorDataColors::colorForSensorType(reading.typeOfReading);
            p.opacityModifier = opacityModifier;
            p.normalizedValue = reading.normalizedValue;
            paintPoints.append(p);
        }

        if (drawGPScircles)
        {
            qreal planetR = 6371; //TODO!! const
            qreal radiusMeters = point.coords.mUncertaintity;
            qreal radiansHeight = radiusMeters / planetR;

            //Latitude circles are smaller as latitude moves away from the equator: cos(lat)
            qreal metersInRadianOfLatitude = cos(marbleCoords.latitude()) * planetR;
            qreal radiansWidth = radiusMeters / metersInRadianOfLatitude;

            //qWarning("Render drawEllipse start %lf %lf", marbleCoords.latitude(), marbleCoords.longitude());
            painter->drawEllipse(marbleCoords, radiansWidth, radiansHeight, true);
        }
    }

    QPen pointPen(Oxygen::sunYellow1);
    foreach (GeoPaintPoint p, paintPoints)
    {
        painter->setOpacity(0.75+p.opacityModifier);
        pointPen.setColor(p.centerColor);
        pointPen.setWidth(2 + p.normalizedValue * 8);
        painter->setPen(pointPen);
        painter->drawPoint(p.coords);
        //qWarning("Render 4, coords %lf %lf %lf", p.coords.latitude(), p.coords.longitude(), 0.75+p.opacityModifier);
    }

    painter->restore();
    return true;
}

#ifndef SENSORDATALAYER_H
#define SENSORDATALAYER_H

#include "MapLayerInterface.h"
#include <QSharedPointer>
#include "SensorData.h"
#include "Mission.h"

namespace MapAbstraction
{
    class SensorDataLayer : public MapLayerInterface
    {
    Q_OBJECT
    public:
        SensorDataLayer(RoboticsMap *rm);
        void resetSensorData(); //New missionID selected
        LayerType type() const { return LayerSensorData; }
         //We assume that it is sent ONLY when data has changed somehow
        void updateSensorData(MapAbstraction::Mission mission,
                              MapAbstraction::SensorData data);

        QStringList renderPosition() const;
        qreal zValue() const;
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport, const QString &renderPos = "NONE",
                    Marble::GeoSceneLayer *layer = 0);
        bool hasContent() const;

    private:
        MapAbstraction::SensorData mSensorData;
        MapAbstraction::Mission mMission;
        bool mHasContent = false;
    };
    typedef QSharedPointer<SensorDataLayer> SensorDataLayerPtr;
}

#endif // SENSORDATALAYER_H

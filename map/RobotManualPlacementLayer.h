#ifndef ROBOTMANUALPLACEMENTLAYER_H
#define ROBOTMANUALPLACEMENTLAYER_H

#include "MapLayerInterface.h"
#include <marble/GeoDataCoordinates.h>
#include "GeoCoords.h"
#include <marble/MarbleMap.h>

namespace MapAbstraction
{
    class RobotManualPlacementLayer : public MapLayerInterface
    {
    Q_OBJECT
    signals:
        void orientationUpdate(MapAbstraction::GeoCoords newPoint);

    public:
        RobotManualPlacementLayer(RoboticsMap *rm);
        QStringList renderPosition() const;
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport,
                    const QString &renderPos = "NONE", Marble::GeoSceneLayer *layer = 0);
        LayerType type() const { return LayerRobotManualPlacement; }
        bool handleEvent(QObject *o, QEvent *e);
        void setReferencePoint(Marble::GeoDataCoordinates coords);

    private:
        Marble::MarbleMap *mMap;

        bool mTracking = false;
        Marble::GeoDataCoordinates mMouseCoords;
        Marble::GeoDataCoordinates mReferenceCoords;
    };
    typedef QSharedPointer<RobotManualPlacementLayer> RobotManualPlacementLayerPtr;
}


#endif // ROBOTMANUALPLACEMENTLAYER_H

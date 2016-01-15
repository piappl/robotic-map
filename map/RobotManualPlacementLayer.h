#ifndef ROBOTMANUALPLACEMENTLAYER_H
#define ROBOTMANUALPLACEMENTLAYER_H

#include <marble/LayerInterface.h>
#include <marble/GeoDataCoordinates.h>
#include "InternalTypesFwd.h"
#include "GeoCoords.h"

namespace Marble
{
    class MarbleMap;
    class RobotManualPlacementLayer : public QObject, public LayerInterface
    {
    Q_OBJECT
    signals:
        void orientationUpdate(MapAbstraction::GeoCoords newPoint);

    public:
        RobotManualPlacementLayer(MarbleMap *map);
        QStringList renderPosition() const;
        bool render(GeoPainter *painter, ViewportParams *viewport,
                    const QString &renderPos = "NONE", GeoSceneLayer *layer = 0);
        void setVisibility(bool visible);
        bool visible() const;

        bool handleEvent(QObject *o, QEvent *e);
        void setReferencePoint(GeoDataCoordinates coords);

    private:
        bool mVisible;
        MarbleMap *mMap;

        bool mTracking;
        GeoDataCoordinates mMouseCoords;
        GeoDataCoordinates mReferenceCoords;
    };
}


#endif // ROBOTMANUALPLACEMENTLAYER_H

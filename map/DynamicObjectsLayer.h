#ifndef DYNAMICOBJECTSLAYER_H
#define DYNAMICOBJECTSLAYER_H

#include <QTimer>
#include "MapLayerInterface.h"
#include "DynamicObject.h"
#include "LocalMapLayer.h"

namespace MapAbstraction
{
    class DynamicObjectsLayer : public MapLayerInterface
    {
    Q_OBJECT
    public:
        DynamicObjectsLayer(RoboticsMap *rm, LocalMapLayerPtr localMap);
        void updateContent(MapAbstraction::DynamicObjects objects, MapAbstraction::MapRobotObjectConstPtr robot);
        LayerType type() const { return LayerDynamicObjects; }

        QStringList renderPosition() const;
        qreal zValue() const;
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport, const QString &renderPos = "NONE",
                    Marble::GeoSceneLayer *layer = 0);
        bool hasContent() const;

    private slots:
        void cleanDynamicObjects();

    private:
        LocalMapLayerPtr mLocalMap;
        MapAbstraction::DynamicObjects mDynamicObjects;
        MapAbstraction::MapRobotObjectConstPtr mRobot;

        bool mHasContent = false;
        QTimer mCleanupTimer;
    };
    typedef QSharedPointer<DynamicObjectsLayer> DynamicObjectsLayerPtr;
}

#endif // DYNAMICOBJECTSLAYER_H

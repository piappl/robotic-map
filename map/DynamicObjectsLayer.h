#ifndef DYNAMICOBJECTSLAYER_H
#define DYNAMICOBJECTSLAYER_H

#include <QTimer>
#include <marble/LayerInterface.h>
#include "InternalTypesFwd.h"
#include "MapAbstractionFwd.h"
#include "DynamicObject.h"
#include "MapLayersFwd.h"
#include "MapIconProvider.h"

namespace Marble
{
    class DynamicObjectsLayer : public QObject, public LayerInterface
    {
    Q_OBJECT
    public:
        DynamicObjectsLayer(LocalMapLayerPtr map);
        void updateContent(MapAbstraction::DynamicObjects objects, MapAbstraction::MapRobotObjectConstPtr robot);

        QStringList renderPosition() const;
        qreal zValue() const;
        bool render(GeoPainter *painter, ViewportParams *viewport, const QString &renderPos = "NONE", GeoSceneLayer *layer = 0);
        void setVisible(bool visible);
        bool visible() const;
        bool hasContent() const;

    private slots:
        void cleanDynamicObjects();

    private:
        LocalMapLayerPtr mLocalMap;
        MapAbstraction::DynamicObjects mDynamicObjects;
        MapAbstraction::MapRobotObjectConstPtr mRobot;

        bool mVisible;
        bool mHasContent;
        QTimer mCleanupTimer;
        MapAbstraction::MapIconProvider mIconProvider;
    };
}

#endif // DYNAMICOBJECTSLAYER_H

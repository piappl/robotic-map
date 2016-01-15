#ifndef LOCALMAPLAYER_H
#define LOCALMAPLAYER_H

#include <marble/LayerInterface.h>
#include <marble/GeoDataLatLonBox.h>
#include <marble/GeoDataDocument.h>

#include "InternalTypesFwd.h"
#include "MapAbstractionFwd.h"
#include "GeoObjectID.h"

class RoboticsMap;

namespace Marble
{
    class MarbleModel;
    class MarbleMap;
    class GeoDataGroundOverlay;

    class LocalMapLayer : public QObject, public LayerInterface
    {
    Q_OBJECT

    signals:
        void localMapHasContent();
        void localMapVisibilityChanged(bool visible);

    public:
        LocalMapLayer(MarbleModel* model, MarbleMap *map);
        ~LocalMapLayer();
        void setLayerContent(const QString &overlayFile, qreal resolution,
                             const GeoDataCoordinates &center, qreal rotation, QPointF origin);

        QStringList renderPosition() const;
        qreal zValue() const;
        bool render(GeoPainter *painter, ViewportParams *viewport, const QString &renderPos = "NONE", GeoSceneLayer *layer = 0);
        void setVisible(bool visible);
        void toggleManualGeolocalization();
        bool visible() const;
        bool hasContent() const;

        void notifyState(); //This is a sign of a bigger problem with connecting slots after constructors

        //Is candidate box a good perspective to see any part of our local map?
        bool isValidPerspective(const GeoDataLatLonBox &candidate) const;

        GeoDataCoordinates localToGlobal(const QPointF &localPoint) const;
        GeoDataCoordinates robotToGlobal(const QPointF &robotLocalPoint,
                                         MapAbstraction::MapRobotObjectConstPtr robotObject) const;
        qreal localToGlobalRotation(qreal localRotation) const;
        //returns false if global address falls outside of the local map scope
        bool globalToLocal(const GeoDataCoordinates &globalPoint,
                           QPointF &localPoint) const;
        bool globalToRobot(const GeoDataCoordinates &globalPoint,
                           MapAbstraction::MapRobotObjectPtr robot, QPointF &robotPoint) const;

        bool hasReferenceFrame(GeoObjectID id) const;
        void updateRefereceRobotFrame(GeoObjectID id, MapAbstraction::MapRobotObjectPtr robot);
        MapAbstraction::MapRobotObjectPtr getReferenceFrame(GeoObjectID id) const;

        bool handleEvent(QObject *o, QEvent *e);

    private slots:
        void reloadContent() const;

    private:
        typedef QMap<GeoObjectID, MapAbstraction::MapRobotObjectPtr> RobotReferenceFrames;

        //Source file for the image overlay icon
        LocalMapLogicPtr mLocalMapLogic;
        MarbleModel *mModel;
        MarbleMap *mMap;

        GeoDataGroundOverlay *mOverlay;
        GeoDataDocument mMainLocalMapDocument;
        bool mVisible;
        bool mHasContent;
        bool mManualGeolocalization;
        RobotReferenceFrames mRobotReferenceFrames;
    };
}

#endif // LOCALMAPLAYER_H

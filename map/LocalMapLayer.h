#ifndef LOCALMAPLAYER_H
#define LOCALMAPLAYER_H

#include <QMap>
#include <marble/LayerInterface.h>
#include <marble/GeoDataLatLonBox.h>
#include <marble/GeoDataDocument.h>

#include "MapObjectsFwd.h"
#include "MapLayerInterface.h"
#include "GeoObjectID.h"
#include "LocalMapLogic.h"

namespace Marble
{
    class MarbleModel;
    class MarbleMap;
    class GeoDataGroundOverlay;
}

namespace MapAbstraction
{
    class LocalMapLayer : public MapLayerInterface
    {
    Q_OBJECT

    signals:
        void localMapHasContent();
        void localMapVisibilityChanged(bool visible);

    public:
        LocalMapLayer(RoboticsMap *rm);
        ~LocalMapLayer();
        void setLayerContent(const QString &overlayFile, qreal resolution,
                             const Marble::GeoDataCoordinates &center, qreal rotation, QPointF origin);
        LayerType type() const { return LayerLocalMap; }

        QStringList renderPosition() const;
        qreal zValue() const;
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport, const QString &renderPos = "NONE",
                    Marble::GeoSceneLayer *layer = 0);
        void setVisible(bool visible);
        void toggleManualGeolocalization();
        bool hasContent() const;

        void notifyState(); //This is a sign of a bigger problem with connecting slots after constructors

        //Is candidate box a good perspective to see any part of our local map?
        bool isValidPerspective(const Marble::GeoDataLatLonBox &candidate) const;

        Marble::GeoDataCoordinates localToGlobal(const QPointF &localPoint) const;
        Marble::GeoDataCoordinates robotToGlobal(const QPointF &robotLocalPoint,
                                         MapAbstraction::MapRobotObjectConstPtr robotObject) const;
        qreal globalToLocalRotation(qreal globalRotation) const;
        //returns false if global address falls outside of the local map scope
        bool globalToLocal(const Marble::GeoDataCoordinates &globalPoint,
                           QPointF &localPoint) const;
        bool globalToRobot(const Marble::GeoDataCoordinates &globalPoint,
                           MapAbstraction::MapRobotObjectPtr robot, QPointF &robotPoint) const;
        void calculateRelativePosition(GeoObjectID id, MapAbstraction::MapRobotObjectPtr oldRobot,
                                       MapAbstraction::MapRobotObjectPtr newRobot, QPointF &robotLocalPoint,
                                       qreal &orientation);
        bool localizeRobot(GeoObjectID, MapAbstraction::MapRobotObjectPtr robot);
        bool hasReferenceFrame(GeoObjectID id) const;
        void updateRefereceRobotFrame(GeoObjectID id, MapAbstraction::MapRobotObjectPtr robot);
        MapAbstraction::MapRobotObjectPtr getReferenceFrame(GeoObjectID id) const;

        bool handleEvent(QObject *o, QEvent *e);

    private slots:
        void reloadContent() const;

    private:
        //Source file for the image overlay icon
        LocalMapLogic mLocalMapLogic;
        Marble::MarbleModel *mModel;
        Marble::MarbleMap *mMap;

        Marble::GeoDataGroundOverlay *mOverlay;
        Marble::GeoDataDocument mMainLocalMapDocument;
        bool mHasContent = false;
        bool mManualGeolocalization;
    };
    typedef QSharedPointer<LocalMapLayer> LocalMapLayerPtr;
}

#endif // LOCALMAPLAYER_H

#ifndef LOCALMAPLOGIC_H
#define LOCALMAPLOGIC_H

#include <QImage>
#include <QString>
#include <QMouseEvent>
#include <marble/GeoDataLatLonBox.h>
#include <marble/GeoDataLinearRing.h>
#include <marble/MarbleModel.h>
#include <marble/MarbleMap.h>
#include "MapObjectsFwd.h"
#include "GeoObjectID.h"

namespace MapAbstraction
{
    class LocalMapLogic : public QObject
    {
    Q_OBJECT
    signals:
        void requiresUpdate();

    public:
        LocalMapLogic(Marble::MarbleModel *model, Marble::MarbleMap *map);
        QImage currentIcon() const;
        Marble::GeoDataLatLonBox currentBox() const;
        void setOverlay(const QString &filePath, qreal resolution, const Marble::GeoDataCoordinates &center,
                        qreal rotation, QPointF origin);
        Marble::GeoDataLinearRing imageArea() const;
        Marble::GeoDataLinearRing currentBoxBorder() const;
        bool handleMouseEvent(QMouseEvent *event, qreal mouseLon, qreal mouseLat);
        bool handleTouchEvent(QTouchEvent *event, qreal touchLon, qreal touchLat);
        Marble::GeoDataCoordinates geoLocalizePoint(const QPointF &point) const;
        Marble::GeoDataCoordinates robotToGlobal(const QPointF &robotLocalPoint,
                                                 MapAbstraction::MapRobotObjectConstPtr robotObject) const
        ;
        QPointF robotToLocal(const QPointF &point, MapAbstraction::MapRobotObjectConstPtr robot) const;
        qreal geoLocalizeRotation(qreal rotation) const;
        qreal globalToLocalRotation(qreal rotation) const;
        bool globalToLocal(const Marble::GeoDataCoordinates &coords, QPointF &outPoint) const;
        bool globalToRobot(const Marble::GeoDataCoordinates &globalPoint, MapRobotObjectPtr robot,
                           QPointF &robotPoint) const;
        void localToRobot(const QPointF &point, MapAbstraction::MapRobotObjectConstPtr robot,
                          QPointF &robotPoint) const;
        bool localizeRobot(GeoObjectID id, MapAbstraction::MapRobotObjectPtr robot);
        void calculateRelativePosition(GeoObjectID id, MapAbstraction::MapRobotObjectPtr oldRobot,
                                       MapAbstraction::MapRobotObjectPtr newRobot, QPointF &robotLocalPoint,
                                       qreal &orientation);
        bool hasReferenceFrame(GeoObjectID id) const;
        void updateRefereceRobotFrame(GeoObjectID id, MapAbstraction::MapRobotObjectPtr robot);
        MapAbstraction::MapRobotObjectPtr getReferenceFrame(GeoObjectID id) const;

    private:
        bool handleMouse(QPointF geoPos, QPointF pos, QEvent::Type type, Qt::MouseButton button);
        QSizeF localMapGeoScale(const Marble::GeoDataCoordinates &center, bool original = false) const;
        Marble::GeoDataLatLonBox fabricateBox(const Marble::GeoDataCoordinates &center) const;
        QSizeF localMapScale() const;
        QSizeF originalMapScale() const;
        void restoreImageQuality();
        void reduceImageQuality();

        Marble::MarbleModel *mModel;
        Marble::MarbleMap *mMap;

        //Untransformed icon - loaded from file and converted to ARGB
        QImage mOriginalIcon;
        //Icon scaled down for performance reasons - used for placing
        QImage mLowResOriginalIcon;
        //Icon that has all the transformations applied, ready for display
        QImage mIcon;

        //A box that is suitable for pre-transform icon
        Marble::GeoDataLatLonBox mOriginalBox;
        //A box that encompasses the icon
        Marble::GeoDataLatLonBox mBox;

        QPointF mOrigin;
        qreal mRotation;
        qreal mMapResolution;
        qreal mCurrentScaleFactor;

        typedef QMap<GeoObjectID, MapAbstraction::MapRobotObjectPtr> RobotReferenceFrames;
        RobotReferenceFrames mRobotReferenceFrames;
    };
}

#endif // LOCALMAPLOGIC_H

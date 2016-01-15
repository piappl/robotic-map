#ifndef LOCALMAPLOGIC_H
#define LOCALMAPLOGIC_H

#include <QImage>
#include <QString>
#include <QMouseEvent>
#include <marble/GeoDataLatLonBox.h>
#include <marble/GeoDataLinearRing.h>
#include "MapObjectsFwd.h"

namespace Marble
{
    class MarbleModel;
    class MarbleMap;

    class LocalMapLogic : public QObject
    {
    Q_OBJECT
    signals:
        void requiresUpdate();

    public:
        LocalMapLogic(MarbleModel *model, MarbleMap *map);
        QImage currentIcon() const;
        GeoDataLatLonBox currentBox() const;
        void setOverlay(const QString &filePath, qreal resolution, const GeoDataCoordinates &center,
                        qreal rotation, QPointF origin);
        GeoDataLinearRing imageArea() const;
        GeoDataLinearRing currentBoxBorder() const;
        bool handleMouseEvent(QMouseEvent *event, qreal mouseLon, qreal mouseLat);
        GeoDataCoordinates geoLocalizePoint(const QPointF &point) const;
        QPointF robotToLocal(const QPointF &point, MapAbstraction::MapRobotObjectConstPtr robot) const;
        qreal geoLocalizeRotation(qreal rotation) const;
        bool globalToLocal(const GeoDataCoordinates &coords, QPointF &outPoint) const;
        void localToRobot(const QPointF &point, MapAbstraction::MapRobotObjectConstPtr robot,
                          QPointF &robotPoint) const;

    private:
        QSizeF localMapGeoScale(const GeoDataCoordinates &center, bool original = false) const;
        GeoDataLatLonBox fabricateBox(const GeoDataCoordinates &center) const;
        QSizeF localMapScale() const;
        QSizeF originalMapScale() const;
        void restoreImageQuality();
        void reduceImageQuality();

        MarbleModel *mModel;
        MarbleMap *mMap;

        //Untransformed icon - loaded from file and converted to ARGB
        QImage mOriginalIcon;
        //Icon scaled down for performance reasons - used for placing
        QImage mLowResOriginalIcon;
        //Icon that has all the transformations applied, ready for display
        QImage mIcon;

        //A box that is suitable for pre-transform icon
        GeoDataLatLonBox mOriginalBox;
        //A box that encompasses the icon
        GeoDataLatLonBox mBox;

        QPointF mOrigin;
        qreal mRotation;
        qreal mMapResolution;
        qreal mCurrentScaleFactor;
    };

}

#endif // LOCALMAPLOGIC_H

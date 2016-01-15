#include <QtMath>
#include <QImage>
#include <QFileInfo>

#include <marble/ViewportParams.h>
#include <marble/GeoPainter.h>
#include <marble/MarbleModel.h>
#include <marble/MarbleMap.h>
#include <marble/GeoDataGroundOverlay.h>
#include <marble/GeoDataTreeModel.h>

#include "LocalMapLayer.h"
#include "LocalMapLogic.h"
#include "MapRobotObject.h"
#include "MapLibraryHelpers.h"

namespace
{
    qreal diameter(const Marble::GeoDataLatLonBox &box)
    {
        return qMax(box.width(Marble::GeoDataCoordinates::Degree),
                    box.height(Marble::GeoDataCoordinates::Degree));
    }
}

namespace Marble
{
    LocalMapLayer::LocalMapLayer(MarbleModel *model, MarbleMap *map)
        : mLocalMapLogic(new LocalMapLogic(model, map)),
          mModel(model),
          mMap(map),
          mVisible(false),
          mHasContent(false),
          mManualGeolocalization(false)
    {
        mOverlay = new GeoDataGroundOverlay();
        mMainLocalMapDocument.append(mOverlay);
        mModel->treeModel()->addDocument(&mMainLocalMapDocument);

        connect(mLocalMapLogic.data(), SIGNAL(requiresUpdate()),
                this, SLOT(reloadContent()));
    }

    LocalMapLayer::~LocalMapLayer()
    {
        mModel->treeModel()->removeDocument(&mMainLocalMapDocument);
    }

    bool LocalMapLayer::isValidPerspective(const GeoDataLatLonBox &candidate) const
    {
        if (!mVisible || !mHasContent)
        {   //any perspective is valid if no content is showing
            return true;
        }

        GeoDataLatLonBox box = mLocalMapLogic->currentBox();
        if (!candidate.contains(box) && !candidate.intersects(box))
        {   //No relation - invalid perspective
            return false;
        }

        //Acceptable scales
        qreal d = diameter(box);
        qreal zoomOutLimit = d * 4;
        qreal zoomInLimit = d * 0.25;
        qreal dCandidate = diameter(candidate);
        if (dCandidate > zoomOutLimit || dCandidate < zoomInLimit)
        {   //Box is too big or too small
            return false;
        }

        return true;
    }

    void LocalMapLayer::setLayerContent(const QString &overlayFile, qreal resolution,
                                        const GeoDataCoordinates &center, qreal rotation, QPointF origin)
    {
        QFileInfo info(overlayFile);
        if (!info.exists() || !info.isFile() || !info.isReadable())
        {
            qWarning("LocalMapLayer received file path that does not match a readable file!");
            return;
        }

        mLocalMapLogic->setOverlay(overlayFile, resolution, center, rotation, origin);
        mHasContent = true;
        reloadContent();
        emit localMapHasContent();
    }

    void LocalMapLayer::reloadContent() const
    {
        if (mVisible)
        {
            mOverlay->setLatLonBox(mLocalMapLogic->currentBox());
            mOverlay->setIcon(mLocalMapLogic->currentIcon());
        }

        mOverlay->setVisible(mVisible);
        mModel->treeModel()->updateFeature(mOverlay);
    }

    QStringList LocalMapLayer::renderPosition() const
    {
        return QStringList() << "SURFACE";
    }

    qreal LocalMapLayer::zValue() const
    {
        return qreal(1); //Some arbitrary value > 0
    }

    bool LocalMapLayer::render(GeoPainter *painter, ViewportParams *viewport, const QString &, GeoSceneLayer *)
    {
        if (!mVisible | !mHasContent)
            return true;

        Q_UNUSED(viewport);

        //Renders shapes useful for debugging
        GeoDataLinearRing ring = mLocalMapLogic->imageArea();
        painter->drawPolygon(ring);

        if (mManualGeolocalization)
        {
            painter->setPen(Qt::red);
            painter->drawPolygon(mLocalMapLogic->currentBoxBorder());
        }

        return true;
    }

    void LocalMapLayer::setVisible(bool visible)
    {
        mVisible = visible;
        reloadContent();
        emit localMapVisibilityChanged(visible);
    }

    bool LocalMapLayer::visible() const
    {
        return mVisible;
    }

    bool LocalMapLayer::hasContent() const
    {
        return mHasContent;
    }

    void LocalMapLayer::notifyState()
    {
        if (mHasContent)
            emit localMapHasContent();
        emit localMapVisibilityChanged(mVisible);
    }

    bool LocalMapLayer::handleEvent(QObject *, QEvent *e)
    {
        if (!mVisible || !mHasContent || !mManualGeolocalization)
            return false;

        if (e->type() == QEvent::MouseButtonPress
            || e->type() == QEvent::MouseMove
            || e->type() == QEvent::MouseButtonRelease)
        {
            QMouseEvent *event = static_cast<QMouseEvent*>(e);
            qreal mouseLon, mouseLat;
            bool isAboveMap = mMap->geoCoordinates(event->x(), event->y(),
                                                   mouseLon, mouseLat, GeoDataCoordinates::Radian);
            if (!isAboveMap)
            {
                return false;
            }

            return mLocalMapLogic->handleMouseEvent(event, mouseLon, mouseLat);
        }
        return false;
    }

    qreal LocalMapLayer::localToGlobalRotation(qreal localRotation) const
    {
        return mLocalMapLogic->geoLocalizeRotation(localRotation);
    }

    GeoDataCoordinates LocalMapLayer::localToGlobal(const QPointF &localPoint) const
    {
        return mLocalMapLogic->geoLocalizePoint(localPoint);
    }

    GeoDataCoordinates LocalMapLayer::robotToGlobal(const QPointF &robotLocalPoint, MapAbstraction::MapRobotObjectConstPtr robotObject) const
    {
        QPointF localCoords = mLocalMapLogic->robotToLocal(robotLocalPoint, robotObject);
        //qDebug("local Coords %f %f", localCoords.x(), localCoords.y());
        return mLocalMapLogic->geoLocalizePoint(localCoords);
    }

    bool LocalMapLayer::globalToLocal(const GeoDataCoordinates &globalPoint,
                                      QPointF &localPoint) const
    {
        return mLocalMapLogic->globalToLocal(globalPoint, localPoint);
    }

    bool LocalMapLayer::globalToRobot(const GeoDataCoordinates &globalPoint,
                                      MapAbstraction::MapRobotObjectPtr robot, QPointF &robotPoint) const
    {
        QPointF localPoint;
        bool inMap = mLocalMapLogic->globalToLocal(globalPoint, localPoint);
        mLocalMapLogic->localToRobot(localPoint, robot, robotPoint);
        return inMap;
    }

    bool LocalMapLayer::hasReferenceFrame(GeoObjectID id) const
    {
        return mRobotReferenceFrames.contains(id);
    }

    void LocalMapLayer::updateRefereceRobotFrame(GeoObjectID id, MapAbstraction::MapRobotObjectPtr robot)
    {
        MapAbstraction::MapObjectPtr frozen(robot->Clone());
        QPointF local;
        globalToLocal(MapLibraryHelpers::transformCoords(robot->coords()), local);
        frozen->setCoords(MapAbstraction::GeoCoords(local.x(), local.y()));
        mRobotReferenceFrames.insert(id, frozen.staticCast<MapAbstraction::MapRobotObject>());
    }

    MapAbstraction::MapRobotObjectPtr LocalMapLayer::getReferenceFrame(GeoObjectID id) const
    {
        Q_ASSERT(hasReferenceFrame(id));
        MapAbstraction::MapObjectPtr defrosted(mRobotReferenceFrames.value(id)->Clone());
        MapAbstraction::MapRobotObjectPtr robot = defrosted.staticCast<MapAbstraction::MapRobotObject>();

        QPointF local(robot->coords().longitude(), robot->coords().latitude());
        robot->setCoords(MapLibraryHelpers::transformCoords(localToGlobal(local)));

        return robot;
    }

    void LocalMapLayer::toggleManualGeolocalization()
    {
        mManualGeolocalization = !mManualGeolocalization;
    }

}


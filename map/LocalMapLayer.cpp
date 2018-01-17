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
#include "RoboticsMap.h"

using namespace MapAbstraction;
using namespace Marble;

namespace
{
    qreal diameter(const Marble::GeoDataLatLonBox &box)
    {
        return qMax(box.width(Marble::GeoDataCoordinates::Degree),
                    box.height(Marble::GeoDataCoordinates::Degree));
    }
}

LocalMapLayer::LocalMapLayer(RoboticsMap *rm)
    : MapLayerInterface(rm),
      mLocalMapLogic(rm->model(), rm->map()),
      mModel(rm->model()),
      mMap(rm->map()),
      mManualGeolocalization(false)
{
    mOverlay = new GeoDataGroundOverlay();
    mMainLocalMapDocument.append(mOverlay);
    mModel->treeModel()->addDocument(&mMainLocalMapDocument);

    QObject::connect(&mLocalMapLogic, SIGNAL(requiresUpdate()), this, SLOT(reloadContent()));
}

LocalMapLayer::~LocalMapLayer()
{
    mModel->treeModel()->removeDocument(&mMainLocalMapDocument);
}

bool LocalMapLayer::isValidPerspective(const GeoDataLatLonBox &candidate) const
{
    if (!visible() || !mHasContent)
    {   //any perspective is valid if no content is showing
        return true;
    }

    GeoDataLatLonBox box = mLocalMapLogic.currentBox();
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

    mLocalMapLogic.setOverlay(overlayFile, resolution, center, rotation, origin);
    mHasContent = true;
    reloadContent();
    qWarning("Has content \n");
    emit localMapHasContent();
    emit requestUpdate();
    emit localMapVisibilityChanged(visible());
}

void LocalMapLayer::reloadContent() const
{
    if (visible())
    {
        mOverlay->setLatLonBox(mLocalMapLogic.currentBox());
        mOverlay->setIcon(mLocalMapLogic.currentIcon());
    }

    mOverlay->setVisible(visible());
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
    if (!visible() | !mHasContent)
        return true;

    Q_UNUSED(viewport);

    //Renders shapes useful for debugging
    GeoDataLinearRing ring = mLocalMapLogic.imageArea();
    painter->drawPolygon(ring);

    if (mManualGeolocalization)
    {
        painter->setPen(Qt::red);
        painter->drawPolygon(mLocalMapLogic.currentBoxBorder());
    }

    return true;
}

void LocalMapLayer::setVisible(bool visible)
{
    MapLayerInterface::setVisible(visible);
    reloadContent();
    qWarning("Vis change to %s\n", visible ? "YES" : "NO");
    emit localMapVisibilityChanged(visible);
}

bool LocalMapLayer::hasContent() const
{
    return mHasContent;
}

void LocalMapLayer::notifyState()
{
    if (mHasContent)
        emit localMapHasContent();
    emit localMapVisibilityChanged(visible());
}

bool LocalMapLayer::handleEvent(QObject *, QEvent *e)
{
    if (!visible() || !mHasContent || !mManualGeolocalization)
        return false;

    if (e->type() == QEvent::TouchBegin
        || e->type() == QEvent::TouchUpdate
        || e->type() == QEvent::TouchEnd)
    {
        QTouchEvent *event = static_cast<QTouchEvent*>(e);
        if (event->touchPoints().isEmpty())
            return false;
        QTouchEvent::TouchPoint p = event->touchPoints().first();

        //Single point supported only
        qreal touchLon, touchLat;
        bool aboveMap = mMap->geoCoordinates(p.pos().x(), p.pos().y(), touchLon, touchLat,
                                             GeoDataCoordinates::Radian);
        if (!aboveMap)
            return false;
        return mLocalMapLogic.handleTouchEvent(event, touchLon, touchLat);
    }

    if (e->type() == QEvent::MouseButtonPress
        || e->type() == QEvent::MouseMove
        || e->type() == QEvent::MouseButtonRelease)
    {
        QMouseEvent *event = static_cast<QMouseEvent*>(e);
        qreal mouseLon, mouseLat;
        bool aboveMap = mMap->geoCoordinates(event->x(), event->y(), mouseLon, mouseLat,
                                             GeoDataCoordinates::Radian);
        if (!aboveMap)
            return false;
        return mLocalMapLogic.handleMouseEvent(event, mouseLon, mouseLat);
    }
    return false;
}

qreal LocalMapLayer::globalToLocalRotation(qreal globalRotation) const
{
    return mLocalMapLogic.globalToLocalRotation(globalRotation);
}

GeoDataCoordinates LocalMapLayer::localToGlobal(const QPointF &localPoint) const
{
    return mLocalMapLogic.geoLocalizePoint(localPoint);
}

GeoDataCoordinates LocalMapLayer::robotToGlobal(const QPointF &robotLocalPoint, MapRobotObjectConstPtr robotObject) const
{
    return mLocalMapLogic.robotToGlobal(robotLocalPoint, robotObject);
}

bool LocalMapLayer::globalToLocal(const GeoDataCoordinates &globalPoint,
                                  QPointF &localPoint) const
{
    return mLocalMapLogic.globalToLocal(globalPoint, localPoint);
}

bool LocalMapLayer::globalToRobot(const GeoDataCoordinates &globalPoint,
                                  MapRobotObjectPtr robot, QPointF &robotPoint) const
{
    return mLocalMapLogic.globalToRobot(globalPoint, robot, robotPoint);
}

bool LocalMapLayer::localizeRobot(GeoObjectID id, MapRobotObjectPtr robot)
{
    if (!hasContent())
        return false;

    return mLocalMapLogic.localizeRobot(id, robot);
}

void LocalMapLayer::updateRefereceRobotFrame(GeoObjectID id, MapRobotObjectPtr robot)
{
    mLocalMapLogic.updateRefereceRobotFrame(id, robot);
}

void LocalMapLayer::toggleManualGeolocalization()
{
    mManualGeolocalization = !mManualGeolocalization;
    requestUpdate();
}

void LocalMapLayer::calculateRelativePosition(GeoObjectID id, MapRobotObjectPtr oldRobot,
                               MapRobotObjectPtr newRobot, QPointF &robotLocalPoint, qreal &orientation)
{
    mLocalMapLogic.calculateRelativePosition(id, oldRobot, newRobot, robotLocalPoint, orientation);
}


#include "GuiController.h"
#include "MapRobotObject.h"
#include "MapOrderedObject.h"
#include "MapIconProvider.h"
#include "PlacemarkType.h"

using namespace MapAbstraction;

namespace
{
    QString nameOfType(PlacemarkType type)
    {
        QString qmlCat;
        switch (type)
        {
            case PlacemarkPlace:
                qmlCat = "place";
                break;
            case PlacemarkWaypoint:
                qmlCat = "path";
                break;
            case PlacemarkPolygonNode:
                qmlCat = "polygon";
                break;
            default:
                qmlCat = "place";
                break;
        }
        return qmlCat;
    }
}

GuiController::GuiController(QQuickItem *parent)
    : QQuickItem(parent)
{
}

void GuiController::updatePlacemarkGuiInfo(GeoObjectID placemarkID,
                                           MapObjectConstPtr placemarkObject,
                                           QString iconPath)
{
    if (!mGuiObjects.contains(placemarkID))
        mGuiObjects.insert(placemarkID, placemarkObject);

    QString positionString = placemarkObject->coords().valid() ?
                placemarkObject->coords().positionString() : "Unknown position";
    if (placemarkObject->category() == PlacemarkRobot)
    {
        MapRobotObjectConstPtr robot = placemarkObject.staticCast<const MapRobotObject>();
        if (robot->visible())
        {
            //qDebug("Icon path %s", qPrintable(iconPath));
            iconPath.prepend(iconPath.startsWith(':') ? "qrc" : "file:/");
            emit robotUpdate(placemarkID.intId(),
                             robot->robotID(),
                             robot->name(),
                             robot->type(),
                             robot->consoleID(),
                             robot->state() == RobotStateConnected ? "connected" :
                                               (robot->state() == RobotStateNormal ? "normal"
                                                                                   : "disconnected"),
                             positionString,
                             iconPath);
        }
        else
            emit robotRemove(placemarkID.intId());
    }
    else
    {
        if (placemarkObject->visible())
        {
            QString qmlCat = nameOfType(placemarkObject->category());
            int order = -1;
            if (placemarkObject->category() == PlacemarkWaypoint
                    || placemarkObject->category() == PlacemarkPolygonNode)
            {
                MapOrderedObjectConstPtr ordered = placemarkObject.staticCast<const MapOrderedObject>();
                order = ordered->number();
            }

            emit placeUpdate(placemarkID.intId(), positionString, order, qmlCat);
        }
        else
        {
            emit placeRemove(placemarkID.intId());
        }
    }
}

void GuiController::placemarkRemoved(GeoObjectID placemarkID)
{
    if (!mGuiObjects.contains(placemarkID))
        return;

    PlacemarkType type = mGuiObjects.value(placemarkID)->category();
    mGuiObjects.remove(placemarkID);
    if (type == PlacemarkRobot)
    {
        emit robotRemove(placemarkID.intId());
    }
    else
    {
        emit placeRemove(placemarkID.intId());
    }
}

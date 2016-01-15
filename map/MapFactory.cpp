#include <QQuickItem>
#include <QQuickView>
#include <QQmlEngine>

#include "MapFactory.h"
#include "GuiController.h"
#include "MapEditor.h"
#include "RobotEditor.h"
#include "RoboticsMap.h"

using namespace MapAbstraction;

IGeoMapPtr MapFactory::createMapItem(QQuickItem *viewMapItem)
{
    qmlRegisterType<RoboticsMap>("RoboticsMap", 1, 0, "RoboticsMap");
    qmlRegisterType<GuiController>("MapController", 1, 0, "MapController");
    qmlRegisterType<MapEditor>("MapEditor", 1, 0, "MapEditor");
    qmlRegisterType<RobotEditor>("RobotEditor", 1, 0, "RobotEditor");
    QUrl source("qrc:/mapwidget/MapItem.qml");

    QQmlComponent component(qmlEngine(viewMapItem), source);
    QObject *componentObject = component.create();
    Q_ASSERT(componentObject);

    RoboticsMap *roboMap = qobject_cast<RoboticsMap*>(componentObject);
    Q_ASSERT(roboMap);

    roboMap->setParentItem(viewMapItem);
    roboMap->setFocus(roboMap->isEnabled());
    return IGeoMapPtr(roboMap);
}


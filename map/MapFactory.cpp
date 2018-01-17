#include <QQuickItem>
#include <QQuickView>
#include <QQmlEngine>

#include "MapFactory.h"
#include "GuiController.h"
#include "MapEditor.h"
#include "RobotEditor.h"
#include "RoboticsMap.h"
#include "SensorLayerControlGui.h"

using namespace MapAbstraction;

IGeoMapPtr MapFactory::createMapItem(QQuickItem *viewMapItem)
{
    qmlRegisterType<RoboticsMap>("RoboticsMap", 1, 0, "RoboticsMap");
    qmlRegisterType<GuiController>("MapController", 1, 0, "MapController");
    qmlRegisterType<SensorLayerControlGui>("SensorControl", 1, 0, "SensorControl");
    qmlRegisterType<MapEditor>("MapEditor", 1, 0, "MapEditor");
    qmlRegisterType<RobotEditor>("RobotEditor", 1, 0, "RobotEditor");
    QUrl source("qrc:/mapwidget/MapItem.qml");

    QQmlComponent component(qmlEngine(viewMapItem), source);
    if (component.status() != QQmlComponent::Ready)
    {
        if (component.status() == QQmlComponent::Error)
            qWarning("Error: %s", qPrintable(component.errorString()));
        exit(1); // or maybe throw
    }
    QObject *componentObject = component.create();
    Q_ASSERT(componentObject);

    RoboticsMap *roboMap = qobject_cast<RoboticsMap*>(componentObject);
    Q_ASSERT(roboMap);

    roboMap->setParentItem(viewMapItem);
    roboMap->setFocus(roboMap->isEnabled());
    return IGeoMapPtr(roboMap);
}


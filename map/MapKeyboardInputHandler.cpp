#include <QKeyEvent>
#include "GeoObjectsManager.h"
#include "MapKeyboardInputHandler.h"

using namespace MapAbstraction;

MapKeyboardInputHandler::MapKeyboardInputHandler(GeoObjectsManagerPtr geoManager) : mGeoObjectsManager(geoManager)
{
}

bool MapKeyboardInputHandler::eventFilter(QObject *obj, QEvent *event)
{
    Q_UNUSED(obj);
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        int key = keyEvent->key();

        //Makes a somewhat impure assumption that keys 1-9 constants follow each other
        if (key >= Qt::Key_1 && key <= Qt::Key_9)
        {
            int placemarkIndex = key - Qt::Key_1;
            emit itemSelected(placemarkIndex);
        }
        else if (key == Qt::Key_H)
        {
            emit toggleVisibility();
        }
        else if (key == Qt::Key_P)
        {
            emit togglePaths();
        }
    }
    return false;
}

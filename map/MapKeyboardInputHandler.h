#ifndef MAPKEYBOARDINPUTHANDLER_H
#define MAPKEYBOARDINPUTHANDLER_H

#include <QSharedPointer>
#include "InternalTypesFwd.h"

//Handles only keyboard events so far. It is important to note that layers can handle and consume
//events on their own.
class MapKeyboardInputHandler : public QObject
{
    Q_OBJECT
    public:
        MapKeyboardInputHandler(MapAbstraction::GeoObjectsManagerPtr geoManager);
        virtual bool eventFilter(QObject *obj, QEvent *event);

    signals:
        void itemSelected(int index);
        void toggleVisibility();
        void togglePaths();
        void hidePopups();

    private:
        MapAbstraction::GeoObjectsManagerPtr mGeoObjectsManager;
};

#endif // MAPKEYBOARDINPUTHANDLER_H

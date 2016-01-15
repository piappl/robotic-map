#ifndef GUICONTROLLER_H
#define GUICONTROLLER_H

#include <QQuickItem>
#include <QMap>
#include "MapObjectsFwd.h"
#include "GeoObjectID.h"

class GuiController : public QQuickItem
{
    Q_OBJECT
public:
    GuiController(QQuickItem *parent = 0);

public slots:
    void updatePlacemarkGuiInfo(GeoObjectID placemarkID, MapAbstraction::MapObjectConstPtr object,
                                QString iconPath);
    void placemarkRemoved(GeoObjectID placemarkID);

signals:
    void robotUpdate(int aPlacemarkID, int aRobotID, QString aName, QString aType,
                       int aConsoleID, QString aStateString, QString aPos, QString aIconPath);
    void placeUpdate(int aPlacemarkID, QString aPos, int aNumber, QString aType);
    void robotRemove(int aPlacemarkID);
    void placeRemove(int aPlacemarkID);

    void itemSelected(int aItemIndex); //0-9
    void robotsVisibilityChanged(bool aVisible, bool aSimplified);

    void cameraChangePreventedWhenTracking();
    void licenseChanged(QString aLicense);
    void centerPositionChanged(QString aPosition);

private:
    QMap<GeoObjectID, MapAbstraction::MapObjectConstPtr> mGuiObjects;
};

#endif //GUICONTROLLER_H

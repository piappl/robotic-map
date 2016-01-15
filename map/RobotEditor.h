#ifndef ROBOTEDITOR_H
#define ROBOTEDITOR_H

#include <QQuickItem>
#include "PlacemarkType.h"

class RobotEditor : public QQuickItem
{
    Q_OBJECT
public:
    RobotEditor(QQuickItem *parent = 0);

signals:
    void finalizePositionEdit(bool persist);
    void startPositionEdit();
    void placeAtCrosshair();
    void orientateToCrosshair();
    void orientationEdit(bool enable);

public slots:
    void orientate();
    void position();
    void finalizeEdit(bool persist);
    void startEdit();
    void orientationEditChanged(bool enable);
};

#endif // ROBOTEDITOR_H

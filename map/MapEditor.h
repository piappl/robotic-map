#ifndef MAPEDITOR_H
#define MAPEDITOR_H

#include <QQuickItem>
#include "PlacemarkType.h"

//TODO - refactor
class MapEditor : public QQuickItem
{
    Q_OBJECT
public:
    enum EditMode
    {
        None,
        Path,
        Place,
        Polygon
    };

    MapEditor(QQuickItem *parent = 0);
signals:
    void addAtCenter(bool orderBeforeSelected);
    void removeAtCenter();
    void removeAll();
    void editModeChanged(PlacemarkType mode, bool turnedOn);
    void selectionModeChanged(bool aSelectionMode);
    void placemarkSelected(int placemarkID);
    void finalizeMapEdit(bool accept);

public slots:
    void finalizeEdit(bool accept);
    void setEditMode(const QString &mode);
    void addAtCenterSelected(bool orderBeforeSelected);
    void removeAtCenterSelected();
    void removeAllSelected();
    void placemarkClickedOnList(int placemarkID);

private:
    PlacemarkType pointType() const;
    EditMode mEditMode;
};

#endif // MAPEDITOR_H

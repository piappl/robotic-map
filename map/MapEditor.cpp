#include "MapEditor.h"

MapEditor::MapEditor(QQuickItem *parent) : QQuickItem(parent), mEditMode(None)
{
}

void MapEditor::setEditMode(const QString &mode)
{
    EditMode old = mEditMode;
    PlacemarkType oldPoint = pointType();

    if (mode == "place")
        mEditMode = Place;
    else if (mode == "path")
        mEditMode = Path;
    else if (mode == "polygon")
        mEditMode = Polygon;
    else
        mEditMode = None;

    if (old != mEditMode)
    {
        emit editModeChanged(oldPoint, false);
        emit editModeChanged(pointType(), true);
    }
}

void MapEditor::addAtCenterSelected(bool orderBeforeSelected)
{
    emit addAtCenter(orderBeforeSelected);
}

void MapEditor::removeAtCenterSelected()
{
    emit removeAtCenter();
}

void MapEditor::removeAllSelected()
{
    emit removeAll();
}

void MapEditor::placemarkClickedOnList(int placemarkID)
{
    emit placemarkSelected(placemarkID);
}

void MapEditor::finalizeEdit(bool persist)
{
    emit finalizeMapEdit(persist);
}

PlacemarkType MapEditor::pointType() const
{
    switch (mEditMode)
    {
        case Place:
            return PlacemarkPlace;
        case Path:
            return PlacemarkWaypoint;
        case Polygon:
            return PlacemarkPolygonNode;
        case None:
        default:
            return PlacemarkNone;
    }
}

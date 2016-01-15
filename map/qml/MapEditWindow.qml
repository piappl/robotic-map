import QtQuick 2.2
import MapEditor 1.0
import "."

Rectangle
{
    id: mapEditWindow
    width: 220
    height: 120
    visible: false
    color: "transparent"
    property string mode: "none"
    property bool showPlacesList: true
    property bool finalized: false

    MapEditor
    {
        id: mapEditor
        objectName: "mapEditor"
        Connections
        {
            target: mapEditWindow
            onModeChanged:
            {
                mapEditor.setEditMode(mapEditWindow.mode)
            }
        }
    }

    onVisibleChanged:
    {
        mapItem.displayCrosshair(visible || robotEditWindow.visible)
        if (!visible)
        {
            mapEditor.setEditMode("none");
            mapEditor.finalizeEdit(mapEditWindow.finalized)
        }
        else
        {
            mapEditor.setEditMode(mapEditWindow.mode)
            mapEditWindow.finalized = false
        }
    }

    Rectangle
    {
        id: styleRect
        anchors.fill: parent
        color: MapStyle.colors.background
        border.color: MapStyle.colors.border
        opacity: 0.4
        radius: 12
    }

    GpsPanel
    {
        id: gps
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.leftMargin: MapStyle.margins.leftMargin
        anchors.topMargin: MapStyle.margins.topMargin
        font.pointSize: 12
    }

    Rectangle
    {
        id: editTypeRect
        anchors.top: gps.bottom
        anchors.left: parent.left
        width: childrenRect.width
        height: childrenRect.height
        color: "transparent"

        MapButton
        {
            id: addPlaceButton
            anchors.left: parent.left
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/addPlace.png"
            colorBackground:
            {
                mapEditWindow.mode === "place" ? MapStyle.colors.border : MapStyle.colors.background
            }
            onClicked:
            {
                mapEditWindow.mode = "place"
            }
        }

        MapButton
        {
            id: addPathButton
            anchors.left: addPlaceButton.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/addPath.png"
            colorBackground:
            {
                mapEditWindow.mode === "path" ? MapStyle.colors.border : MapStyle.colors.background
            }
            onClicked:
            {
                mapEditWindow.mode = "path"
            }
        }

        MapButton
        {
            id: addPolygon
            anchors.left: addPathButton.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/addPolygon.png"
            colorBackground:
            {
                mapEditWindow.mode === "polygon" ? MapStyle.colors.border : MapStyle.colors.background
            }
            onClicked:
            {
                mapEditWindow.mode = "polygon"
            }
        }

        MapButton
        {
            id: showPlaces
            anchors.left: addPolygon.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/placesList.png"
            colorBackground:
            {
                mapEditWindow.showPlacesList ? MapStyle.colors.border : MapStyle.colors.background
            }
            onClicked:
            {
                mapEditWindow.showPlacesList = !mapEditWindow.showPlacesList
            }
        }
    }

    Rectangle
    {
        id: editActionsRect
        anchors.top: editTypeRect.bottom
        anchors.left: parent.left
        width: childrenRect.width
        height: { visible ? childrenRect.height : 0 }
        color: "transparent"
        visible: { mapEditWindow.mode !== "none" }

        MapButton
        {
            id: placeRemoveAll
            anchors.left: parent.left
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/removeAllPlacemarks.png"
            onClicked:
            {
                mapEditor.removeAll()
            }
        }

        MapButton
        {
            id: placeRemove
            anchors.left: placeRemoveAll.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/removePlacemark.png"
            onClicked:
            {
                mapEditor.removeAtCenter()
            }
        }

        MapButton
        {
            id: placeAdd
            anchors.left: placeRemove.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/addPlacemark.png"
            onClicked:
            {
                mapEditor.addAtCenter(false)
            }
        }

        MapButton
        {
            id: placeAddBefore
            anchors.left: placeAdd.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/addPlacemarkBefore.png"
            property bool selectionMode: false
            visible:
            {
                selectionMode && (mapEditWindow.mode === "polygon"
                                    || mapEditWindow.mode === "path")
            }
            onClicked:
            {
                mapEditor.addAtCenter(true)
            }

            Connections
            {
                target: mapEditor
                onSelectionModeChanged:
                {
                    placeAddBefore.selectionMode = aSelectionMode
                }
            }
        }

        MapButton
        {
            id: acceptEdits
            anchors.left: placeAddBefore.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/accept.png"
            onClicked:
            {
                finalized = true
                mapEditWindow.visible = false
            }
        }

    }

    PlacesList
    {
        id: placesList
        anchors.left: parent.right
        anchors.top: parent.top
        filter: mapEditWindow.mode
    }
}


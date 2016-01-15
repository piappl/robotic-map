import QtQuick 2.2
import RobotEditor 1.0
import "."

//TODO - this is copied from MapEditWindow and should be refactored
Rectangle
{
    id: robotEditWindow
    width: 180
    height: 60
    visible: false
    color: "transparent"
    property bool finalized: false

    RobotEditor
    {
        id: robotEditor
        objectName: "robotEditor"
    }

    onVisibleChanged:
    {
        mapItem.displayCrosshair(visible || mapEditWindow.visible)
        if (!visible)
        {
            robotEditor.orientationEditChanged(false)
            robotOrientateButton.toggled = false
            robotEditor.finalizeEdit(robotEditWindow.finalized)
        }
        else
        {
            robotEditor.startEdit()
            robotEditWindow.finalized = false
            acceptPosition.visible = false
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

    Rectangle
    {
        id: robotEditButtonsRect
        anchors.top: parent.top
        anchors.left: parent.left
        width: childrenRect.width
        height: childrenRect.height
        color: "transparent"

        MapButton
        {
            id: robotLocalizeButton
            anchors.left: parent.left
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/location.png"
            onClicked:
            {
                acceptPosition.visible = true
                robotEditor.position();
            }
        }

        MapButton
        {
            id: robotOrientateButton
            anchors.left: robotLocalizeButton.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/orientation.png"
            property bool toggled: false

            colorBackground:
            {
                toggled ? MapStyle.colors.border : MapStyle.colors.background
            }

            onClicked:
            {
                acceptPosition.visible = true
                robotEditor.orientate()
            }

            onPressedShort:
            {
                toggled = !toggled
                robotEditor.orientationEditChanged(toggled)
            }
        }

        MapButton
        {
            id: acceptPosition
            anchors.left: robotOrientateButton.right
            anchors.top: parent.top
            marginLeft: MapStyle.margins.smallLeftMargin
            visible: false
            icon: "qrc:/mapwidget/icons/accept.png"
            onClicked:
            {
                finalized = true
                robotEditWindow.visible = false
            }
        }
    }
}



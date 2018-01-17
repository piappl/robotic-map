import QtQuick 2.2
import RoboticsMap 1.0
import MapController 1.0
import "."

RoboticsMap
{   //TODO - make QtCreator recognize this type
    id: mapItem
    anchors.fill: parent
    onEnabledChanged: focus = enabled
    property int followedPlacemarkIndex: -1
    property string state:
    {
        if (parent)
            parent.state
        else
            "hide"
    }

    function followPlacemark(aIndex)
    {
        buttonOverview.turn(false)
        followedPlacemarkIndex = aIndex;
    }

    function followOverview()
    {
        followedPlacemarkIndex = -1;
        buttonOverview.turn(true);
    }

    function stopCameraFollowing()
    {
        followedPlacemarkIndex = -1;
        buttonOverview.turn(false);
        mapItem.stopCameraFollow()
    }

    MapController
    {
        id: controller
        objectName: "guiController"
    }

    Rectangle
    {
        id: mapControls
        anchors.fill: parent
        color: "transparent"
        visible: { mapItem.state !== "showMinimap" && mapItem.state !== "hide" }

        PinchArea
        {
            objectName: "pinchArea"
            anchors.fill: parent
            enabled: true
            onPinchStarted: { mapItem.handlePinchStart(pinch.center) }
            onPinchUpdated: { mapItem.handlePinchUpdate(pinch.center, pinch.scale) }
            onPinchFinished:{ mapItem.handlePinchEnd(pinch.center, false) }
        }

        MapButton
        {
            id: buttonZoomIn
            objectName: "buttonZoomIn"
            anchors.top: parent.top
            marginTop: MapStyle.margins.firstButtonMargin
            icon: "qrc:/mapwidget/icons/zoomIn.png"
            onClicked: { mapItem.zoomIn() }
        }

        MapButton
        {
            id: buttonZoomOut
            objectName: "buttonZoomOut"
            anchors.top: buttonZoomIn.bottom
            icon: "qrc:/mapwidget/icons/zoomOut.png"
            onClicked: { mapItem.zoomOut() }
        }

        MapButton
        {
            id: buttonRobotsVisible
            objectName: "buttonRobotsVisible"
            anchors.top: buttonZoomOut.bottom
            property string displayMode: "normal"
            icon:
            {
                if (displayMode === "normal")
                    "qrc:/mapwidget/icons/robotsVisible64.png"
                else if (displayMode === "simplified")
                    "qrc:/mapwidget/icons/robotsPointVisible64.png"
                else //"none"
                    "qrc:/mapwidget/icons/robotsNotVisible64.png"
            }
            onClicked: { mapItem.toggleVisibility() }
            Connections
            {
                target: controller
                onRobotsVisibilityChanged:
                {
                    if (aVisible)
                    {
                        if (aSimplified)
                            buttonRobotsVisible.displayMode = "simplified"
                        else
                            buttonRobotsVisible.displayMode = "normal"
                    }
                    else
                        buttonRobotsVisible.displayMode = "none"
                }
            }
        }

        MapButton
        {
            id: buttonToggleLayer
            objectName: "buttonToggleLayer"
            anchors.top: buttonRobotsVisible.bottom
            icon: "qrc:/mapwidget/icons/layers64.png"
            onClicked:
            {
                mapItem.toggleLayer();
            }
        }

        MapButton
        {
            id: buttonOverview
            objectName: "buttonOverview"
            anchors.top: buttonToggleLayer.bottom
            property bool toggled: false
            icon: "qrc:/mapwidget/icons/overview.png"
            colorBackground:
            {
                toggled ? MapStyle.colors.border : MapStyle.colors.background
            }

            function turn(aOn)
            {
                if (toggled === aOn)
                    return

                toggled = aOn
                if (toggled)
                {
                    mapItem.overviewPlacemarks()
                    mapItem.turnPlacemarksOverview()
                }
            }

            onClicked:
            {
                mapItem.overviewPlacemarks()
            }

            onPressedShort:
            {
                var toggle = !toggled
                if (toggle)
                    followOverview()
                else
                {
                    stopCameraFollowing()
                }
            }

            visible: { mapInfoPanel.count() > 0 }
            onVisibleChanged:
            {
                if (!visible && mapControls.visible)
                    stopCameraFollowing()
            }

            Connections
            {
                target: controller
                onCameraChangePreventedWhenTracking:
                {   //give visual hint as to why zoom was prevented (user needs to turn off tracking)
                    if (buttonOverview.toggled)
                    {
                        buttonOverview.standOut()
                    }
                }
            }
        }

        MapButton
        {
            id: buttonTogglePaths
            objectName: "buttonTogglePaths"
            anchors.top: buttonOverview.bottom
            icon: "qrc:/mapwidget/icons/paths64.png"
            visible: { mapInfoPanel.count() > 0 && buttonRobotsVisible.displayMode !== "none"}
            onClicked:
            {
                mapItem.togglePaths();
            }
        }

        /* Work in progress
        SensorDataItem
        {
            id: sensorLayerGui
            objectName: "sensorLayerGui"
            anchors.top: buttonTogglePaths.bottom
        }
        */

        MapButton
        {
            id: buttonShowMapEdit
            objectName: "buttonShowMapEdit"
            anchors.top: parent.top
            anchors.left: buttonZoomIn.right
            marginTop: MapStyle.margins.firstButtonMargin
            marginLeft: MapStyle.margins.smallLeftMargin
            icon: "qrc:/mapwidget/icons/editMap.png"
            onClicked:
            {
                mapEditWindow.visible = !mapEditWindow.visible
                if (mapEditWindow.visible)
                {
                    robotEditWindow.visible = false
                }
            }
        }

        MapEditWindow
        {
            id: mapEditWindow
            objectName: "mapEditWindow"
            anchors.left: buttonShowMapEdit.right
            anchors.top: buttonShowMapEdit.top
        }

        MapButton
        {   //TODO - only show when there is a non-autoplacement robot
            id: buttonShowRobotEdit
            objectName: "buttonShowRobotEdit"
            anchors.top: buttonShowMapEdit.bottom
            anchors.left: buttonZoomIn.right
            marginLeft: MapStyle.margins.smallLeftMargin
            visible: false
            icon: "qrc:/mapwidget/icons/positionRobot.png"
            onClicked:
            {
                robotEditWindow.visible = !robotEditWindow.visible
                if (robotEditWindow.visible)
                {
                    mapEditWindow.visible = false
                    toggleLocalMapPositioning.cancel()
                }
            }

            Connections
            {
                target: mapItem
                onLocalMapVisibilityChanged:
                {
                    buttonShowRobotEdit.visible = aVisible
                    if (!aVisible)
                    {
                        robotEditWindow.visible = false
                    }
                }
            }
        }

        RobotEditWindow
        {
            id: robotEditWindow
            objectName: "robotEditWindow"
            anchors.left: buttonShowRobotEdit.right
            anchors.top: buttonShowRobotEdit.top
        }

        MapButton
        {
            id: toggleLocalMapPositioning
            objectName: "toggleLocalMapPositioning"
            anchors.top: buttonShowRobotEdit.bottom
            anchors.left: buttonShowRobotEdit.left
            marginLeft: 0
            icon: "qrc:/mapwidget/icons/localMapPositioning.png"
            property bool toggled: false
            property bool available: false
            visible: false

            function cancel()
            {
                if (toggled)
                {
                    mapItem.toggleLocalMapPositioning()
                    toggled = !toggled
                }
            }

            onClicked:
            {
                mapItem.toggleLocalMapPositioning()
                toggled = !toggled
                if (toggled)
                {
                    robotEditWindow.visible = false
                }
            }

            colorBackground:
            {
                toggled ? MapStyle.colors.border : MapStyle.colors.background
            }

            Connections
            {
                target: mapItem
                onLocalMapHasContent:
                {
                    toggleLocalMapPositioning.available = true
                }
                onLocalMapVisibilityChanged:
                {
                    toggleLocalMapPositioning.visible = aVisible && toggleLocalMapPositioning.available
                }
            }
        }

        MapButton
        {
            id: toggleLocalMap
            objectName: "toggleLocalMap"
            anchors.top: toggleLocalMapPositioning.bottom
            anchors.left: toggleLocalMapPositioning.left
            marginLeft: 0
            icon: "qrc:/mapwidget/icons/toggleLocalMap.png"
            visible: false
            onClicked:
            {
                mapItem.toggleLocalMapVisibility()
            }

            Connections
            {
                target: mapItem
                onLocalMapHasContent:
                {
                    toggleLocalMap.visible = true
                }
            }
        }

        //Left side, bottom-up
        Rectangle
        {
            id: placeholderBottom
            anchors.bottom: parent.bottom
            height: 280
            visible: false
        }

        MapInfoPanel
        {
            id: mapInfoPanel
            objectName: "mapInfoPanel"
            anchors.bottom: placeholderBottom.top
        }

        //Right side, bottom-up

        Rectangle
        {
            id: rightBottomPanel
            objectName: "rightBottomPanel"
            color: "transparent"
            width: childrenRect.width
            height: childrenRect.height
            anchors.bottom: parent.bottom
            anchors.right: parent.right
            anchors.rightMargin:
            {
                if (mapItem.state === "showFull")
                    MapStyle.margins.rightPanelFullscreenMargin
                else
                    MapStyle.margins.rightPanelMargin
            }
            anchors.bottomMargin: MapStyle.margins.bottomMargin

            Behavior on anchors.rightMargin { NumberAnimation { duration: 250 } }

            GpsPanel
            {
                id: gpsPanel
                visible: false
            }

            LicensePanel
            {
                id: licensePanel
                anchors.top: gpsPanel.bottom
                anchors.right: gpsPanel.right
            }
        }
    }
}




import QtQuick 2.2
import SensorControl 1.0
import "."

Rectangle
{
    id: sensorLayerGui
    width: 240; height: 100
    color: "transparent"
    visible: true

    SensorControl
    {
        id: sensorController
        objectName: "sensorController"
    }

    ListModel
    {
        id: missionList
        ListElement { missionID: 1; missionName: "Misja 1"}
        ListElement { missionID: 2; missionName: "Ratuj Orkę"}
        ListElement { missionID: 3; missionName: "Nie ratuj orki"}
    }

    MapButton
    {
        id: toggleSensorControlView
        icon: "qrc:/mapwidget/icons/hazardsensors.png"
        anchors.top: parent.top
        onClicked:
        {
            missionsListView.visible = !missionsListView.visible
        }
    }

    Component
    {
        id: missionDelegate
        Item
        {
            width: 120
            height: 20
            Rectangle
            {
                id: styleRect
                anchors.fill: parent
                color: MapStyle.window.background
                border.color: MapStyle.colors.border
                border.width: 2
                opacity: MapStyle.window.lowOpacity
                smooth: true
                radius: 10
            }
            Rectangle
            {
                anchors.leftMargin: 5
                id: layoutRect
                color: "transparent"
                Text { text: missionID + ": " + missionName }
            }
        }
    }

    ListView
    {
        id: missionsListView
        anchors.left: toggleSensorControlView.right
        anchors.top: parent.top
        visible: false
        height: 90
        width: 120
        model: missionList
        delegate: missionDelegate
        highlight: Rectangle { color: "lightsteelblue"; radius: 5}
        focus: true
    }
}




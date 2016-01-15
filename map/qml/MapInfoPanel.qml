import QtQuick 2.2
import "."

PlacemarkListWindow
{
    id: mapInfoPanel
    anchors.left: parent.left
    placemarksList: robotInfoList
    placemarksListModel: robotInfoModel

    ListModel
    {
        id: robotInfoModel
    }

    Component
    {
        id: robotInfoListDelegate
        Item
        {
            height: robotInfoList.singleItemHeight
            width: robotInfoList.singleItemWidth

            function buttonClicked()
            {
                var itemIndex = modelItemIndex(placemarkID)
                mapItem.onRobotConnectToggle(robotID);
                selectItem(itemIndex, true)
            }

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

            MapButton
            {
                id: buttonRobotCenter
                anchors.left: styleRect.left
                property int verticalMargin: { (parent.height - height)/2 }
                marginTop: verticalMargin
                marginBottom: verticalMargin
                marginLeft: MapStyle.margins.smallLeftMargin
                icon: iconPath
                colorBackground:
                {
                    toggled ? MapStyle.colors.border : MapStyle.colors.background
                }

                Image
                {
                    anchors.bottom: parent.bottom
                    anchors.right: parent.right
                    height: { parent.height / 2 }
                    width:  { parent.width / 2 }
                    source: { robotState === "connected" ? "qrc:/mapwidget/icons/connected.png" : (robotState === "disconnected" ? "qrc:/mapwidget/icons/disconnected.png" : "") }
                }
                property bool toggled: false
                function turn(aOn)
                {
                    if (toggled === aOn)
                        return

                    toggled = aOn
                    if (toggled)
                    {
                        mapItem.center(placemarkID)
                        mapItem.turnFollowPlacemark(placemarkID);
                    }
                }

                onClicked: { buttonClicked() }
                onPressedShort:
                {
                    var toggle = !toggled
                    if (toggle)
                        mapItem.followPlacemark(placemarkID)
                    else
                        mapItem.stopCameraFollowing()
                }

                Connections
                {
                    target: mapItem
                    onFollowedPlacemarkIndexChanged:
                    {
                        if (mapItem.followedPlacemarkIndex === placemarkID)
                            buttonRobotCenter.turn(true)
                        else
                            buttonRobotCenter.turn(false)
                    }
                }
                Connections
                {
                    target: controller
                    onCameraChangePreventedWhenTracking:
                    {//give visual hint as to why zoom was prevented (user needs to turn off tracking)
                        if (mapItem.followedPlacemarkIndex === placemarkID)
                            buttonRobotCenter.standOut();
                    }
                }
            }

            Rectangle
            {
                id: textLayout
                width: robotInfoList.singleItemTextWidth
                anchors.left: buttonRobotCenter.right
                anchors.verticalCenter: parent.verticalCenter
                anchors.leftMargin: 5
                anchors.topMargin: 5
                height:
                {
                    placemarkIDLabel.height + consoleIDLabel.height + locationLabel.height
                }
                color: "transparent"

                Text
                {
                    id: placemarkIDLabel
                    anchors.top: parent.top
                    text: robotType + " " + robotName + " " + robotID;
                }
                Text
                {
                    id: consoleIDLabel
                    anchors.top: placemarkIDLabel.bottom
                    text: qsTr("Console ID") + ": " + consoleID;
                }
                Text
                {
                    id: locationLabel
                    anchors.top: consoleIDLabel.bottom
                    text: location;
                }

                Timer
                {
                    id: listTextClickEffectTimer
                    interval: 100; running: false; repeat: false
                    onTriggered:
                    {
                        styleRect.color = MapStyle.window.background
                    }
                }

                MouseArea
                {
                    anchors.fill: parent
                    onClicked:
                    {
                        buttonClicked()
                        styleRect.color = MapStyle.colors.shadow
                        listTextClickEffectTimer.start()
                    }
                }
            }
        }
    }

    ListView
    {
        id: robotInfoList
        property int singleItemTextWidth: 165
        property int singleItemIconWidth: 50
        property int singleItemWidth: { singleItemIconWidth + singleItemTextWidth }
        property int singleItemHeight: 58
        property real maxDisplayedItems: 3
        highlightFollowsCurrentItem: true
        //boundsBehavior: Flickable.StopAtBounds
        width: parent.width
        height: parent.height
        model: robotInfoModel
        delegate: robotInfoListDelegate
        clip: true

        add: Transition
        {
            NumberAnimation
            {
                properties: "scale"; from: 0; to: 1; duration: timeToShowSlow
            }
        }

        displaced: Transition
        {
            NumberAnimation
            {
                properties: "x,y"; duration: timeToShowSlow
            }
        }

        remove: Transition
        {
            SequentialAnimation
            {
                PropertyAction { property: "ListView.delayRemove"; value: true }
                NumberAnimation { property: "scale"; to: 0; duration: timeToShowSlow }
                PropertyAction { property: "ListView.delayRemove"; value: false }
            }
        }

        highlight:
            Component
            {
                Rectangle
                {
                    width: robotInfoList.singleItemWidth
                    height: robotInfoList.singleItemHeight
                    color: "skyblue"
                    border.color: "yellow"
                    border.width: 2
                    opacity: 0.66
                    radius: 10
                }
            }
    }

    Connections
    {
        target: controller
        onRobotUpdate:
        {
            console.log("STATE!!: ", aStateString)
            for (var i = 0; i < robotInfoModel.count; i++)
            {   //Find if already on list
                if(robotInfoModel.get(i).placemarkID === aPlacemarkID)
                {
                    var item = modelItem(aPlacemarkID)
                    if (item)
                    {   //placemarkID and robotID are not changed (not supposed to change
                        item.robotName = aName
                        item.robotType = aType
                        item.iconPath = aIconPath
                        item.robotState = aStateString
                        item.location = aPos
                        item.consoleID = aConsoleID
                    }
                    return;
                }
            }

            //not on list - add robot to list
            robotInfoModel.append( {"placemarkID": aPlacemarkID,
                                    "robotName" : aName,
                                    "robotType" : aType,
                                    "consoleID": aConsoleID,
                                    "location": aPos,
                                    "robotState" : aStateString,
                                    "robotID": aRobotID,
                                    "iconPath": aIconPath} )
            mapInfoPanel.updateVisibilityFromModel()
            selectItem(modelItemIndex(aPlacemarkID, false));
        }

        onRobotRemove:
        {
            if (robotInfoModel.count > 0)
            {
                for (var i = 0; i < robotInfoModel.count; i++)
                {
                    if (robotInfoModel.get(i).placemarkID === aPlacemarkID)
                    {
                        if (robotInfoModel.get(i).placemarkID === mapItem.followedPlacemarkIndex)
                            mapItem.stopCameraFollowing()

                        robotInfoModel.remove(i)
                        break
                    }
                }
            }
            mapInfoPanel.updateVisibilityFromModel()
        }

        onItemSelected:
        {
            if (robotInfoList.count > aItemIndex)
            {
                selectItem(aItemIndex, true)
            }
        }

        onRobotsVisibilityChanged:
        {
            mapInfoPanel.visible = aVisible
        }
    }
}


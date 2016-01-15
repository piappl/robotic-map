import QtQuick 2.2
import "."

PlacemarkListWindow
{
    id: placesListWindow
    anchors.left: parent.left
    placemarksList: placeInfoList
    placemarksListModel: filteredPlacesModel
    property string filter: "place"
    toggleable: false
    visible: { (placemarksListModel.count > 0
               && mapEditWindow.showPlacesList) }

    Connections
    {
        target: mapEditWindow
        onModeChanged:
        {
            filteredPlacesModel.proxyUpdate()
            updateList()
        }
    }

    function addPlace(aItem)
    {
        filteredPlacesModel.proxyAddItem(aItem)
        allPlacesModel.append(aItem)
        updateList()
    }

    function updateList()
    {
        placesListWindow.updateVisibilityFromModel()
    }

    ListModel
    {
        id: allPlacesModel
    }

    ListModel
    {
        id: filteredPlacesModel
        function proxyUpdate()
        {   //TODO - sort by number?
            filteredPlacesModel.clear()
            for (var i = 0; i != allPlacesModel.count; ++i)
            {
                var item = allPlacesModel.get(i)
                if (item.type === mapEditWindow.mode)
                {
                    filteredPlacesModel.append(item);
                }
            }
        }
        function proxyAddItem(aItem)
        {
            if (aItem.type === mapEditWindow.mode)
            {
                filteredPlacesModel.append(aItem);
            }
        }

        function proxyRemoveItem(aItem)
        {
            for (var i = 0; i < filteredPlacesModel.count; i++)
            {
                if (filteredPlacesModel.get(i).placemarkID === aItem.placemarkID)
                {
                    filteredPlacesModel.remove(i)
                    break
                }
            }
        }

        function proxyUpdateItem(aItem, aPos, aNumber)
        {
            for (var i = 0; i < filteredPlacesModel.count; i++)
            {
                var item = filteredPlacesModel.get(i)
                if (item.placemarkID === aItem.placemarkID)
                {
                    item.location = aPos
                    item.number = aNumber
                    return
                }
            }
        }
    }

    Component
    {   //Similar to MapInfoPanel (TODO)
        id: placeInfoListDelegate
        Item
        {
            height: placeInfoList.singleItemHeight
            width: placeInfoList.singleItemWidth

            function buttonClicked()
            {
                var itemIndex = modelItemIndex(placemarkID)
                selectItem(itemIndex, true)
                mapEditor.placemarkClickedOnList(placemarkID)
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

            Rectangle
            {
                id: textLayout
                anchors.fill: parent
                anchors.verticalCenter: parent.verticalCenter
                anchors.leftMargin: 5
                anchors.topMargin: 5
                color: "transparent"

                Text
                {
                    id: locationLabel
                    anchors.top: parent.top
                    text:
                    {
                        if (number !== -1)
                            number + " - " + location
                        else
                            location
                    }
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
        id: placeInfoList
        visible:  { placesListWindow.state !== "hide" }
        property int singleItemWidth: 175
        property int singleItemHeight: 30
        property real maxDisplayedItems: 6
        highlightFollowsCurrentItem: true
        //boundsBehavior: Flickable.StopAtBounds
        width: parent.width
        height: parent.height
        model: filteredPlacesModel
        delegate: placeInfoListDelegate
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
                    width: placeInfoList.singleItemWidth
                    height: placeInfoList.singleItemHeight
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
        onPlaceUpdate:
        {
            for (var i = 0; i < allPlacesModel.count; i++)
            {
                var item = allPlacesModel.get(i)
                if(item.placemarkID === aPlacemarkID)
                {
                    filteredPlacesModel.proxyUpdateItem(item, aPos, aNumber)
                    item.location = aPos
                    item.number = aNumber
                    return
                }
            }
            var newItem = {"placemarkID": aPlacemarkID, "location": aPos,
                "number": aNumber, "type": aType }
            addPlace(newItem)
            selectItem(modelItemIndex(aPlacemarkID, false));
        }

        onPlaceRemove:
        {
            if (allPlacesModel.count > 0)
            {
                for (var i = 0; i < allPlacesModel.count; i++)
                {
                    var item = allPlacesModel.get(i)
                    if (item.placemarkID === aPlacemarkID)
                    {
                        filteredPlacesModel.proxyRemoveItem(item)
                        allPlacesModel.remove(i)
                        break
                    }
                }
            }
            updateList();
        }
    }
}


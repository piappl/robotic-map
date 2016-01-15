import QtQuick 2.2
import "."

DefaultWindow
{
    id: placemarkListWindow
    anchors.left: parent.left
    width:
    {
        if (placemarksList)
            placemarksList.singleItemWidth
        else 0
    }
    height:
    {
        if (placemarksList)
            placemarksList.singleItemHeight * Math.max(1, Math.min(placemarksList.maxDisplayedItems, placemarksListModel.count))
        else
            0
    }

    color: "transparent"
    colorBackground: "transparent"
    colorShadow: "transparent"
    colorBorder: "transparent"
    state: "hide"
    enabled: false
    visible: { placemarksListModel.count > 0 }
    childAnimate: true
    showAfterCreation: false
    property bool fadedOut: false
    property int timeToShow: MapStyle.animation.animationTime
    property int timeToShowSlow: MapStyle.animation.slowAnimationTime
    property bool showArrows: true
    property bool toggleButtonMoving: true
    property bool toggleable: true
    scale: { fadedOut ? 0.0 : 1.0 }
    Behavior on scale { NumberAnimation { duration: timeToShow } }
    Behavior on height { NumberAnimation { duration: timeToShow } }
    property ListModel placemarksListModel
    property ListView placemarksList

    function updateVisibilityFromModel()
    {
        if (!placemarksListModel)
        {
            console.warn("No placemark list model..")
            return
        }

        if (placemarksListModel.count > 0)
        {
            if (placemarkListWindow.state === "hide")
            {
                show()
            }
            placemarkListWindow.enabled = true
        }
        else
        {
            if (placemarkListWindow.state === "show")
            {
                hide()
            }
            placemarkListWindow.enabled = false
        }
    }

    function count()
    {
        return placemarksListModel.count
    }

    function toggle()
    {
        if(placemarkListWindow.state === "hide")
            show();
        else
            hide();
    }

    function show()
    {
        placemarkListWindow.state = "show";
        toggleButton.toggle();
    }

    function hide()
    {
        placemarkListWindow.state = "hide"
        toggleButton.toggle();
    }

    function modelItem(aID)
    {
        var itemIndex = modelItemIndex(aID)
        return placemarksListModel.get(itemIndex)
    }

    function modelItemIndex(aID)
    {
        for (var i = 0; i < placemarksListModel.count; i++)
        {
            if (placemarksListModel.get(i).placemarkID === aID)
            {
                return i
            }
        }
        return -1
    }

    function selectItem(aIndex, aCenter)
    {
        if (aIndex === -1)
            return

        if (placemarksList.currentIndex !== aIndex)
            placemarksList.currentIndex = aIndex
        if (aCenter)
            centerOnItem(aIndex) //always center on item - because user might have navigated away
    }

    function centerOnItem(aIndex)
    {
        mapItem.center(placemarksListModel.get(aIndex).placemarkID)
    }

    MapInfoPanelArrow
    {
        id: arrowUp
        orientation: "up"
    }

    MapInfoPanelArrow
    {
        id: arrowDown
        orientation: "down"
    }

    MapButton
    {
        id: toggleButton
        visible: { placemarkListWindow.enabled && placemarkListWindow.toggleable }
        anchors.top: parent.top
        property int verticalMargin: { (placemarkListWindow.height - height)/2 }
        property bool revealed: false
        anchors.left:
        {
            if (placemarkListWindow.toggleButtonMoving || !revealed)
                parent.right
            else
                placemarkListWindow.left
        }

        marginTop: verticalMargin
        marginBottom: verticalMargin
        marginLeft:
        {
            var offset = placemarkListWindow.toggleButtonMoving ? 0 : -width
            if (revealed)
            {
                0 + offset
            }
            else
            {
                MapStyle.margins.leftMargin
            }
        }
        rotation:   { if (revealed) { 270 } else { 90 } }
        Behavior on rotation { NumberAnimation { duration: timeToShow } }
        icon: "qrc:/mapwidget/icons/arrowUp32.png"

        function toggle() { revealed = !revealed; }
        onClicked: { placemarkListWindow.toggle() }
    }

    states: [
        State {
            name: "hide"
            PropertyChanges
            {
                target: placemarkListWindow
                anchors.leftMargin: -placemarkListWindow.width
            }
        },
        State {
            name: "show"
            PropertyChanges
            {
                target: placemarkListWindow
                anchors.leftMargin:
                {
                    if (toggleButtonMoving)
                        5
                    else
                        toggleButton.width + 5
                }
            }
        }
    ]

    transitions: [
        Transition
        {
            from: "show"
            to: "hide"
            NumberAnimation
            {
                properties: "anchors.leftMargin";
                easing.type: MapStyle.animation.animationType;
                duration: timeToShow
            }
        },
        Transition
        {
            from: "hide"
            to: "show"
            NumberAnimation
            {
                property: "anchors.leftMargin";
                easing.type: MapStyle.animation.animationType;
                duration: timeToShow
            }
        }
    ]
}


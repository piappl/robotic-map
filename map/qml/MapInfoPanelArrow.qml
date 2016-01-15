import QtQuick 2.2

Image
{
    visible: { placemarkListWindow.showArrows }
    property string orientation
    property bool showCondition:
    {
        if (orientation === "up")
            placemarksList.currentIndex > 0
        else
            placemarksList.currentIndex < placemarksList.count - 1
    }
    enabled:
    {
        if (orientation === "up")
            placemarksList.currentIndex !== 0
        else
            placemarksList.currentIndex !== placemarksList.count - 1
    }
    anchors.bottom: { orientation === "up" ? parent.top : undefined }
    anchors.top: { orientation === "up" ? undefined : parent.bottom }
    anchors.horizontalCenter: parent.horizontalCenter

    opacity:
    {
        if (showCondition)
            0.6
        else
            0
    }
    Behavior on opacity {  NumberAnimation { duration: 500; easing.type: Easing.Linear } }
    width: 32
    height: 32
    source: "qrc:/mapwidget/icons/arrowUp32.png"
    rotation:
    {
        if (orientation === "up")
            0
        else if (orientation === "down")
            180
        else if (orientation === "right")
            90
        else
            270
    }

    MouseArea
    {
        anchors.fill: parent
        onClicked:
        {
            if (showCondition)
            {
                var increase = orientation === "up" ? -1 : 1
                placemarkListWindow.selectItem(placemarksList.currentIndex + increase)
            }
        }
    }
}

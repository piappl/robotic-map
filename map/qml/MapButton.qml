import QtQuick 2.2
import "."

MyButton
{
    id: button
    width: MapStyle.button.width
    height: MapStyle.button.height
    marginLeft: MapStyle.margins.leftMargin
    marginTop: MapStyle.margins.topMargin
    marginBottom: MapStyle.margins.bottomMargin
    clickDelay: MapStyle.button.clickDelay
    normalOpacity: MapStyle.button.opacity
    colorBackground: MapStyle.colors.background
    colorHover: MapStyle.colors.hover
    colorBorder: MapStyle.colors.border
    colorShadow: MapStyle.colors.shadow
    property real alertScale: 1.2
    property int mapButtonAnimationTime: MapStyle.animation.animationTime

    Rectangle
    {
        id: alertRectangle
        width: Math.round(button.width / visibleRatio)
        height: Math.round(button.height / visibleRatio)
        radius: parent.radius
        anchors.fill: parent
        scale: 1.0 / visibleRatio
        color: MapStyle.colors.alert
        opacity: 0
        Behavior on opacity { NumberAnimation { duration: mapButtonAnimationTime } }
    }

    Timer
    {
        id: standOutTimer
        interval: mapButtonAnimationTime; running: false; repeat: false
        onTriggered:
        {
            button.scale = 1
            alertRectangle.opacity = 0
        }
    }

    function standOut()
    {
        button.scale = alertScale
        alertRectangle.opacity = MapStyle.window.lowOpacity
        standOutTimer.start()
    }

    Component.onCompleted:
    {
        button.showButton(true)
    }
}


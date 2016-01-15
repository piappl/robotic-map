import QtQuick 2.2
import QtGraphicalEffects 1.0

Rectangle {
    id: defaultWindow

    property string colorShadow: "grey"
    property string colorBackground: "black"
    property string colorBorder: "blue"
    property int parentRadius: 5
    property real parentOpacity: 1.0
    property bool showAfterCreation: true

    property bool childAnimate: false //set to true in child to handle state animations

    function setAnimateTime(aVal)
    {
        d.animateTime = aVal
    }

    function show()
    {
        defaultWindow.state = "show"
    }

    function hide()
    {
        defaultWindow.state = "hide"
    }

    QtObject {
        id: d
        property int animateTime: 400
        property string colorFont: "white"
        property string colorFontOutline: "black"
    }

    radius: parentRadius
    color: "transparent"
    state: "hide"

    /* if you using loader, connect to signal to destroy object */
    signal destroyMe()

    RectangularGlow {
        id: effect
        anchors.fill: rect
        cached: true
        color: colorShadow
        cornerRadius: rect.radius + glowRadius
        glowRadius: 3
        spread: 0.6
    }

    Rectangle {
        id: rect
        anchors.centerIn: parent
        color: parent.colorBackground
        width: parent.width - 3
        height: parent.height - 3
        radius: parent.radius
        border.color: parent.colorBorder
        border.width: 1
    }

    Timer {
        id: timerHide
        interval: d.animateTime; repeat: false; running: false
        onTriggered: {
            destroyMe()
        }
    }

    Component.onCompleted: {

        if (showAfterCreation == true)
            defaultWindow.state = "show"
        else
            defaultWindow.state = "hide"
    }
//    Component.onDestruction: state = "hide"

    states: [
        State {
            name: childAnimate ? "parent.hide" : "hide"
            PropertyChanges { target: defaultWindow; opacity: 0 }
            PropertyChanges { target: defaultWindow; scale: 0 }
        },
        State {
            name: childAnimate ? "parent.show" : "show"
            PropertyChanges { target: defaultWindow; opacity: parentOpacity }
            PropertyChanges { target: defaultWindow; scale: 1 }
        }

    ]
    transitions: [
        Transition {
            NumberAnimation { properties: "scale,opacity"; easing.type: Easing.InQuart; duration: d.animateTime}

        }
    ]
}

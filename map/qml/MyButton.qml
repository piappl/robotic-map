import QtQuick 2.2
import QtGraphicalEffects 1.0

Rectangle {
    id: button

    property bool markAlligment: false
    property real visibleRatio: 1.2
    property int index: 0
    property int marginTop: 3
    property int marginRight: 3
    property int marginLeft: 3
    property int marginBottom: 3
    property alias caption: name.text
    property alias markSource: mark.source
    property alias markWidth: mark.width
    property alias pointSizeCaption: name.font.pointSize
    property alias icon: image.source
    property int timeToShow: 200
    property int scaleAnimate: 100
    property int clickDelay: 200
    property real normalScale: 1.0
    property real normalOpacity: 0.7
    property bool centerText: false
    property bool iconShadow: false
//    property bool show: false
    property bool pressed: false
    property string defaultColorShadow: "grey"
    property string colorShadow: "grey"
    property string colorBackground: "black"
    property string colorHover: "blue"
    property string colorBorder: "blue"

//    anchors.rightMargin: margins; anchors.leftMargin: margins; anchors.topMargin: margins
    width: 50
    height: 50
    color: "transparent"
    opacity: normalOpacity
    radius: 10
    smooth: false
    state: "show"

    signal clicked()
    signal clickedAndAnimate()
    signal pressedShort()
    signal pressedLong()
    signal pressedVeryLong()
    signal show()
    signal hide()
    signal showChanged(bool aOn)

    function showButton(aOn)
    {
        if(aOn){
            timerStart.start()
        }else {
//            button.show = false
            state = "hide"
        }
    }

    function setPressed()
    {
        showPressed()
        button.clicked()
    }

    function showPressed()
    {
        button.pressed = true
        effect.color = button.colorHover
        delay.start()
    }

    function setReleased()
    {
        button.pressed = false
        effect.color = button.defaultColorShadow
    }

    function setColorShadow(aVal)
    {
        colorShadow = aVal
    }

    function setTimeToAnimate(aVal)
    {
        effect.timeToAnimate = aVal
    }

    RectangularGlow {
        id: effect
        property int timeToAnimate: 500
        anchors.fill: rect
        cached: true
        color: button.colorShadow
        cornerRadius: rect.radius + glowRadius
        glowRadius: 3
        spread: 0.6
    }

    Rectangle {
        id: rect
        anchors.centerIn: button
        color: button.colorBackground

        width: Math.round(button.width / visibleRatio)
        height: Math.round(button.height / visibleRatio)
        radius: parent.radius

        border.color: button.colorBorder
        border.width: 1
        Rectangle {
            id: title
            anchors.fill: parent
            color: "transparent"
            Image {
                id: mark               
                anchors.verticalCenter: parent.verticalCenter
                scale: 0.0
//                width: 0
            }
            ColorOverlay {
                anchors.fill: mark
                source: mark
                color: "#5fffffff"
                scale: 0.8
            }

            Text {
                id: name
                text: ""
                font.pointSize: 12; //style: Text.Outline; styleColor: "blue"
                color: "white"
                anchors.centerIn: parent
//                anchors.left: button.center === true ? mark.right : ""; anchors.leftMargin: 10
                Component.onCompleted: {
                    if(!button.centerText){
                        anchors.left = mark.right
                        anchors.leftMargin = 10
                    }
                }
            }
        }

        Component.onCompleted: {
            if (markAlligment == true) {
                mark.x = rect.x+rect.width-mark.width-5
            }
        }
    }

    Timer {
        id: timerStart
        interval: button.timeToShow; running: false; repeat: false
        onTriggered: {
            state = "show"
        }
    }

    Timer {
        id: delay
        interval: button.clickDelay; running: false; repeat: false
        onTriggered: {
            effect.color = button.colorShadow
            button.pressed = false
        }
    }

    Timer {
        id: timerLongPress
        property int cInterval: 100
        property int cnt: 0
        interval: cInterval; running: false; repeat: true
        onTriggered: {
            cnt++
            if(cnt === cInterval/10){
//                pressedShort()
            }else if(cnt === cInterval/5)
                pressedLong()
            else if(cnt === cInterval/2){
                pressedVeryLong()
                timerLongPress.stop()
                cnt = 0
            }
        }
    }

    Timer {
        id: timerClickAnimate
        interval: scaleAnimate; running: false; repeat: false
        onTriggered: {
            button.scale = normalScale
            clickedAndAnimate()
        }
    }

    Image {
        id: image
        anchors.fill: rect
        anchors.margins: 4
//        scale: 0.8
        source: ""
//        visible: false
    }

    Glow {
        id: glow
        anchors.fill: image
        cached: true
        color: rect.border.color
        radius: 8
        samples: 16
        spread: 0.2
        source: image
        visible: iconShadow
        scale: image.scale
    }

    onScaleChanged: {
        if(scale === 0)
            showChanged(false)
        else if(scale === 1)
            showChanged(true)
    }

//    border.width: 1
//    border.color: "#1a1a1a"
//        gradient: Gradient {
//            GradientStop {
//                position: 0.00;
//                color: "#6f6f6f";
//            }
//            GradientStop {
//                position: 0.40;
//                color: "#000000";
//            }
//            GradientStop {
//                position: 1.00;
//                color: "#1a1919";
//            }
//        }

//    radius: 10.0
    Behavior on anchors.topMargin { NumberAnimation { easing.period: 0.74; easing.amplitude: 2; easing.type: Easing.InElastic; duration: effect.timeToAnimate } }
    Behavior on anchors.rightMargin { NumberAnimation { easing.period: 0.74; easing.amplitude: 2; easing.type: Easing.InElastic; duration: effect.timeToAnimate } }
    Behavior on anchors.leftMargin { NumberAnimation { easing.period: 0.74; easing.amplitude: 2; easing.type: Easing.InElastic; duration: effect.timeToAnimate } }
//    Behavior on anchors.bottomMargin { NumberAnimation { easing.period: 0.74; easing.amplitude: 2; easing.type: Easing.InElastic; duration: effect.timeToAnimate } }
//    Behavior on anchors.margins { NumberAnimation { easing.period: 0.74; easing.amplitude: 2; easing.type: Easing.InElastic; duration: effect.timeToAnimate } }
    Behavior on scale { NumberAnimation { duration: scaleAnimate } }

    states: [
        State {
            name: "hide"
//            PropertyChanges { target: button; anchors.margins: -height }
            PropertyChanges { target: button; anchors.topMargin: -height }
            PropertyChanges { target: button; anchors.rightMargin: -height }
            PropertyChanges { target: button; anchors.leftMargin: -height }
//            PropertyChanges { target: button; anchors.bottomMargin: -height }
            PropertyChanges { target: button; scale: 0 }
        },
        State {
            name: "show"
            PropertyChanges{ target: button; anchors.topMargin: marginTop }
            PropertyChanges{ target: button; anchors.rightMargin: marginRight }
            PropertyChanges{ target: button; anchors.leftMargin: marginLeft }
//            PropertyChanges { target: button; anchors.bottomMargin: marginBottom }
            PropertyChanges { target: button; scale: 1 }
        }
    ]

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        enabled: parent.enabled
        onPressed: {
            button.scale = normalScale - 0.1
            timerLongPress.start()
        }

        onClicked: {
            button.scale = normalScale - 0.1
            button.clicked()
            timerClickAnimate.start()
        }

        onReleased: {
            button.scale = normalScale
            if(timerLongPress.cnt < timerLongPress.cInterval/10){
                button.pressed = true
                effect.color = button.colorHover
                delay.start()
//                button.clicked()
            }
            timerLongPress.stop()
            timerLongPress.cnt = 0
        }
        onPressAndHold: pressedShort()
//        onEntered: {
//            button.color = "#f0f0f0"
//        }
//        onExited: {
//            button.color = "#707070"
//        }
//        onReleased: {
//            button.pressed = false
//        }
    }

//    Connections {
//        target: main
//        onSetRadius: rect.radius = aVal
//        onSetOpacity: normalOpacity = aVal    /*
//    }
    onStateChanged: {
        if(state == "hide")
            button.hide()
        if(state == "show")
            button.show()
    }

    Component.onCompleted: {
        opacity = normalOpacity
    }
}

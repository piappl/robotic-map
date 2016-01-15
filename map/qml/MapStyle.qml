pragma Singleton
import QtQuick 2.2

QtObject
{
    property QtObject colors: QtObject
    {
        property color border: "skyblue"
        property color background: "lightcyan"
        property color hover: "transparent"
        property color shadow: "gray"
        property color alert: "red"
    }
    property QtObject margins: QtObject
    {
        property int firstButtonMargin: 60
        property int leftMargin: 10
        property int rightMargin: 10
        property int rightPanelMargin: 20
        property int rightPanelFullscreenMargin: 100
        property int smallLeftMargin: 3
        property int topMargin: 1
        property int bottomMargin: 1
    }
    property QtObject animation: QtObject
    {
        property int quickAnimationTime: 150
        property int animationTime: 250
        property int slowAnimationTime: 400
        property int animationType: Easing.InQuart
    }
    property QtObject button: QtObject
    {
        property int width: 50
        property int height: 50
        property int clickDelay: 100
        property real opacity: 0.7
    }
    property QtObject window: QtObject
    {
        property real opacity: 0.7
        property real lowOpacity: 0.6
        property real highOpacity: 0.8
        property color background: "white"
    }
}

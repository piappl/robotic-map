import QtQuick 2.2
import "."

Text
{
    id: gpsPanel
    width: paintedWidth
    height: paintedHeight
    text: ""
    style: Text.Outline
    styleColor: "white"
    font.pointSize: 10
    Connections
    {
        target: controller
        onCenterPositionChanged:
        {
            text = aPosition
        }
    }
}


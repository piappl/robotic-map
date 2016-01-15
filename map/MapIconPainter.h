#ifndef MAPICONPAINTER_H
#define MAPICONPAINTER_H

#include <MapObjectsFwd.h>
#include <QImage>

class MapIconPainter
{
public:
    QImage paintIcon(QImage base, MapAbstraction::MapObjectConstPtr mapObject, bool simplified);

    //TODO - cache: MapIconCache for complex icons
};

#endif // MAPICONPAINTER_H

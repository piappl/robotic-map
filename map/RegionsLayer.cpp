#include <marble/GeoPainter.h>
#include <marble/GeoDataCoordinates.h>

#include "GeoCoords.h"
#include "GeoObjectsManager.h"
#include "PlacemarkType.h"
#include "MapLibraryHelpers.h"
#include "MapPolygonNodeObject.h"
#include "RegionsLayer.h"

using namespace MapAbstraction;

namespace Marble
{
    RegionsLayer::RegionsLayer(MapAbstraction::GeoObjectsManagerPtr geoObjectsManager)
        : mGeoObjectsManager(geoObjectsManager), mVisible(true)
    {
    }

    QStringList RegionsLayer::renderPosition() const
    {
        return QStringList() << "HOVERS_ABOVE_SURFACE";
    }

    bool RegionsLayer::render(GeoPainter *painter, ViewportParams*, const QString&, GeoSceneLayer*)
    {
        if (!mVisible)
            return true;

        OrderedPoints nodes = mGeoObjectsManager->orderedPoints(PlacemarkPolygonNode);
        if (nodes.size() < 2)   //There is no area to draw for a 1-point polygon
            return true;

        painter->save();
        painter->setRenderHint(QPainter::Antialiasing, true);
        painter->setOpacity(0.8);

        QBrush brush(Oxygen::brickRed2, Qt::Dense1Pattern);
        QPen pen(brush, 3, Qt::DashLine);
        painter->setPen(pen);
        painter->setBrush(brush);

        GeoDataLinearRing nodesPath(Tessellate);
        GeoDataCoordinates firstNode(0, 0);
        bool first = true;
        foreach (int num, nodes.keys())
        {
           PlacemarkPtr placemark = nodes.value(num);
           MapObjectPtr mapObject = mGeoObjectsManager->getMapObjectForPlacemark(placemark);
           GeoCoords coord = mapObject->coords();
           GeoDataCoordinates marbleCoords = MapLibraryHelpers::transformCoords(coord);
           if (first)
           {
               firstNode = marbleCoords;
               first = false;
           }
           nodesPath << marbleCoords;
        }
        if (nodesPath.size() > 1)
        {
            nodesPath << firstNode;
            painter->drawPolygon(nodesPath);
        }
        painter->restore();
        return true;
    }
}

#ifndef LOCALMAPLOADER_H
#define LOCALMAPLOADER_H

#include <QObject>
#include <QString>
#include <QPointF>
#include <QSharedPointer>
#include "GeoCoords.h"
#include "Orientation.h"

class RoboticsMap;

namespace MapAbstraction
{
    class LocalMapLoader : public QObject
    {
    Q_OBJECT
    signals:
        void localMapLoaded(QString newLocalMap, qreal resolution,
                            MapAbstraction::GeoCoords centerApprox, MapAbstraction::Orientation rotation, QPointF origin);

    public:
        LocalMapLoader(RoboticsMap *map);

    private:
        void readConfig();
    };
    typedef QSharedPointer<LocalMapLoader> LocalMapLoaderPtr;
}

#endif // LOCALMAPLOADER_H

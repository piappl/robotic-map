#ifndef IGEOMAP_H
#define IGEOMAP_H

#include <QSharedPointer>
#include "IGeoMapFwd.h"
#include "IMapSignalReceiver.h"
#include "GeoMapSender.h"
#include "GeoLocalMapReceiver.h"

namespace MapAbstraction
{
    class IGeoMap : public IMapSignalReceiver
    {
    public:
        virtual ~IGeoMap() {}
        virtual GeoLocalMapReceiverPtr localMapReceiver() const = 0;
        virtual GeoMapSenderPtr sender() const = 0;
    };
}

#endif // IGEOMAP_H

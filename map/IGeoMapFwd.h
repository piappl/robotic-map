#ifndef IGEOMAPFWD_H
#define IGEOMAPFWD_H

#include <QSharedPointer>

namespace MapAbstraction
{
    class IGeoMap;
    class ISensorReadings;

    typedef QSharedPointer<IGeoMap> IGeoMapPtr;
    typedef QSharedPointer<const IGeoMap> IGeoMapConstPtr;

    typedef QSharedPointer<ISensorReadings> ISensorReadingsPtr;
}

#endif // IGEOMAPFWD_H

#ifndef IGEOMAPFWD_H
#define IGEOMAPFWD_H

#include <QSharedPointer>

namespace MapAbstraction
{
    class IGeoMap;

    typedef QSharedPointer<IGeoMap> IGeoMapPtr;
    typedef QSharedPointer<const IGeoMap> IGeoMapConstPtr;
}

#endif // IGEOMAPFWD_H

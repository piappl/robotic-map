#include <QMutex>
#include "GeoObjectID.h"

namespace
{
    int sNextId = 1;
    QMutex sLock;
}

bool IntIdentifier::isNull() const { return intId() == null(); }
int IntIdentifier::null() { return 0; }
bool IntIdentifier::operator==(const IntIdentifier &other) const { return intId() == other.intId(); }
bool IntIdentifier::operator<(const IntIdentifier &other) const { return intId() < other.intId(); }
int IntIdentifier::intId() const { return mID; }
QString IntIdentifier::toString() const { return QString::number(mID); }
IntIdentifier::IntIdentifier() { mID = null(); }
IntIdentifier::IntIdentifier(int id) { mID = id; }

IntIdentifier IntIdentifier::createIntIdentifier()
{
    int id;
    sLock.lock();
        id = sNextId++;
    sLock.unlock();
    return IntIdentifier(id);
}

IntIdentifier IntIdentifier::fromInt(int id)
{
    return IntIdentifier(id);
}

GeoObjectID GeoReferenceFactory::createGeoObjectId()
{
    return IntIdentifier::createIntIdentifier();
}

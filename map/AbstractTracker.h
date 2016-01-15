#ifndef ABSTRACTTRACKER_H
#define ABSTRACTTRACKER_H

#include "MapLibraryTypes.h"

class AbstractTracker : public QObject
{
Q_OBJECT

signals:
    void boxChanged(const Marble::GeoDataLatLonBox &box);

public:
    virtual ~AbstractTracker() {}
    void trackerUpdate();

private:
    virtual Marble::GeoDataLatLonBox calculateBox() const = 0;
    //TODO - if required, post only changes: GeoDataLatLonBox mLastBox;
};

#endif // ABSTRACTTRACKER_H

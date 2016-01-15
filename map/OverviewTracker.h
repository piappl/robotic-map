#ifndef OVERVIEWTRACKER_H
#define OVERVIEWTRACKER_H
#include <QSharedPointer>
#include "AbstractTracker.h"
#include "InternalTypesFwd.h"

class OverviewTracker : public AbstractTracker
{
public:
    OverviewTracker(PlacemarkLogicPtr logic);
private:
    Marble::GeoDataLatLonBox calculateBox() const;
    PlacemarkLogicPtr mPlacemarkLogic; //Non-owning
};

#endif // OVERVIEWTRACKER_H

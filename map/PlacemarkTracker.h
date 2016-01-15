#ifndef PLACEMARKTRACKER_H
#define PLACEMARKTRACKER_H

#include "AbstractTracker.h"

class PlacemarkTracker : public AbstractTracker
{
public:
    PlacemarkTracker(PlacemarkConstPtr placemark);

private:
    Marble::GeoDataLatLonBox calculateBox() const;
    PlacemarkConstPtr mPlacemark;
};

#endif // PLACEMARKTRACKER_H

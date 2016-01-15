#include "OverviewTracker.h"
#include "PlacemarkLogic.h"
using namespace Marble;

OverviewTracker::OverviewTracker(PlacemarkLogicPtr logic) : mPlacemarkLogic(logic)
{
}

GeoDataLatLonBox OverviewTracker::calculateBox() const
{
    return mPlacemarkLogic->allPlacemarksGeoRect();
}

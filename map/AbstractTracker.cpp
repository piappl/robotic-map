#include "AbstractTracker.h"
using namespace Marble;

void AbstractTracker::trackerUpdate()
{
     GeoDataLatLonBox box = calculateBox();
     emit boxChanged(box);
}

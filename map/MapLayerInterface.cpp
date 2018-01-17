#include "MapLayerInterface.h"
#include "RoboticsMap.h"

using namespace MapAbstraction;
using namespace Marble;

MapLayerInterface::MapLayerInterface(RoboticsMap *rm)
{
    connect(this, SIGNAL(requestUpdate()), rm, SLOT(updateRequested()));
}

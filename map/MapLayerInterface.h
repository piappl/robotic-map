#ifndef MAPLAYERINTERFACE_H
#define MAPLAYERINTERFACE_H

#include <QObject>
#include <LayerInterface.h>
#include <MarbleMap.h>
#include <map>
#include <memory>

class RoboticsMap;
namespace MapAbstraction
{
    enum LayerType
    {
        LayerLocalMap,
        LayerCrosshair,
        LayerDynamicObjects,
        LayerLaserCloud,
        LayerRegions,
        LayerRobotManualPlacement,
        LayerSensorData,
        LayerPaths,
        LayerWaypoints
    };

    class MapLayerInterface : public QObject, public Marble::LayerInterface
    {
    Q_OBJECT
    signals:
        void requestUpdate();

    public:
        MapLayerInterface(RoboticsMap *rm);
        virtual void setVisible(bool v) { mVisible = v; emit requestUpdate(); }
        bool visible() const { return mVisible; }
        virtual LayerType type() const = 0;

    private:
        bool mVisible = false;
    };
    typedef QSharedPointer<MapLayerInterface> MapLayerInterfacePtr;
}

#endif // MAPLAYERINTERFACE_H

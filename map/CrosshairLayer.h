#ifndef MARBLECROSSHAIRLAYER_H
#define MARBLECROSSHAIRLAYER_H

#include <marble/LayerInterface.h>

namespace Marble
{
    class CrosshairLayer : public LayerInterface
    {
    public:
        CrosshairLayer();
        QStringList renderPosition() const;
        bool render(GeoPainter *painter, ViewportParams *viewport, const QString &renderPos = "NONE", GeoSceneLayer *layer = 0);
        void setVisibility(bool visible);
        bool visible() const;

    private:
        bool mVisible;
    };
}

#endif // MARBLECROSSHAIRLAYER_H

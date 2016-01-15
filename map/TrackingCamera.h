#ifndef TRACKINGCAMERA_H
#define TRACKINGCAMERA_H

#include <QTimer>
#include "InternalTypesFwd.h"
#include "MapLibraryTypes.h"
#include "GeoObjectID.h"

class TrackingCamera : public QObject
{
Q_OBJECT
signals:
    void cameraUpdate(const Marble::GeoDataLatLonBox& box);

public:
    TrackingCamera();
    void TrackPlacemark(const GeoObjectID &id, PlacemarkConstPtr placemark);
    void TrackOverview(PlacemarkLogicPtr logic);
    void toggleTracking(bool on);

private slots:
    void onTimerExpired();

private:
    void replaceTracker(AbstractTracker *tracker);

    //For optimization only
    GeoObjectID mLastPlacemarkID;
    bool mIsTrackingOverview;

    bool mOn;
    QTimer mCameraUpdateTimer;
    AbstractTrackerPtr mTracker;
};

#endif // TRACKINGCAMERA_H

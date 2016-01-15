#include "TrackingCamera.h"
#include "PlacemarkLogic.h"
#include "AbstractTracker.h"
#include "OverviewTracker.h"
#include "PlacemarkTracker.h"

namespace
{
    const int updateIntervalMs = 1000;
}

TrackingCamera::TrackingCamera() : mIsTrackingOverview(false), mOn(false)
{
    mCameraUpdateTimer.setSingleShot(false);
    mCameraUpdateTimer.setInterval(updateIntervalMs);
    connect(&mCameraUpdateTimer, SIGNAL(timeout()), this, SLOT(onTimerExpired()));
}

void TrackingCamera::toggleTracking(bool on)
{
    if (mOn == on)
        return;

    mOn = on;
    if (on)
    {
        if (!mCameraUpdateTimer.isActive())
            mCameraUpdateTimer.start();
    }
    else
    {
        mCameraUpdateTimer.stop();
    }
}

void TrackingCamera::TrackOverview(PlacemarkLogicPtr logic)
{
    if (mIsTrackingOverview)
        return; //already tracking overview

    AbstractTracker *tracker = new OverviewTracker(logic);
    GeoObjectID emptyID;
    mLastPlacemarkID = emptyID;
    mIsTrackingOverview = true;
    replaceTracker(tracker);
}

void TrackingCamera::TrackPlacemark(const GeoObjectID &id, PlacemarkConstPtr placemark)
{
    if (mLastPlacemarkID == id)
        return; //already tracking this object

    AbstractTracker *tracker = new PlacemarkTracker(placemark);
    mLastPlacemarkID = id;
    mIsTrackingOverview = false;
    replaceTracker(tracker);
}

void TrackingCamera::onTimerExpired()
{
    mTracker->trackerUpdate();
}

void TrackingCamera::replaceTracker(AbstractTracker *tracker)
{
    mTracker.reset(tracker);
    connect(tracker, SIGNAL(boxChanged(Marble::GeoDataLatLonBox)),
            this, SIGNAL(cameraUpdate(Marble::GeoDataLatLonBox)));
}

#include "MapLogPlacemarkData.h"
#include <QMap>

namespace
{
    const int msecsInDay = 1000 * 60 * 60 * 24;
    const int limit = 100;  //can go over this limit with all points being critical
}

namespace MapAbstraction
{
    SinglePointData::SinglePointData(GeoCoords coords)
        : mTime(QDateTime::currentDateTime()), mDurationMs(0), mCoords(coords)
    {
    }

    GeoCoords SinglePointData::coords() const
    {
        return mCoords;
    }

    QDateTime SinglePointData::time() const
    {
        return mTime;
    }

    qint64 SinglePointData::durationMs() const
    {
        return mDurationMs;
    }

    void SinglePointData::advanceDurationMs(qint64 msToAdd)
    {   //doesn't handle overflow here, assumes that time res will be much less then qint64 max
        mDurationMs+=msToAdd;
    }

    bool SinglePointData::operator==(const SinglePointData &other) const
    {
        return coords() == other.coords();
    }

    bool SinglePointData::isCritical() const
    {   //Critical data is not subject to ignoring or compressing.
        return false; //No data now is critical
    }

    MapLogPlacemarkData::MapLogPlacemarkData(int timeResolutionMs, ExtraDataBehavior behavior)
        : mTimeResolution(timeResolutionMs), mExtraDataBehavior(behavior)
    {
    }

    void MapLogPlacemarkData::addData(const GeoObjectID &id, SinglePointDataPtr data)
    {
        if (!mData.contains(id))
        {
            QVector<SinglePointDataPtr> newHistory;
            newHistory.push_back(data);
            mData.insert(id, newHistory);
            return;
        }

        if (data->isCritical())
        {
            insert(id, data);
        }

        SinglePointDataPtr mostRecent = mData[id].last();

        QDateTime lastTime = mostRecent->time();
        QDateTime recently = lastTime.addMSecs(mostRecent->durationMs());
        QDateTime now = QDateTime::currentDateTime();
        qint64 timespan = recently.daysTo(now) * msecsInDay + recently.time().msecsTo(now.time());

        if (*data == *mostRecent)
        {   //ignore, but advance duration
            //qDebug("Data didn't change");
            mostRecent->advanceDurationMs(timespan);
            return;
        }

        if (timespan > mTimeResolution)
        {
            //qDebug("Data changed and fits in resolution, inserting");
            insert(id, data);
        }
        else
        {
            //qDebug("Data discarded");
            if (mExtraDataBehavior == MapLogPlacemarkData::Ignore)
            { //ignore data completely
                return;
            }
            else
            { //TODO compress data somehow (for now also ignore)
                return;
            }
        }
    }

    void MapLogPlacemarkData::insert(const GeoObjectID &id, SinglePointDataPtr data)
    {
        if (!mData.contains(id))
            return;
        if (mData[id].size() > limit)
        {
            //compressData(id);
            //simple:
            mData[id].removeFirst();
        }
        mData[id].push_back(data);
    }

    void MapLogPlacemarkData::compressData(const GeoObjectID &id)
    {   //TODO - real smart compression depending on type of data
        //Coordinates should be compressed with importance heuristics
        //Direction changes should be preserved rather than same-line points
        const int rate = 2;
        ObjectDataHistory history = mData[id];
        ObjectDataHistory::iterator it;
        ObjectDataHistory rewrittenHistory;
        int count = 1;  //always take first
        qint64 accumulatedTime = 0;
        for (it = history.begin(); it != history.end(); ++it)
        {
            SinglePointDataPtr point = *it;
            bool critical = point->isCritical();
            if (count == 1 || critical)
            {   //Take this element
                point->advanceDurationMs(accumulatedTime);
                rewrittenHistory.push_back(point);
                count = rate;
                accumulatedTime = 0;
            }
            else
            {
                accumulatedTime += point->durationMs();
                count--;
            }
        }
        mData[id] = rewrittenHistory;
    }

    ObjectDataHistory MapLogPlacemarkData::getData(const GeoObjectID &id) const
    {
        Q_ASSERT(mData.contains(id));
        return mData.value(id);
    }

    QList<GeoObjectID> MapLogPlacemarkData::recordedObjects() const
    {
        return mData.keys();
    }
}

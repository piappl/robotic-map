#ifndef MAPLOGPLACEMARKDATA_H
#define MAPLOGPLACEMARKDATA_H

#include <QDateTime>
#include <QVector>
#include <QMap>
#include <QSharedPointer>
#include "GeoCoords.h"
#include "GeoObjectID.h"

namespace MapAbstraction
{
    class SinglePointData
    {   //TODO - maybe it should be LocalizedObjectData (consistency)
    public:
        SinglePointData(GeoCoords coords);
        bool operator==(const SinglePointData &other) const;
        bool isCritical() const;

        GeoCoords coords() const;
        QDateTime time() const;

        qint64 durationMs() const;
        void advanceDurationMs(qint64 msToAdd);

    private:
        QDateTime mTime;
        qint64 mDurationMs; //How long this data is valid
        GeoCoords mCoords;
    };

    typedef QSharedPointer<SinglePointData> SinglePointDataPtr;
    typedef QVector<SinglePointDataPtr> ObjectDataHistory;

    class MapLogPlacemarkData
    {
    public:
        enum ExtraDataBehavior
        {   //TODO: Not working yet - always ignores
            Compress,
            Ignore
        };

        MapLogPlacemarkData(int timeResolutionMs = 1500,
                            ExtraDataBehavior behavior = Ignore);
        void addData(const GeoObjectID &id, SinglePointDataPtr data);
        QList<GeoObjectID> recordedObjects() const;
        ObjectDataHistory getData(const GeoObjectID &id) const;
        //TODO - data read/serialization api

    private:
        void insert(const GeoObjectID &id, SinglePointDataPtr data);
        void compressData(const GeoObjectID &id);

        int mTimeResolution;
        ExtraDataBehavior mExtraDataBehavior;

        typedef QMap<GeoObjectID, ObjectDataHistory> ObjectsData;
        ObjectsData mData;
    };

}

#endif // MAPLOGPLACEMARKDATA_H

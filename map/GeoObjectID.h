#ifndef GEOOBJECTID_H
#define GEOOBJECTID_H

#include <QString>

class IntIdentifier
{
public:
    static IntIdentifier createIntIdentifier();
    static IntIdentifier fromInt(int id);
    bool isNull() const;
    bool operator==(const IntIdentifier &other) const;
    bool operator<(const IntIdentifier &other) const;
    int intId() const;
    QString toString() const;
    IntIdentifier();
private:
    static int null();
    IntIdentifier(int id);

    int mID;
};

typedef IntIdentifier GeoObjectID;

class GeoReferenceFactory
{
public:
    static GeoObjectID createGeoObjectId();
};



#endif // GEOOBJECTID_H

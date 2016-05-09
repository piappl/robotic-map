#include "peoplesubscriber.h"
#include <string>

using namespace MapAbstraction;

PeopleSubscriber::PeopleSubscriber(int robotID, QSharedPointer<ros::NodeHandle> handle)
    : mID(robotID)
{
    QString subscriberTopic = "/robot/" + QString::number(robotID) + "/pedestrians";
    mSub = handle->subscribe<people_msgs::People>(
                subscriberTopic.toStdString(), 100, &PeopleSubscriber::peopleCallback, this);
}

void PeopleSubscriber::peopleCallback(const people_msgs::People::ConstPtr &msg)
{
    int peopleCount = msg->people.size();
    DynamicObjects people;
    for (int i = 0; i < peopleCount; ++i)
    {
        people_msgs::Person person = msg->people.at(i);
        DynamicObject mapPerson;
        QString name(person.name.c_str());
        mapPerson.name = name;
        mapPerson.localizationType = MapObject::LocalRelative;
        GeoCoords coords(person.position.x, person.position.y);
        mapPerson.position = coords;
        mapPerson.velocityX = person.velocity.x;
        mapPerson.velocityY = person.velocity.y;
        mapPerson.reliability = person.reliability;
        mapPerson.objectType = DynamicObjectPedestrian;

        people.append(mapPerson);
        //TODO - support tags
    }

    if (people.empty())
        return;

    qDebug("Emitting people");
    emit peopleUpdate(mID, people);
}

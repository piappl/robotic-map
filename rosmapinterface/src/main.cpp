/*
 * A simple ros node for robotic map interface
 * Author: Adam Dąbrowski, PIAP (www.piap.pl), adabrowski@piap.pl
 */

#include <QtQuick/QQuickView>
#include <QQuickItem>
#include <QQmlEngine>
#include <QFileInfo>
#include <QApplication>
#include <QTimer>
#include <QMap>

#include <map/MapWrap.h>
#include <map/MapRobotObject.h>
#include <map/MapWaypointObject.h>
#include <map/GeoMapSender.h>
#include <map/DynamicObject.h>

//#include <std_msgs/String.h>
//#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "rosmapinterface/WaypointInformation.h"
#include "robotlistener.h"

//TODO - this is prototype code, no structure yet

using namespace MapAbstraction;

namespace
{
    const int consoleID = 1331;
}

class RosWaypointSender : public RobotIDListener
{
    Q_OBJECT
    signals:
        void robotConnected(int robot, bool connected);

    public:
        RosWaypointSender(MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle)
            : mRosHandle(handle), mMapWrap(mapWrap)
        {
            connectSignals();
        }

    public slots:
        void waypointsReceived(int robotID, MapAbstraction::MapPath waypoints)
        {
            if (waypoints.size() < 1)
                return;

            onRobotDetected(robotID);
            qDebug("Received %d waypoints for robot %d", waypoints.size(), robotID);
            rosmapinterface::WaypointInformation msg;
            MapWaypointObjectPtr waypoint = waypoints.first();
            msg.waypoint_name = ""; //"Go there now"
            msg.x = waypoint->coords().longitude();
            msg.y = waypoint->coords().latitude();
            msg.theta = 0;

            mPublishersForRobots.value(robotID)->publish(msg);
        }

        void onRobotDetected(int robotID)
        {
            if (!mPublishersForRobots.contains(robotID))
            {
                qDebug("Robot %d detected", robotID);
                QString topic = "/robot/" + QString::number(robotID) + "/waypoint_information";
                ros::Publisher pub = mRosHandle->advertise<rosmapinterface::WaypointInformation>(
                            topic.toStdString(), 5);
                qDebug("Advertising topic %s", qPrintable(topic));
                mPublishersForRobots.insert(robotID, QSharedPointer<ros::Publisher>(new ros::Publisher(pub)));
            }
        }

    private slots:
        void onConnectionRequest(int robotID, bool connected)
        {   //For connection oriented robots, some logic could be here (i.e negotiate connection,
            //disconnect from previous robot)
            emit robotConnected(robotID, connected);
        }

    private:
        void connectSignals()
        {
            qRegisterMetaType<MapWaypointObjectPtr>("MapWaypointObjectPtr");
            qRegisterMetaType<MapAbstraction::MapPath>("MapPath");
            connect(mMapWrap->sender().data(), SIGNAL(mapPathCreated(int,MapAbstraction::MapPath)),
                    this, SLOT(waypointsReceived(int,MapAbstraction::MapPath)),
                    Qt::QueuedConnection);

            connect(mMapWrap->sender().data(), SIGNAL(requestConnect(int,bool)),
                    this, SLOT(onConnectionRequest(int,bool)),
                    Qt::QueuedConnection);

            connect(this, SIGNAL(robotConnected(int, bool)),
                    mMapWrap.data(), SLOT(robotConnected(int, bool)),
                    Qt::QueuedConnection);
        }

        QMap<int, QSharedPointer<ros::Publisher> > mPublishersForRobots;
        QSharedPointer<ros::NodeHandle> mRosHandle;
        MapWrapPtr mMapWrap;
};

class MapQuickItem : public QQuickView
{
    Q_OBJECT
public:
    MapQuickItem(int argc, char *argv[])
    {
        ros::init(argc, argv, "rosmapinterface");
    }

    void start()
    {
        while (!ros::master::check())
        {
            qDebug("Waiting for ros master");
            sleep(1);
        }

        mHandle.reset(new ros::NodeHandle());

        setSource(QUrl("qrc:/wrap.qml"));
        QQuickItem *viewMapItem = rootObject();
        mMapWrap.reset(new MapWrap(viewMapItem, consoleID));
        if(status()!=QQuickView::Ready)
            qDebug("can't initialise view");

        QSurfaceFormat format;
        format.setAlphaBufferSize(8);
        setFormat(format);
        setClearBeforeRendering(true);
        setColor(QColor(Qt::transparent));
        setTitle("Ros map interface");

        show();

        mSender.reset(new RosWaypointSender(mMapWrap, mHandle));
        RobotListener::start(mMapWrap, mHandle, mSender.data());

        QTimer *checker = new QTimer(this);
        connect(checker, SIGNAL(timeout()), this, SLOT(checkMaster()));
        checker->setSingleShot(false);
        checker->start(1000);
    }

private slots:
    void checkMaster()
    {
        if (!ros::ok())
        {
            qWarning("Ros not ok, shutting down (1)");
            exit(1);
        }
    }

private:
    bool event(QEvent *e)
    {
        if (e->type() == QEvent::Close)
        {
            qDebug("shutting down (0)");
            ros::shutdown();
            exit(0);
        }
        return QQuickView::event(e);
    }

    QSharedPointer<ros::NodeHandle> mHandle;
    MapWrapPtr mMapWrap;
    QSharedPointer<RosWaypointSender> mSender;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    qRegisterMetaType<MapAbstraction::MapRobotObjectPtr>("MapAbstraction::MapRobotObjectPtr");
    qRegisterMetaType<MapAbstraction::LaserScanPoints>("MapAbstraction::LaserScanPoints");
    qRegisterMetaType<MapAbstraction::DynamicObjects>("MapAbstraction::DynamicObjects");

    MapQuickItem node(argc, argv);
    node.start();
    return app.exec();
}

#include "main.moc"

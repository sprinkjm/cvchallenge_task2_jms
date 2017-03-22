// Author: Jonathan Sprinkle
// Copyright (c) 2017 Arizona Board of Regents
// License: BSD
//
// This (very simple) node reads a laser scan, and 
// groups nearby points into a geometry_msgs/PolygonStamped
// message. Multiple messages are published, if at least 3
// points can be grouped into a polygon.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include <vector>
#include <cassert>
#include <cstdio>
#include <cstdlib>

// how far away must points be in order to be considered part of the same
// object? TODO: change this to a ROS parameter
#define CONTIGUOUS 1.0

// we collect messages here, in the callback function
std::vector<geometry_msgs::PolygonStamped> detections_msgs;
// Store the minimum distance to consider (a ROS parameter)
double dist_min;
// Store the maximum distance to consider (a ROS parameter)
double dist_max;
// frame id in which we should publish the PolygonStamped message
// e.g., if points are collected in the laser frame, use that frame;
// TODO: if you want to publish into a different frame, you should subscribe to a 
// tf topic that publishes that frame of reference, and do the transform in
// this code before you publish it.
std::string frame_id;

// start with this sequence number for what we produce
int seq=1;

// The enclosed ranges and angles should correspond to points that must be
// converted into a PolygonStamped message. This function performs the math
// to convert to cartesian coordinates, and adds each point into the queue.
// Notes: 
//   * the header and other information for the message is updated just before publishing
//   * at least 3 points are needed or else the function returns
void addToPublisherQueue(std::vector<float> ranges, std::vector<float> angles) {
    // create a polygonstamped msg and put it into the detections queue
//    ROS_DEBUG_STREAM("Found " << angles.size() << " points");
    if( angles.size() < 3 ) {
        ROS_DEBUG_STREAM("Skipping the add to publisher queue, polygon has less than 3 sides.");
        return;
    }
    ROS_INFO_STREAM("Adding " << ranges.size() << " points to a polygon" );
    geometry_msgs::PolygonStamped msg;
    assert(ranges.size() == angles.size());
    // reserve the size we know we need
    msg.polygon.points.reserve(ranges.size());
    // fill out the header when we send it
    // set the points of the polygon, using the points we have collected
    for( int ii=0; ii<ranges.size(); ii++ )
    {
        geometry_msgs::Point32 point;
        float range = ranges[ii];
        float angle = angles[ii];
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0.0f;
        msg.polygon.points.push_back(point);
    }
    // and now add this msg to the queue that we will publish
    detections_msgs.push_back(msg);
}

// This very simple callback looks through the data array, and then
// adds any contiguous points into the publisher queue
void scanCallback( const sensor_msgs::LaserScan::ConstPtr& scan )
{
    // each time we're called, we zero out the vector; this way, if we
    // receive 10 scans for each publish we do, only the most recent
    // scan is published
    detections_msgs.clear();

    // the data from the rangefinder is used to calculate the angle of
    // the index in the scan, based on the data in the message
    // for instance, the distance returned at index [13] will be
    // at an angle of (angle_min + 13*angle_increment)
    float angle_tmp=scan->angle_min;
    float inc = scan->angle_increment;
    // these vectors store contiguous values; each time we see that we have
    // a complete set, we will reset these vectors
    std::vector<float> contiguousAngles;
    std::vector<float> contiguousRanges;
    // iterate over the entire scan
    for( std::vector<float>::const_iterator it=scan->ranges.begin();
        it!=scan->ranges.end(); it++, angle_tmp=angle_tmp+inc )
    {
        // only look at this value if it is valid within desired ranges
        if( *it > scan->range_min && *it < scan->range_max )
        {
            // only look at this value if the distance is within our parameter
            if( *it < dist_max )
            {
                // initialize this value, which will help us know how close we are to 
                // the previous valid index
                float prev_range = 0.0f;
                // if we have a set of contiguous points, look at the last one
                if( contiguousRanges.size() > 0 )
                {
                    prev_range = contiguousRanges[contiguousRanges.size()-1];
                }
                // this signals that we might be finding a new obstacle
                if( contiguousAngles.size() == 0 )
                {
                    contiguousAngles.push_back(angle_tmp);
                    contiguousRanges.push_back(*it);
                }
                // this tells whether this item is contiguous to the previous
                else if( fabs(prev_range - *it) < CONTIGUOUS )
                {
                    contiguousAngles.push_back(angle_tmp);
                    contiguousRanges.push_back(*it);
                }
                // the next point is too far away, so we publish what we have now,
                // and then add this point to an otherwise empty set of contiguous items
                else
                {
                    addToPublisherQueue(contiguousRanges, contiguousAngles);
                    contiguousRanges.clear();
                    contiguousAngles.clear();
                    contiguousRanges.push_back(*it);
                    contiguousAngles.push_back(angle_tmp);
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Distance of " << *it << " is too far away (dist_max=" << dist_max << ")");
            }
        }
        else
        {
            // I don't expect we will ever see this pop up, but...
            ROS_DEBUG_STREAM("Distance of " << *it << " is not within max/min ranges of the sensor");
        }
    }
    // it could happen that we finish our scan with a contiguous range of values
    if( contiguousAngles.size() > 2 )
    {
        ROS_DEBUG_STREAM("Adding to publisher queue at the end of the cycle");
        addToPublisherQueue(contiguousRanges, contiguousAngles);
    }
}

// right before publishing, let's add the current time and frame to the msg
void updateHeaders(std::vector<geometry_msgs::PolygonStamped>& msgs) 
{
    ros::Time nowTime = ros::Time::now();
    for( int i=0; i<msgs.size(); i++ )
    {
        geometry_msgs::PolygonStamped msg = msgs[i];
        msg.header.stamp = nowTime;
        msg.header.seq = seq++;
        msg.header.frame_id = frame_id;
        msgs[i] = msg;
    }
}

/*
    The main function is largely based on the standard ROS tutorial for CPP programs
*/
int main( int argc, char **argv )
{
    std::string scan_topic;
    std::string detections_topic;

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "SimpleSolution");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
//	ros::NodeHandle n;
	// set up the handle for this node, in order to publish information
	// the node handle is retrieved from the parameters in the .launch file,
	// but we have to initialize the handle to have the name in that file.
	ros::NodeHandle n("~");

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
	n.param("detections_topic", detections_topic, std::string("/detections"));
	n.param("frame_id", frame_id, std::string("catvehicle/front_laser_link"));
	n.param("scan_topic", scan_topic, std::string("/catvehicle/front_laser_points"));
	n.param("dist_min", dist_min, 1.0); // only consider returns larger than this
	n.param("dist_max", dist_max, 40.0);// skip data larger than this

    ROS_INFO_STREAM("Node namespace is " << ros::this_node::getNamespace());
    ROS_INFO_STREAM("Node name is " << ros::this_node::getName( ) );

    // publish the polygons we find to this topic: up to 50 could be published at a time
  	ros::Publisher detections_pub = n.advertise<geometry_msgs::PolygonStamped>(detections_topic, 50);

  	// we also want to subscribe to the scanner; we don't look through old scans, so making the queue 1
  	ros::Subscriber sub = n.subscribe(scan_topic, 1, &scanCallback);

    ROS_INFO_STREAM("Looking for scan in topic " << scan_topic);
    ROS_INFO_STREAM("Publishing detections to topic " << detections_topic);

  	// publish polygons at 1Hz
  	ros::Rate loop_rate(1);

	while( ros::ok() )
	{

		ros::spinOnce( );

		ROS_DEBUG_STREAM( "Have " << detections_msgs.size() << " detections" );
        // make a copy, to prevent any race conditions later
        std::vector<geometry_msgs::PolygonStamped> vectorToPublish = detections_msgs;
        // update the headers
        updateHeaders(vectorToPublish);
  		ROS_INFO_STREAM( "Publishing " <<vectorToPublish.size() << " detections" );
        // this logic "spreads" out the detections in time, so they are easier to see in RViz
        ros::Duration sleepPeriod((1.0-0.2)/(vectorToPublish.size()));
        for( int ii=0; ii<vectorToPublish.size(); ii++ )
        {
    		detections_pub.publish(vectorToPublish[ii]);
            sleepPeriod.sleep();
        }
        // wait until the rest of the time is up
		loop_rate.sleep( );

	}

	return EXIT_SUCCESS;
}


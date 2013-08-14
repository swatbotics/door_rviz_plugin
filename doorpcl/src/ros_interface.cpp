#include "door_finder/edge_detector.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>


static EdgeDetector * detector;
static ros::Publisher pub;
static image_transport::Publisher image_pub;
static ros::Publisher marker_pub;
static ros::Publisher drawpoints_pub;
static visualization_msgs::Marker points, line_list;
static std::vector<geometry_msgs::Point> handle_points;
 
void publishImage(cv::Mat & mat);

void doorpoint_cb(const geometry_msgs::Point& input)
{
    std::cout << "sending a door point to edge detector" << std::endl;
    detector->doorMouseClick(input.x, input.y);
    if (detector->drawPoints.size() == 4 )
    {
        geometry_msgs::Polygon poly_out;
        for (int i = 0; i < 4; i++)
        {
            geometry_msgs::Point32 p;
            p.x = detector->drawPoints.at(i).x();
            p.y = detector->drawPoints.at(i).y();
            poly_out.points.push_back(p);
        }
        drawpoints_pub.publish(poly_out);
    }
}

void handlepoint_cb(const geometry_msgs::Point& input)
{
    if (handle_points.size() < 2)
    {
        handle_points.push_back(input);
        std::cout << "adding a handle point" <<std::endl;
    }
    if (handle_points.size() == 2)
    {
        std::cout << "sending handle points to edge detector" << std::endl;
        detector->handle0[0] = handle_points.at(0).x;
        detector->handle0[1] = handle_points.at(0).y;
        detector->handle1[0] = handle_points.at(1).x;
        detector->handle1[1] = handle_points.at(1).y;
        detector->getHandlePoints();
        handle_points.clear();
    }
    std::cout << "there are " << handle_points.size() << " handle points" << std::endl;
}


void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    cout << "entered cloud callback" << endl;

    points.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "points_and_lines";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    line_list.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    line_list.scale.x = 0.1;

    points.color.g = 1.0f;
    points.color.a = 1.0;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;


    //sensor_msgs::PointCloud2 output;
    EdgeDetector::PointCloud::Ptr cloud (new EdgeDetector::PointCloud );
    pcl::fromROSMsg (*input, *cloud);
    detector->inputPointCloud( cloud, true );
    cout << "exited edge_detector" << endl; 
    /*
    std::vector<PlaneSegmenter::LinePosArray> plane_lines = detector->planarLines;
    cout << "found planar lines" << endl; 
    
    points.points.clear();
    line_list.points.clear();
    
    for ( int i = 0; i < plane_lines.size(); i++ )
    {
        for ( int j = 0; j < plane_lines.at(i).size(); j++ )
        {
            geometry_msgs::Point p;
            p.x = plane_lines.at(i).at(j).x;
            p.y = plane_lines.at(i).at(j).y;
            p.z = plane_lines.at(i).at(j).z;

            points.points.push_back(p);
            line_list.points.push_back(p);
        }
    }
    */
    /*
    if ( detector->doorPoints.size() == 4 )
    {
        for (int j = 0; j < 4; j++ )
        {
            geometry_msgs::Point t;
            t.x = detector->doorPoints[j].x;
            t.y = detector->doorPoints[j].y;
            t.z = detector->doorPoints[j].z;

            points.points.push_back(t);
        }
    }
    */

    //pub.publish (output);
    publishImage(detector->displayImage);
    //cout << "number of points: " << points.points.size() << endl;
    //marker_pub.publish(points);
    //marker_pub.publish(line_list);

}   

void publishImage( cv::Mat & mat ){
    cout << "entered publish image function" << endl; 
    if (mat.data != NULL ){
        cout << "displayImage contains data" << endl;
        cv_bridge::CvImage out_msg;
        //const IplImage * iplimg = new IplImage(mat);
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = "rgb8";
        out_msg.image = mat;
        image_pub.publish (out_msg.toImageMsg());

        //image_pub.publish( iplimg.toImageMsg() );
        cout << "published image" << endl;
    }

}

int main (int argc, char** argv)
{
    std::string configFileName;
    //configFileName = "~/config/edgeDetectorConfig.txt";
    configFileName ="/home/swatdrc/catkin_ws/src/doorpcl/rospkg/src/config.txt";

    //cout << "Using default config file: " << configFileName << "\n";
  
    SimpleConfig config( configFileName );
    std::string cloudInputName, lineOutputName;
    config.get( "cloudInputStream", cloudInputName);
    
    //initialize the edge detector.
    detector = new EdgeDetector( configFileName );

    // Initialize ROS
    ros::init (argc, argv, "door_finder");
    std::cout << argc << "\t" << *argv << std::endl; 
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(cloudInputName, 1, cloud_cb);
    ros::Subscriber point_sub = nh.subscribe("doorPoints", 1, doorpoint_cb);
    ros::Subscriber handle_sub = nh.subscribe("handlePoints", 1, handlepoint_cb);

    // Create a ROS publisher for the output point cloud and the lines drawn
    // by the algorithm
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>
        ("vizualization_marker", 100);
    drawpoints_pub = nh.advertise<geometry_msgs::Polygon>("drawPoints", 1);
    
    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("door/image", 1);

    // Spin
    ros::spin ();
}

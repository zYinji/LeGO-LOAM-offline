#include "utility.h"
#include "imageProjection.h"
#include "featureAssociation.h"
#include "transformFusion.h"
#include "mapOptmization.h"


int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;

    FeatureAssociation FA;

    TransformFusion TFusion;

    mapOptimization MO;


    ROS_INFO("\033[1;32m---->\033[0m LeGo Started.");


    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    /***ZHU->DOWN**************************************/
    ros::Rate rate(200);
    rosbag::Bag bag;
    bag.open("/home/yinji/catkin_ws/2018-05-18-15-25-12_36.bag", rosbag::bagmode::Read);
 
    std::vector<std::string> topics;
    topics.push_back(std::string("/imu/data"));
    topics.push_back(std::string("/velodyne_points"));
 
    rosbag::View view(bag, rosbag::TopicQuery(topics));//note:TopicQuery;TypeQuery
	

    boost::shared_ptr<sensor_msgs::Imu> IMUptr;
    boost::shared_ptr<sensor_msgs::PointCloud2> RawCloudptr;
    /***ZHU->UP**************************************/

 
    if (ros::ok())
    {
        /***ZHU->DOWN**************************************/

        // ros::Time start_time_ = view.begin()->getTime();
        // TimePublisher time_publisher_;
        // time_publisher_.setTime(start_time_);
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            ROS_INFO("FFFFFFF");
            // time_publisher_.runClock(ros::WallDuration(.1));

            ros::spinOnce();
            if(IP.SegmentedCloudFlag == true)
            {
                ROS_INFO("GOGOGO");
                FA.laserCloudHandler(IP.pubSegmentedCloudptr);
                FA.outlierCloudHandler(IP.pubOutlierCloudptr);
                FA.laserCloudInfoHandler(IP.pubSegmentedCloudInfoptr);
                IP.SegmentedCloudFlag = false;
            }
            if(FA.laserCloudInitFlag == true)
            {
                ROS_INFO("GOGOGO2");
                MO.laserCloudCornerLastHandler(FA.publaserCloudCornerLastptr);
                MO.laserCloudSurfLastHandler(FA.publaserCloudSurfLastptr);
                FA.laserCloudInitFlag = false;
            }
            if(FA.laserCloudCornerLastFlag == true)
            {
                ROS_INFO("GOGOGO3");
                MO.laserCloudCornerLastHandler(FA.publaserCloudCornerLastptr);
                MO.laserCloudSurfLastHandler(FA.publaserCloudSurfLastptr);
                MO.laserCloudOutlierLastHandler(FA.publaserCloudOutlierLastptr);
                MO.laserOdometryHandler(FA.publaserOdometryptr);
                TFusion.laserOdometryHandler(FA.publaserOdometryptr);
                FA.laserCloudCornerLastFlag = false;
            }
            if(MO.pubOdomAftMappedFlag == true)
            {
                ROS_INFO("GOGOGO4");
                TFusion.odomAftMappedHandler(MO.pubOdomAftMappedptr);
                MO.pubOdomAftMappedFlag = false;
            }
            
            FA.runFeatureAssociation();
            MO.run();
            
            sensor_msgs::Imu::ConstPtr i = m.instantiate<sensor_msgs::Imu>();
            if (i != NULL)
            {
                ROS_INFO("Pub A IMU msg");
                // sensor_msgs::Imu * IMUp = new sensor_msgs::Imu(i);
                // IMUptr.reset(IMUp);
                // FA.imuHandler(IMUptr);
                // MO.imuHandler(IMUptr);
                // i->header.stamp = ros::Time::now();
                FA.imuHandler(i);
                MO.imuHandler(i);
            }
            sensor_msgs::PointCloud2::ConstPtr r = m.instantiate<sensor_msgs::PointCloud2>();
            if (r != NULL)
            {
                ROS_INFO("Pub A PointCloud");
                // sensor_msgs::PointCloud2 * RawCloudp = new sensor_msgs::PointCloud2(r);
                // RawCloudptr.reset(RawCloudp);
                // IP.cloudHandler(RawCloudptr);
                IP.cloudHandler(r);
            }
             rate.sleep();
        }
        /***ZHU->UP**************************************/
    }
    
    bag.close();
    loopthread.join();
    visualizeMapThread.join();
    // ros::spin();
    return 0;
}

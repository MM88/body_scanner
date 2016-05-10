#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl-1.8/pcl/io/ply_io.h>
#include <pcl-1.8/pcl/io/pcd_io.h>

#include <pcl-1.8/pcl/filters/passthrough.h>
#include <pcl-1.8/pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/surface/mls.h>


////////////
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include "cloud_utils/cloud_utils.h"

using namespace pcl;
using namespace std;

#define COLOR pcl::PointXYZRGBA
#define BW pcl::PointXYZ

typedef pcl::PointCloud<COLOR> Cloud;
typedef typename Cloud::ConstPtr CloudConstPtr;

class OpenNI2Scanner
{
public:



    OpenNI2Scanner(unsigned int number) { }

    void
    cloud1_callback (const CloudConstPtr& cloud)
    {
        cout<<" ci sono 1 "<<endl;
        boost::mutex::scoped_lock lock (cloud_mutex_);
        cloud1_ = cloud;
    }

    void
    cloud2_callback (const CloudConstPtr& cloud)
    {
        cout<<" ci sono 2 "<<endl;
        boost::mutex::scoped_lock lock (cloud_mutex_);
        cloud2_ = cloud;
    }

    void
    activate_kinect1 ()
    {
        cout<<" attivo 1 "<<endl;
        boost::mutex::scoped_lock lock (grab_mutex_);
        grabber1_->start ();
        cout<<"primo grabber"<<endl;
//per kinect v2
//        grabber1_->setDepthCameraIntrinsics(364.52,364.52,259.198,212.727);
//        grabber1_->setRGBCameraIntrinsics(1081.37,1081.37,959.5,539.5);
    }

    void
    deactivate_kinect1 ()
    {
        cout<<" disattivo 1 "<<endl;
        boost::mutex::scoped_lock lock (grab_mutex_);
        grabber1_->stop ();
    }

    void
    activate_kinect2 ()
    {
        cout<<" attivo 2 "<<endl;
        boost::mutex::scoped_lock lock (grab_mutex_);
        grabber2_->start ();
        cout<<"secondo grabber"<<endl;
//per kinect v2
//        grabber2_->setDepthCameraIntrinsics(364.52,364.52,259.198,212.727);
//        grabber2_->setRGBCameraIntrinsics(1081.37,1081.37,959.5,539.5);
    }

    void
    deactivate_kinect2 ()
    {
        cout<<" disattivo 2 "<<endl;
        boost::mutex::scoped_lock lock (grab_mutex_);
        grabber2_->stop ();
    }

    void save_clouds(int cloud_index){

        std::ostringstream cloud1_name;
        cloud1_name<< "/home/miky/Scrivania/clouds/cloud1_"<<cloud_index<<".ply";

        std::ostringstream cloud2_name;
        cloud2_name<< "/home/miky/Scrivania/clouds/cloud2_"<<cloud_index<<".ply";

        boost::mutex::scoped_lock lock (cloud_mutex_); //cosi non vengono modificate mentre sta salvando

        //  f i l t e r i n g
        pcl::PointCloud<COLOR>::Ptr cloudfiltered(new pcl::PointCloud<COLOR>);
        pcl::PassThrough<COLOR> pass ;
        pass.setInputCloud(cloud1_) ;
//        pass.setFilterFieldName("z" ) ;
////        pass.setFilterLimits(0.5, 1.5);
//        pass.setFilterLimits(0.5, 1.0);
//        pass.filter(*cloudfiltered);
        pass.setFilterFieldName("z" ) ;
        pass.setFilterLimits(0.3, 1.5);
        pass.filter(*cloudfiltered);

        pass.setInputCloud(cloudfiltered) ;
        pass.setFilterFieldName("x" ) ;
        pass.setFilterLimits(-0.3, 0.3);
        pass.filter(*cloudfiltered);

        pass.setInputCloud(cloudfiltered) ;
        pass.setFilterFieldName("y" ) ;
        pass.setFilterLimits(-0.5, 0.5);
        pass.filter(*cloudfiltered);


        pcl::PointCloud<COLOR>::Ptr cloudfiltered2(new pcl::PointCloud<COLOR>);
        pcl::StatisticalOutlierRemoval<COLOR> sor;
        sor.setInputCloud (cloudfiltered);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloudfiltered2);

        cout << "Clouds saved: " << cloudfiltered2->points.size() << endl;
        cout <<"Saving cloud:"<< endl;

        pcl::io::savePLYFileBinary(cloud1_name.str(), *cloudfiltered2);

        cout << "Mesh1 saved!" << endl;

        pass.setInputCloud(cloud2_) ;
//        pass.setFilterFieldName("z" ) ;
//        pass.setFilterLimits(0.5, 1.0);
//        pass.filter(*cloudfiltered);
        pass.setFilterFieldName("z" ) ;
        pass.setFilterLimits(0.3, 1.5);
        pass.filter(*cloudfiltered);

        pass.setInputCloud(cloudfiltered) ;
        pass.setFilterFieldName("x" ) ;
        pass.setFilterLimits(-0.4, 0.4);
        pass.filter(*cloudfiltered);

        pass.setInputCloud(cloudfiltered) ;
        pass.setFilterFieldName("y" ) ;
        pass.setFilterLimits(-0.5, 0.5);
        pass.filter(*cloudfiltered);


//        pass.setInputCloud(cloudfiltered) ;
//        pass.setFilterFieldName("x" ) ;
//        pass.setFilterLimits(-0.20, 0.20);
//        pass.filter(*cloudfiltered);

//        pass.setInputCloud(cloudfiltered) ;
//        pass.setFilterFieldName("y" ) ;
//        pass.setFilterLimits(-0.15, 0.50);
//        pass.filter(*cloudfiltered);

        sor.setInputCloud (cloudfiltered);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloudfiltered2);

        cout << "Clouds saved: " << cloudfiltered2->points.size() << endl;
        cout <<"Saving cloud:"<< endl;

        pcl::io::savePLYFileBinary(cloud2_name.str(), *cloudfiltered2);

        cout << "Mesh2 saved!" << endl;

    }

    void
    run ()
    {

        std::string device_id1 = "#"+boost::lexical_cast<std::string>(1);

        //le kinect 2 hanno 4 dev_id per ognuna, quindi la seconda è #5, mentre per le v1 la seconda è #2
        std::string device_id2 = "#"+boost::lexical_cast<std::string>(2);

        grabber1_ = new pcl::io::OpenNI2Grabber (device_id1);
        grabber2_ = new pcl::io::OpenNI2Grabber (device_id2);


        boost::function<void (const CloudConstPtr&) > cloud1_cb = boost::bind (&OpenNI2Scanner::cloud1_callback, this, _1);
        boost::signals2::connection cloud1_connection = grabber1_->registerCallback (cloud1_cb);

        boost::function<void (const CloudConstPtr&) > cloud2_cb = boost::bind (&OpenNI2Scanner::cloud2_callback, this, _1);
        boost::signals2::connection cloud2_connection = grabber2_->registerCallback (cloud2_cb);

//        grabber2_->setDepthCameraIntrinsics(366.006,366.006,258.188,204.534);
//        grabber2_->setRGBCameraIntrinsics(1081.37,1081.37,959.5,539.5);

        activate_kinect1();
        activate_kinect2();

        //ho letto che la kinect 2 ha bisogno di un po di tempo di "warmup"
        boost::this_thread::sleep (boost::posix_time::seconds (10));
        //potremmo chiamarlo 5 o 10 volte per avere piu informazioni se fa ababstanza veloce

        for (int i=0;i<1;i++){
            save_clouds(i);
        }

        deactivate_kinect1();
        deactivate_kinect2();

        cloud1_connection.disconnect ();
        cloud2_connection.disconnect ();

    }

    const int num_cloud = 5;
    CloudConstPtr cloud1_;
    CloudConstPtr cloud2_;
    pcl::io::OpenNI2Grabber*  grabber1_;
    pcl::io::OpenNI2Grabber*  grabber2_;
    mutable boost::mutex cloud_mutex_;
    mutable boost::mutex grab_mutex_;

};


int main() {

    OpenNI2Scanner scanner(1);
    scanner.run ();


    return (0);
}



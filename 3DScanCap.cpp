//! [headers]
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

using namespace std;

bool protonect_shutdown = false; // Whether the running application should shut down.
bool savePointClouds = false;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

int main()
{
    std::cout << "Streaming from Kinect One sensor!" << std::endl;

    //! [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    //! [context]

    //! [discovery]
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;

    if(pipeline)
    {
        //! [open]
        dev = freenect2.openDevice(serial, pipeline);
        //! [open]
    } else {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    signal(SIGINT, sigint_handler); //set protonect_shutdown to true once program interrupted(e.g. press Ctrl+C)
    protonect_shutdown = false;

    //! [listeners]  SyncMultiFrameListener will wait until all specified types of frames are received once
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                  libfreenect2::Frame::Depth |
                                                  libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener); //let listener to receive color frames.
    dev->setIrAndDepthFrameListener(&listener); //let listener to receive depth and IR frames.
    //! [listeners]

    //! [start]
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    //! [start]

    //! [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4); // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger
    //! [registration setup]

    cv::Mat rgbmat, rgbmatResized,rgbd2Resized, rgbd2;//,depthmat, depthmatUndistorted, irmat, rgbd;

    //! [loop start]
    while(!protonect_shutdown)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        //! [loop start]

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        // cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        // cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        cv::resize(rgbmat,rgbmatResized,cv::Size(rgbmat.cols/4,rgbmat.rows/4));
        cv::imshow("rgb", rgbmatResized);
        // cv::imshow("ir", irmat / 4096.0f);
        // cv::imshow("depth", depthmat / 4096.0f);

        //! [registration]
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        //! [registration]

        // cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        // cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);    

        // cv::imshow("undistorted", depthmatUndistorted / 4096.0f);
        // cv::imshow("registered", rgbd);

        cv::resize(rgbd2,rgbd2Resized,cv::Size(rgbd2.cols/4,rgbd2.rows/4));
        cv::imshow("depth2RGB", rgbd2Resized / 4096.0f);

        if(savePointClouds)
        {
            typedef pcl::PointXYZRGB PointT;
            typedef pcl::PointCloud<PointT> PointCloud;

            const double cx = dev->getColorCameraParams().cx;
            const double cy = dev->getColorCameraParams().cy;
            const double fx = dev->getColorCameraParams().fx;
            const double fy = dev->getColorCameraParams().fy;
            const double depthScale = 1000.0;

            PointCloud::Ptr pointCloud( new PointCloud );

            for ( int v=0; v<rgbmat.rows; v++ )
              for ( int u=0; u<rgbmat.cols; u++ )
              {
                float d = rgbd2.ptr<float> ( v+1 )[u]; 
                if ( d== INFINITY ) continue; // inf means invalid depth value
                cv::Vec3d point;
                point[2] = double(d)/depthScale;
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy;
                cv::Vec3d pointWorld = point;

                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = rgbmat.data[ v*rgbmat.step+u*rgbmat.channels() ];
                p.g = rgbmat.data[ v*rgbmat.step+u*rgbmat.channels()+1 ];
                p.r = rgbmat.data[ v*rgbmat.step+u*rgbmat.channels()+2 ];
                pointCloud->points.push_back( p );
              }

            pointCloud->is_dense = false;
            printf("\033[36mpoint cloud saved! Number of point: %lu\033[0m\n",pointCloud->size());            
            pcl::io::savePCDFileBinary("map.pcd", *pointCloud ); 
            savePointClouds = false;
        }

        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
        savePointClouds  = key>0 && ((key & 0xFF) == 115) ;

    //! [loop end]
        listener.release(frames);
    }
    //! [loop end]

    //! [stop]
    dev->stop();
    dev->close();
    //! [stop]

    delete registration;

    std::cout << "Streaming Ends!" << std::endl;
    return 0;
}

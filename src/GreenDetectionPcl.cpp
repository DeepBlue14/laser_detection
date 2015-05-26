#include "GreenDetectionPcl.h"

bool GreenDetectionPcl::activateGuiBool = false;
int GreenDetectionPcl::redInt = 0;
int GreenDetectionPcl::greenInt = 0;
int GreenDetectionPcl::blueInt = 0;

GreenDetectionPcl::GreenDetectionPcl()
{
    pub = new Publisher();
}


void GreenDetectionPcl::dcallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud = msg;

    filterByColor(pclCloud);

    pub->publish(pclCloud);
}


void GreenDetectionPcl::filterByColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg)
{
    for(size_t i = 0; i < msg->points.size(); i++)
    {
        if(msg->points[i].r < redInt)
        {
            msg->points[i].r = 0;
            msg->points[i].g = 0;
            msg->points[i].b = 0;
        }

        if(msg->points[i].g < greenInt)
        {
            msg->points[i].r = 0;
            msg->points[i].g = 0;
            msg->points[i].b = 0;
        }


    }

}


void GreenDetectionPcl::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool GreenDetectionPcl::getActivateGuiBool()
{
    return activateGuiBool;
}


void GreenDetectionPcl::setRedInt(int redInt)
{
    this->redInt = redInt;
}


int GreenDetectionPcl::getRedInt()
{
    return redInt;
}


void GreenDetectionPcl::setGreenInt(int greenInt)
{
    this->greenInt = greenInt;
}


int GreenDetectionPcl::getGreenInt()
{
    return greenInt;
}


void GreenDetectionPcl::setBlueInt(int blueInt)
{
    this->blueInt = blueInt;
}


int GreenDetectionPcl::getBlueInt()
{
    return blueInt;
}


Publisher* GreenDetectionPcl::getPublisher()
{
    return pub;
}


GreenDetectionPcl::~GreenDetectionPcl()
{
    ;
}

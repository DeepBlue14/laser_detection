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

    pub->publish(pclCloud);
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

#include "GreenDetectionPcl.h"

bool GreenDetectionPcl::activateGuiBool = false;
int GreenDetectionPcl::redMinInt = 0;
int GreenDetectionPcl::greenMinInt = 0;
int GreenDetectionPcl::blueMinInt = 0;
int GreenDetectionPcl::redMaxInt = 0;
int GreenDetectionPcl::greenMaxInt = 0;
int GreenDetectionPcl::blueMaxInt = 0;


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
        if((msg->points[i].r < getRedMinInt() ) && (msg->points[i].r > getRedMaxInt() ) )
        {
            msg->points[i].r = 0;
            msg->points[i].g = 0;
            msg->points[i].b = 0;
        }

        if((msg->points[i].g < getGreenMinInt() ) && (msg->points[i].g > getGreenMaxInt() ) )
        {
            msg->points[i].r = 0;
            msg->points[i].g = 0;
            msg->points[i].b = 0;
        }

        if((msg->points[i].b < getBlueMinInt() ) && (msg->points[i].b > getBlueMaxInt() ) )
        {
            msg->points[i].r = 0;
            msg->points[i].g = 0;
            msg->points[i].b = 0;
        }


    }

}


void GreenDetectionPcl::conditionalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    // build the condition
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>() );
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 0.8)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(range_cond);
    condrem.setInputCloud(msg);
    condrem.setKeepOrganized(true);
    // apply filter
    //  condrem.filter(*cloud_filtered); 
}


void GreenDetectionPcl::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem; //or use PointXYZ???
    // build the filter
    outrem.setInputCloud(msg);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius(2);  // dyn config this value
    //apply filter
    outrem.filter(*cloud_filtered);

    msg = cloud_filtered;
}


void GreenDetectionPcl::calculateLaserLoc()
{
    ;
}


void GreenDetectionPcl::setActivateGuiBool(bool activateGuiBool)
{
    this->activateGuiBool = activateGuiBool;
}


bool GreenDetectionPcl::getActivateGuiBool()
{
    return activateGuiBool;
}


void GreenDetectionPcl::setRedMinInt(int redMinInt)
{
    this->redMinInt = redMinInt;
}


int GreenDetectionPcl::getRedMinInt()
{
    return redMinInt;
}


void GreenDetectionPcl::setGreenMinInt(int greenMinInt)
{
    this->greenMinInt = greenMinInt;
}


int GreenDetectionPcl::getGreenMinInt()
{
    return greenMinInt;
}


void GreenDetectionPcl::setBlueMinInt(int blueMinInt)
{
    this->blueMinInt = blueMinInt;
}


int GreenDetectionPcl::getBlueMinInt()
{
    return blueMinInt;
}


void GreenDetectionPcl::setRedMaxInt(int redMaxInt)
{
    this->redMaxInt = redMaxInt;
}


int GreenDetectionPcl::getRedMaxInt()
{
    return redMaxInt;
}


void GreenDetectionPcl::setGreenMaxInt(int greenMaxInt)
{
    this->greenMaxInt = greenMaxInt;
}


int GreenDetectionPcl::getGreenMaxInt()
{
    return greenMaxInt;
}


void GreenDetectionPcl::setBlueMaxInt(int blueMaxInt)
{
    this->blueMaxInt = blueMaxInt;
}


int GreenDetectionPcl::getBlueMaxInt()
{
    return blueMaxInt;
}


Publisher* GreenDetectionPcl::getPublisher()
{
    return pub;
}


GreenDetectionPcl::~GreenDetectionPcl()
{
    ;
}

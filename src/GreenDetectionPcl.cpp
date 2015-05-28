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
    pcPubPtr = new Publisher();
    vec3PubPtr = new Publisher();
}


void GreenDetectionPcl::dcallback(PointCloud<PointXYZRGB>::Ptr msg)
{
    PointCloud<PointXYZRGB>::Ptr pclCloud = msg;

    filterByColor(pclCloud);

    pcPubPtr->publish(pclCloud);
    vec3PubPtr->publish(pointVec3);
}


void GreenDetectionPcl::filterByColor(PointCloud<PointXYZRGB>::Ptr msg)
{
    numPointsSelected = 0.0;
    combinedX = 0.0;
    combinedY = 0.0;
    combinedZ = 0.0;

    if(activateGuiBool == true)
    {
        for(size_t i = 0; i < msg->points.size(); i++)
        {
            if((msg->points[i].r < getRedMinInt() ) || (msg->points[i].r > getRedMaxInt() ) )
            {
                msg->points[i].r = 0;
                msg->points[i].g = 0;
                msg->points[i].b = 0;
            }
            else if((msg->points[i].g < getGreenMinInt() ) || (msg->points[i].g > getGreenMaxInt() ) )
            {
                msg->points[i].r = 0;
                msg->points[i].g = 0;
                msg->points[i].b = 0;
            }
            else if((msg->points[i].b < getBlueMinInt() ) || (msg->points[i].b > getBlueMaxInt() ) )
            {
                msg->points[i].r = 0;
                msg->points[i].g = 0;
                msg->points[i].b = 0;
            }
            else
            {
                //valid points
                numPointsSelected += 1.0;
                combinedX += msg->points[i].x;
                combinedY += msg->points[i].y;
                combinedZ += msg->points[i].z;

                avPoint.x = combinedX / numPointsSelected;
                avPoint.y = combinedY / numPointsSelected;
                avPoint.z = combinedZ / numPointsSelected;
            }


        } // end of for loop
    }

    if(numPointsSelected == 0)
    {
        avPoint.x = -1;
        avPoint.y = -1;
        avPoint.z = -1;
    }

} // end of method


void GreenDetectionPcl::conditionalOutlierRemoval(PointCloud<PointXYZRGB>::Ptr msg)
{
    PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);

    // build the condition
    ConditionAnd<PointXYZRGB>::Ptr range_cond(new ConditionAnd<PointXYZRGB>() );
    range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("z", ComparisonOps::GT, 0.0)));
    range_cond->addComparison(FieldComparison<PointXYZRGB>::ConstPtr(new FieldComparison<PointXYZRGB>("z", ComparisonOps::LT, 0.8)));
    // build the filter
    ConditionalRemoval<PointXYZRGB> condrem(range_cond);
    condrem.setInputCloud(msg);
    condrem.setKeepOrganized(true);
    // apply filter
    //  condrem.filter(*cloud_filtered); 
}


void GreenDetectionPcl::radiusOutlierRemoval(PointCloud<PointXYZRGB>::Ptr msg)
{
    PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);

    RadiusOutlierRemoval<PointXYZRGB> outrem; //or use PointXYZ???
    // build the filter
    outrem.setInputCloud(msg);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius(2);  // dyn config this value
    //apply filter
    outrem.filter(*cloud_filtered);

    msg = cloud_filtered;
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


Publisher* GreenDetectionPcl::getPcPubPtr()
{
    return pcPubPtr;
}


Publisher* GreenDetectionPcl::getVec3PubPtr()
{
    return vec3PubPtr;
}


GreenDetectionPcl::~GreenDetectionPcl()
{
    ;
}

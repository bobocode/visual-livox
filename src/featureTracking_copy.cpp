#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "parameters.h"


using namespace std;
using namespace cv;

class FeatureTracker
{
    public:
        FeatureTracker(ros::NodeHandle &nh)
        {
            // Load params
            std::string config_file;
            n.getParam("feature_config_file", config_file);
            cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
            if(!fsSettings.isOpened())
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
            usleep(100);

            fs["imageWidth"] >> imageWidth;
            fs["imageHeight"] >> imageHeight;
            fs["kImage"] >> kImage;
            fs["dImage"] >> dImage;
            fs["maxFeatureNumPer"] >> maxFeatureNumPerSubregion;
            fs["xSubregionNum"] >> xSubregionNum;
            fs["ySubregionNum"] >> ySubregionNum;
            fs["xBoundary"] >> xBoundary;
            fs["yBoundary"] >> yBoundary;
            fs["maxTrackDis"] >> maxTrackDis;
            fs["winSize"] >> winSize;

            systemInited = false;
            imgSize = cvSize(imageWidth, imageHeight);

            imageCur = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
            imageLast = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);

            showCount = 0;

            imageShow = cvCreateImage(showSize, IPL_DEPTH_8U, 1);
            harrisLast = cvCreateImage(showSize, IPL_DEPTH_32F, 1);

            kMat = cvMat(3, 3, CV_64FC1, kImage);
            dMat = cvMat(4, 1, CV_64FC1, dImage);

            totalSubregionNum = xSubregionNum * ySubregionNum;
            MAXFEATURENUM = maxFeatureNumPerSubregion * totalSubregionNum;

            subregionWidth = (double)(imageWidth - 2 * xBoundary) / (double)xSubregionNum;
            subregionHeight = (double)(imageHeight - 2 * yBoundary) / (double)ySubregionNum;

            featuresCur = new CvPoint2D32f[2 * MAXFEATURENUM];
            featuresLast = new CvPoint2D32f[2 * MAXFEATURENUM];

            featuresFound = new char[2 * MAXFEATURENUM];
            featuresError = new char[2 * MAXFEATURENUM];

            featureIndFromStart = 0;
            featuresInd = new int[ 2 * MAXFEATURENUM] = {0};

            totalFeatureNum = 0；
            subregionFeatureNum = new int[s * totalSubregionNum] = {0};

            mapx = cvCreateImage(imgSize, IPL_DEPTH_32F, 1);
            mapy = cvCreateImage(imgSize, IPL_DEPTH_32F, 1);
            cvInitUndistortMap(&kMat, &dMat, mapx, mapy);

            CvSize subregionSize = cvSize((int)subregionWidth, (int)subregionHeight);
            imageEig = cvCreateImage(subregionSize, IPL_DEPTH_32F, 1);
            imageTmp = cvCreateImage(subregionSize, IPL_DEPTH_32F, 1);

            CvSize pyrSize = cvSize(imageWidth + 8, imageHeight / 3);
            pyrCur = cvCreateImage(pyrSize, IPL_DEPTH_32F, 1);
            pyrLast = cvCreateImage(pyrSize, IPL_DEPTH_32F, 1);

            imageDataSub = nh.subscribe<sensor_msgs::Image>("/image/raw", 1, this->imageDataHandler);

            imagePointsLastPub = nh.advertise<sensor_msgs::PointCloud2> ("/image_points_last", 5);
            imagePointsLastPubPointer = &imagePointsLastPub;

            imageShowPub = nh.advertise<sensor_msgs::Image>("/image/show", 1);
            imageShowPubPointer = &imageShowPub;

        }

        void imageDataHandler(const sensor_msgs::Image::ConstPtr& imageData)
        {
            timeLast = timeCur;
            timeCur = imageData->header.stamp.toSec();

            IplImage *imageTemp = imageLast;
            imageLast = imageCur;
            imageCur = imageTemp;

            const int imagePixelNum = imageHeight * imageWidth;

            for (int i = 0; i < imagePixelNum; i++)
            {
                imageCur->imageData[i] = (char)imageData->data[i];
            }

            IplImage *t = cvCloneImage(imageCur);
            cvRemap(t, imageCur, mapx, mapy);
            //cvEqualizeHist(imageCur, imageCur);
            cvReleaseImage(&t);

            cvResize(imageLast, imageShow);
            cvCornerHarris(imageShow, harrisLast, 3);

            CvPoint2D32f *featuresTemp = featuresLast;
            featuresLast = featuresCur;
            featuresCur = featuresTemp;

            pcl::PointCloud<ImagePoint>::Ptr imagePointsTemp = imagePointsLast;
            imagePointsLast = imagePointsCur;
            imagePointsCur = imagePointsTemp;
            imagePointsCur->clear();

            if (!systemInited)
            {
                systemInited = true;
                return;
            }

            int recordFeatureNum = totalFeatureNum;
            for (int i = 0; i < ySubregionNum; i++) {
                for (int j = 0; j < xSubregionNum; j++) {
                    int ind = xSubregionNum * i + j;
                    int numToFind = maxFeatureNumPerSubregion - subregionFeatureNum[ind];

                    if (numToFind > 0) 
                    {
                        int subregionLeft = xBoundary + (int)(subregionWidth * j);
                        int subregionTop = yBoundary + (int)(subregionHeight * i);
                        CvRect subregion = cvRect(subregionLeft, subregionTop, (int)subregionWidth, (int)subregionHeight);
                        cvSetImageROI(imageLast, subregion);

                        cvGoodFeaturesToTrack(imageLast, imageEig, imageTmp, featuresLast + totalFeatureNum,
                              &numToFind, 0.1, 5.0, NULL, 3, 1, 0.04);

                        int numFound = 0;
                        for(int k = 0; k < numToFind; k++) 
                        {
                            featuresLast[totalFeatureNum + k].x += subregionLeft;
                            featuresLast[totalFeatureNum + k].y += subregionTop;

                            int xInd = (featuresLast[totalFeatureNum + k].x + 0.5) / showDSRate;
                            int yInd = (featuresLast[totalFeatureNum + k].y + 0.5) / showDSRate;

                            if (((float*)(harrisLast->imageData + harrisLast->widthStep * yInd))[xInd] > 1e-7)
                            {
                                featuresLast[totalFeatureNum + numFound].x = featuresLast[totalFeatureNum + k].x;
                                featuresLast[totalFeatureNum + numFound].y = featuresLast[totalFeatureNum + k].y;
                                featuresInd[totalFeatureNum + numFound] = featuresIndFromStart;

                                numFound++;
                                featuresIndFromStart++;
                            }
                        }
                        totalFeatureNum += numFound;
                        subregionFeatureNum[ind] += numFound;

                        cvResetImageROI(imageLast);
                    }
                }
            }

            cvCalcOpticalFlowPyrLK(imageLast, imageCur, pyrLast, pyrCur,
                         featuresLast, featuresCur, totalFeatureNum, cvSize(winSize, winSize), 
                         3, featuresFound, featuresError, 
                         cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.01), 0);

            for (int i = 0; i < totalSubregionNum; i++)
            {
                subregionFeatureNum[i] = 0;
            }

            ImagePoint point;
            int featureCount = 0;
            double meanShiftX = 0, meanShiftY = 0;
            for (int i = 0; i < totalFeatureNum; i++) {
                double trackDis = sqrt((featuresLast[i].x - featuresCur[i].x) 
                    * (featuresLast[i].x - featuresCur[i].x)
                    + (featuresLast[i].y - featuresCur[i].y) 
                    * (featuresLast[i].y - featuresCur[i].y));

                if (!(trackDis > maxTrackDis || featuresCur[i].x < xBoundary || 
                     featuresCur[i].x > imageWidth - xBoundary || featuresCur[i].y < yBoundary || 
                     featuresCur[i].y > imageHeight - yBoundary))
                {

                    int xInd = (int)((featuresLast[i].x - xBoundary) / subregionWidth);
                    int yInd = (int)((featuresLast[i].y - yBoundary) / subregionHeight);
                    int ind = xSubregionNum * yInd + xInd;

                    if (subregionFeatureNum[ind] < maxFeatureNumPerSubregion) 
                    {
                        featuresCur[featureCount].x = featuresCur[i].x;
                        featuresCur[featureCount].y = featuresCur[i].y;
                        featuresLast[featureCount].x = featuresLast[i].x;
                        featuresLast[featureCount].y = featuresLast[i].y;
                        featuresInd[featureCount] = featuresInd[i];

                        point.u = -(featuresCur[featureCount].x - kImage[2]) / kImage[0];
                        point.v = -(featuresCur[featureCount].y - kImage[5]) / kImage[4];
                        point.ind = featuresInd[featureCount];
                        imagePointsCur->push_back(point);

                        if (i >= recordFeatureNum)
                        {
                            point.u = -(featuresLast[featureCount].x - kImage[2]) / kImage[0];
                            point.v = -(featuresLast[featureCount].y - kImage[5]) / kImage[4];
                            imagePointsLast->push_back(point);
                        }

                        meanShiftX += fabs((featuresCur[featureCount].x - featuresLast[featureCount].x) / kImage[0]);
                        meanShiftY += fabs((featuresCur[featureCount].y - featuresLast[featureCount].y) / kImage[4]);

                        featureCount++;
                        subregionFeatureNum[ind]++;
                    }
                }
            }
            totalFeatureNum = featureCount;
            meanShiftX /= totalFeatureNum;
            meanShiftY /= totalFeatureNum;

            sensor_msgs::PointCloud2 imagePointsLast2;
            pcl::toROSMsg(*imagePointsLast, imagePointsLast2);
            imagePointsLast2.header.stamp = ros::Time().fromSec(timeLast);
            imagePointsLastPubPointer->publish(imagePointsLast2);

            showCount = (showCount + 1) % (showSkipNum + 1);
            if (showCount == showSkipNum)
            {
                Mat imageShowMat(imageShow);
                bridge.image = imageShowMat;
                bridge.encoding = "mono8";
                sensor_msgs::Image::Ptr imageShowPointer = bridge.toImageMsg();
                imageShowPubPointer->publish(imageShowPointer);
            }

        }

    private:

        ros::NodeHandle nh;
        bool systemInited;
        double timeCur, timeLast;
        int imageWidth;
        int imageHeight;
        double kImage[9];
        double dImage[4];
        int maxFeatureNumPerSubregion;
        int xSubregionNum;
        int ySubregionNum;
        int xBoundary;
        int yBoundary;
        double maxTrackDis;
        int winSize, showCount;
        int showSkipNum;
        int ShowDSRate;
        CvSize imgSize, showSize;
        IplImage *imageCur, *imageLast, *imageShow, *harrisLast;
        CvMat kMat, dMat;
        IplImage *mapx, *mapy;
        int totalSubregionNum;
        int MAXFEATURENUM; 
        double subregionWidth, subregionHeight;
        IplImage *imageEig, *imageTmp, *pyrCur, *pyrLast;
        CvPoint2D32f *featuresCur, *featuresLast;
        int featureIndFromStart;
        char* featuresFound;
        float* featuresError;
        int* featuresInd;
        int totalFeatureNum;
        int* subregionFeatureNum;

        pcl::PointCloud<ImagePoint>::Ptr imagePointsCur(new pcl::PointCloud<ImagePoint>());
        pcl::PointCloud<ImagePoint>::Ptr imagePointsLast(new pcl::PointCloud<ImagePoint>());
        
        ros::Subscriber imageDataSub;
        ros::Publisher imagePointLastPub;
        ros::Publisher imageShowPub;
        ros::Publisher *imagePointsLastPubPointer;
        ros::Publisher *imageShowPubPointer;

        cv_bridge::CvImage bridge;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "featureTracking");
  ros::NodeHandle nh;

  FeatureTracker feature_tracker(nh);

  ros::spin();

  return 0;
}


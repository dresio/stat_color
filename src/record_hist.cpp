#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

using namespace cv;
using namespace std;

//patches the currently non-functional to_string function
namespace patch
{
    template < typename T > string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

string pack_path;
int DISPLAY;
int bin_size =256;
int white_threshold = 40;

void calcHistColorMask(Mat image, vector<Mat> &out)
{
	// Seperate BGR
	vector<Mat> bgr;
	split( image, bgr );

	Mat mask;
	inRange(image, Scalar(white_threshold, white_threshold, white_threshold), Scalar(255, 255, 255), mask);
    bitwise_not(mask,mask);

	int histSize = bin_size;

	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;

	Mat b_hist, g_hist, r_hist;

	calcHist( &bgr[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 500;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	
	out.push_back(b_hist);
	out.push_back(g_hist);
	out.push_back(r_hist);
}


vector<Mat> getHist(Mat image)
{
	Mat img;
	vector<Mat> compare;

	vector<Mat> hist_img;
	calcHistColorMask(image, hist_img);

    return hist_img;
} 

//saves histogram to .hist files
void saveHist(vector<Mat> color, string name)
{
    //save mat directly to computer to be loaded back later
	string path = pack_path + "/config/hists/";
	cv::FileStorage file(path + name + "_h.hist", cv::FileStorage::WRITE);
	file << "" << color[0];
	file = cv::FileStorage(path + name + "_s.hist", cv::FileStorage::WRITE);
	file << "" << color[1];
	file = cv::FileStorage(path + name +"_v.hist", cv::FileStorage::WRITE);
	file << "" << color[2];    
}

//streamline this later
int main(int argc, char **argv)
{
    //initializes node
    ros::init(argc, argv, "Histogram_Generator");
	ros::NodeHandle nh;

    //gets basic data
    string file_type;
    nh.param("/path", pack_path, pack_path);
    nh.param("/display", DISPLAY, DISPLAY);
    nh.param("/file_type", file_type, file_type);

    //gets number of catagories
    int num_colors;
    nh.param("/num_colors", num_colors, num_colors);

    //gets the name of catagories
    vector<string> name_colors;
    for(int i = 0; i < num_colors; i++)
    {
        string temp_name;
        string param_name = "/color_" + patch::to_string(i);
        nh.param(param_name, temp_name, temp_name);
        name_colors.push_back(temp_name);
    }    

    //finds the number of samples
    vector<int> num_samples;
    for(int i = 0; i < num_colors; i++)
    {
        int samples;
        string param_name = "/num_" + patch::to_string(i);
        nh.param(param_name, samples, samples);
        num_samples.push_back(samples);
    }

    //processes iterativly through colors and samples
    for(int i = 0; i < num_colors; i++)
    {
        //definitions and printouts
        ROS_WARN("PROCESSING %d IMAGES FROM COLOR: %s", num_samples[i], name_colors[i]);
        Mat color;
        string part_path = pack_path + "/data/" + name_colors[i] + "/";  //path without indiviual pic num
        
        //initilizes first histogram
        string full_path = part_path + "0" + file_type;
        color = imread(full_path, CV_LOAD_IMAGE_COLOR);
        cvtColor(color, color, CV_RGB2HSV);
        vector<Mat> hist_color = getHist(color);

        //iterates through samples now
        for(int i = 1; i < num_samples[i]; i++)
        {
            full_path = part_path + patch::to_string(i) + file_type;
            Mat image_temp = imread(full_path, CV_LOAD_IMAGE_COLOR);
            cvtColor(image_temp, image_temp, CV_RGB2HSV);
            vector<Mat> hist_temp = getHist(image_temp);

            //averages the histograms
            for(int i = 0; i < 3; i++)
            {
                Mat temp;
                cv::add(hist_color[i], hist_temp[i], temp);
                temp/=2;
                hist_color[i] = temp;
            }
        }

        //normailize histogram
        int hist_w = 512; int hist_h = 500;
	    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
        normalize(hist_color[0], hist_color[0], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
        normalize(hist_color[1], hist_color[1], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
        normalize(hist_color[2], hist_color[2], 0, histImage.rows, NORM_MINMAX, -1, Mat() );

        //save histogram
        saveHist(hist_color, name_colors[i]);
        ROS_WARN("SAVED %s HISTOGRAM", name_colors[i]);
    }

    ROS_WARN("FINISHED AND EXITING");


	return 0;

}

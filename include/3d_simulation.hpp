#ifndef CAMSIM_SIMULATION_H
#define CAMSIM_SIMULATION_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include "camera.hpp"
#include "geometry.hpp"
#include "objectMeshModel.hpp"

inline double abs(Point p)
{
  return std::sqrt(p.x()*p.x() + p.y()*p.y() + p.z()*p.z());
}

inline double sq(float x)
{
  return x*x;
}


class Simulation{
    public:
        Simulation(Camera *camera, std::string model_path, std::string pattern_path){
            
            out_path_ = "/home/nullbyte/Desktop/3D-CamSim/output/";
            /* Allocate memory for depth image */
            w = camera->width;
	        h = camera->height;
        
            depth_im_ = cv::Mat(h, w, CV_32FC1);
            scaled_im_ = cv::Mat(h, w, CV_32FC1);
            model_ = new ObjectMeshModel(model_path);
            search_ = new TreeAndTri;
            updateTree();
            
            /* colour map */
            color_map_.push_back(cv::Scalar( rand()&255, rand()&255, rand()&255 ));
            
            dot_pattern_ = cv::imread(pattern_path.c_str(), 0);

            initFilter();

            transform_ = Eigen::Affine3d::Identity();

            camera_ = camera;

        }
    void updateTree();
    void initFilter();
    void projection(const Eigen::Affine3d &new_tf, bool store_depth);
    void intersect(const Eigen::Affine3d &p_transform, cv::Mat &depth_map);

    private:
        Camera *camera_;
        int w, h;

        cv::Mat depth_im_, scaled_im_;
        // boost::shared_ptr<ObjectMeshModel> model_;
        TreeAndTri* search_;
        // filter masks 
        static const int size_filt_ = 9;
        cv::Mat weights_;
        cv::Mat fill_weights_;

        ObjectMeshModel *model_;
        std::vector<cv::Scalar> color_map_;
        cv::Mat dot_pattern_; 

        Eigen::Affine3d transform_; 

        std::string out_path_;
        
        int countf;
        static constexpr float invalid_disp_ = 99999999.9; 
};
#endif


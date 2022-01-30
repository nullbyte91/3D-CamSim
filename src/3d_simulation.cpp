#include "3d_simulation.hpp"

// Function that triggers AABB tree update by exchanging old vertices 
// with new vertices according to the updated transform
void Simulation::updateTree()
{
    model_->uploadVertices(search_);
    model_->uploadIndices(search_);
    // since we are only dealing with one mesh for now, ID=0
    model_->uploadPartIDs(search_, 0);

    search_->tree.rebuild(search_->triangles.begin(), search_->triangles.end());
    search_->tree.accelerate_distance_queries();
}

void Simulation::initFilter()
{
    weights_ = cv::Mat(size_filt_,size_filt_,CV_32FC1);
    for(int x=0; x<size_filt_; ++x){
        float *weights_i = weights_.ptr<float>(x);
        for(int y=0; y<size_filt_; ++y){
	        int tmp_x = x-size_filt_/2;
	        int tmp_y = y-size_filt_/2;
	        if(tmp_x!=0 && tmp_y!=0)
	            weights_i[y] = 1.0/(sq(1.2*(float)tmp_x)+sq(1.2*(float)tmp_y));
	        else 
	            weights_i[y] = 1.0;
        }
    }

    fill_weights_ = cv::Mat(size_filt_, size_filt_, CV_32FC1);
    for(int x=0; x<size_filt_; ++x){
        float *weights_i = fill_weights_.ptr<float>(x);
        for(int y=0; y<size_filt_; ++y){
            int tmp_x = x-size_filt_/2;
            int tmp_y = y-size_filt_/2;
            if(std::sqrt(sq(tmp_x)+sq(tmp_y)) < 3.1)
                weights_i[y] = 1.0/(1.0+sq(tmp_x)+sq(tmp_y));
            else 
                weights_i[y] = -1.0;
        }
    }
}

void Simulation::intersect(const Eigen::Affine3d &p_transform,  cv::Mat &depth_map)
{
    model_->updateTransformation(p_transform);
    updateTree();

    /* allocate memory for depth map and labels */
    depth_map = cv::Mat(h, w, CV_64FC1);
    depth_map.setTo(0.0);
    cv::Mat disp(h, w, CV_32FC1);
    disp.setTo(invalid_disp_);

}
void Simulation::projection(const Eigen::Affine3d &new_tf, bool store_depth){
    /* Update new transform */
    transform_ = new_tf;

    
}

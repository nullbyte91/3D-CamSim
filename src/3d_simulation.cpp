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
    cv::Mat out_disp;
    model_->updateTransformation(p_transform);
    updateTree();

    /* allocate memory for depth map and labels */
    depth_map = cv::Mat(h, w, CV_64FC1);
    depth_map.setTo(0.0);
    cv::Mat disp(h, w, CV_32FC1);
    disp.setTo(invalid_disp_);

    /* go through the whole image and create a ray from a pixel -> dir */    
    std::vector<cv::Point3f> vec;
    int n_occluded = 0;
    vec.reserve(h * w);

    for(int c=0; c<w; ++c) { 
        for(int r=0; r<h; ++r) {
	    /* compute ray from pixel and camera configuration */
	    cv::Point3f ray = camera_->projectPixelTo3dRay(cv::Point2f(c,r));
	    
        /* check if there is any intersection of the ray with an object by do_intersect */
	    uint32_t reach_mesh = search_->tree.do_intersect(Ray(Point(0,0,0), Vector(ray.x, ray.y, ray.z)));
	    if (reach_mesh){
	        /* if there is one or many intersections, order them according to distance to camera
	           and continue computation with closest */
	        std::list<Object_and_Primitive_id> intersections;
	        search_->tree.all_intersections(Ray(Point(0,0,0),Vector( ray.x, ray.y, ray.z)), 
					  std::back_inserter(intersections));
	        if(!intersections.empty()) {
	            std::list<Object_and_Primitive_id>::const_iterator it;
	            Point min_p(0,0,0);
	            double min_dist = std::numeric_limits<double>::infinity();
	            double min_id = -1;
	            for (it = intersections.begin(); it != intersections.end(); ++it) {
	                CGAL::Object object = it->first;
	                std::size_t triangle_id = std::distance(search_->triangles.begin(),it->second); 
	                assert( search_->triangles[triangle_id] == *(it->second) ); 
	                Point point;
	                if(CGAL::assign(point,object)){
		                double dist = abs(point);
		                if(dist<min_dist){
		                    /* distance to point */
		                    min_dist = dist;
		  
                            /* intersection coordinates */
		                    min_p = point;
		                    /* label of the intersected object (will be zero for this simple case) */
		                    min_id = triangle_id;
		                }
	                    } else {
		                    std::cout << "Intersection object is NOT a point ?????" << std::endl;
	                    }
	            }

	        /* check if point is also visible in second camera by casting a ray to this point */
	        uint32_t reach_mesh_r = search_->tree.do_intersect(Ray(Point(camera_->tx_,0,0), 
								   Point(min_p.x(), min_p.y(), min_p.z())));
	        if(reach_mesh_r) {
	            /* if there are intersections, get the closest to the camera */
	            std::list<Object_and_Primitive_id> intersections_r;
	            search_->tree.all_intersections(Ray(Point(camera_->tx_,0,0), Point(min_p.x(), min_p.y(), min_p.z())), 
					      std::back_inserter(intersections_r));
	            if(!intersections_r.empty()) {
		            std::list<Object_and_Primitive_id>::const_iterator id;
		            Point min_p_r(min_p.x(), min_p.y(), min_p.z());
		            double min_dist_r = std::numeric_limits<double>::infinity();
		            for (id = intersections_r.begin(); id != intersections_r.end(); ++id) {
		                CGAL::Object object = id->first;
		                Point point;
		                if(CGAL::assign(point,object)){
		                    double dist = abs(point);
		                    if(dist<min_dist_r){
		                        min_dist_r = dist;
		                        min_p_r = point;
		                    }
		                }
		            }
		
		            /* check if closest intersection is the same as from the left image
		            if not, point is not visible in both images -> occlusion boundary */
		            Point diff(min_p_r.x() - min_p.x(), min_p_r.y() - min_p.y(), min_p_r.z() - min_p.z());
		            if(abs(diff)<0.0001) {
                        // get pixel position of ray in right image
                        float tx_fx = camera_->tx_;
                        cv::Point3d point_right( min_p.x()-tx_fx, min_p.y(), min_p.z());
                        cv::Point2f right_pixel = camera_->project3dToPixel(point_right);
                        // quantize right_pixel
                        right_pixel.x = round(right_pixel.x*8.0)/8.0;
                        right_pixel.y = round(right_pixel.y*8.0)/8.0;
                        // compute disparity image
                        float quant_disp = c - right_pixel.x; 
                        float* disp_i = disp.ptr<float>(r);
                        disp_i[(int)c] = quant_disp;
                        // fill label image with part id 
                        // unsigned char* labels_i = labels.ptr<unsigned char>(r);
                        // cv::Scalar color = color_map_[search_->part_ids[min_id]];
                        // for(int col=0; col<3; ++col){
                        //     labels_i[(int)c*3+col] = color(col);
                        // }
                    } else {
                    n_occluded++;
                    }
	            } // if there are non-zero intersections from right camera
	        } // if mesh reached from right camera
	    } // if non-zero intersections
	} // if mesh reached 
    } // camera_.getHeight()
    } // camera_.getWidth()

    for(int r=0; r<h; ++r) {
        float* disp_i = out_disp.ptr<float>(r);
        double* depth_map_i = depth_map.ptr<double>(r);
        for(int c=0; c<w; ++c) {
	        float disp = disp_i[c];
	        if(disp<invalid_disp_){
	            cv::Point3d new_p;
	            new_p.z = (camera_->fx_ * camera_->tx_) / disp;
	            if(new_p.z<camera_->z_near || new_p.z>camera_->z_far){
	                continue;
	            }
                new_p.x = (new_p.z/ camera_->fx_) * (c - camera_->cx_);
                new_p.y = (new_p.z/ camera_->fy_) * (r - camera_->cy_);
                vec.push_back(new_p);
                depth_map_i[(int)c] = new_p.z;
	        }
        }
    }
}   
void Simulation::projection(const Eigen::Affine3d &new_tf, bool store_depth){

    /* Frame count */
    countf++;

    /* Update new transform */
    transform_ = new_tf;

    intersect(transform_, depth_im_);

    convertScaleAbs(depth_im_, scaled_im_, 255.0f);

    std::stringstream lD;
	lD << out_path_ << "depth_orig" << std::setw(3) << std::setfill('0')
	<< countf << ".png";

    // cv::imwrite(lD.str().c_str(), depth_im_);
    cv::imshow("window", depth_im_);
    cv::waitKey(0);
    
}

#include "camera.hpp"
#include "geometry.hpp"
#include "3d_simulation.hpp"

void getRandomTransform(const double &p_x,
			const double &p_y,
			const double &p_z,
			const double &p_angle,
			Eigen::Affine3d &p_tf)
{
  Eigen::Vector3d axis(((double)(rand()%1000))/1000.0,
		       ((double)(rand()%1000))/1000.0,
		       ((double)(rand()%1000))/1000.0);
  Eigen::Vector3d t(p_x*(double)(rand()%2000 -1000)/1000,
		    p_y*(double)(rand()%2000 -1000)/1000,
		    p_z*(double)(rand()%2000 -1000)/1000);
  p_tf = Eigen::Affine3d::Identity();
  p_tf.translate(t);
  p_tf.rotate(Eigen::AngleAxisd( p_angle*(double)(rand()%2000 - 1000)/1000, axis));
}

int main(int argc, char **argv)
{
    int w = 1000;
    int h = 1000;
    double cx = 320;
    double cy = 240;
    double fx = 580.0, fy = 580.0;
    double tx = 0.075, ty = 0.0;
    double z_n =  0.5, z_f = 6.0;
    Camera *camera = new Camera(w, h, z_n, z_f, fx, fy, cx, cy, tx);
    
    std::string model_path = "/home/nullbyte/Desktop/3D-CamSim/model/cube.obj";
    std::string pattern_path = "../data/kinect-pattern_3x3.png";
    Simulation * simulation = new Simulation(camera, model_path, pattern_path);

    int frames = 10;
    bool store_depth = 1;

    Eigen::Affine3d noise;

    /* Set init transform */
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());
    transform.translate(Eigen::Vector3d(0.089837, -0.137769, 0.949210));
    transform.rotate(Eigen::Quaterniond(0.906614,-0.282680,-0.074009,-0.304411));

    for(int i=0; i<frames; ++i) {
        getRandomTransform(0,0,0,0, noise);
        Eigen::Affine3d current_tf = noise*transform;
        simulation->projection(current_tf, store_depth);
    }

    return 0;
}
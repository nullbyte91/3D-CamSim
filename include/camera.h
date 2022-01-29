#ifndef CAMSIM_CAMERA_H
#define CAMSIM_CAMERA_H

#include <opencv2/opencv.hpp>

/* Reference taken from sensor_msgs/CameraInfo.msg */

class CameraInfo{
    public:
        //The image dimensions with which the camera was calibrated. Normally
        //this will be the full camera resolution in pixels.
        int width, height;
        
        // The zNear and zFar are distances away from the "camera" in world units
        double z_near, z_far; 
        
        // Focal length
        double fx_, fy_;

        // Principal points 
        double cx_, cy_; 

        // For a stereo pair
        double tx_, ty_;
        
        /* Projection/camera matrix
            [fx'  0  cx' Tx]
        P = [ 0  fy' cy' Ty]
            [ 0   0   1   0]
        By convention, this matrix specifies the intrinsic (camera) matrix
        of the processed (rectified) image. That is, the left 3x3 portion
        is the normal camera intrinsic matrix for the rectified image.
        It projects 3D points in the camera coordinate frame to 2D pixel
        coordinates using the focal lengths (fx', fy') and principal point
        (cx', cy') - these may differ from the values in K.
        For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
        also have R = the identity and P[1:3,1:3] = K.
        For a stereo pair, the fourth column [Tx Ty 0]' is related to the
        position of the optical center of the second camera in the first
        camera's frame. We assume Tz = 0 so both cameras are in the same
        stereo image plane. The first camera always has Tx = Ty = 0. For
        the right (second) camera of a horizontal stereo pair, Ty = 0 and
        Tx = -fx' * B, where B is the baseline between the cameras.
        Given a 3D point [X Y Z]', the projection (x, y) of the point onto
        the rectified image is given by:
        [u v w]' = P * [X Y Z 1]'
                x = u / w
                y = v / w
        This holds for both images of a stereo pair.*/
};

class Camera : public CameraInfo{
    private:
        Camera(int w, int h, double z_n, double z_f, double fx, 
                        double fy, double cx, double cy, double tx){
            width =w;
            h = h;
            z_near = z_n;
            z_far = z_f;
            fx_ = fx;
            fy_ = fy;
            cx_ = cx;
            cy_ = cy;
            tx_ = tx;
        }
        /*
        @brief Project a 3d point to rectified pixel coordinates
        @Param xyz	3d point in the camera coordinate frame
        @Return (u,v) in rectified pixel coordinates
        */
        cv::Point2d project3dToPixel(const cv::Point3d& xyz) const
        {
            cv::Point2d uv_rect;
            uv_rect.x = (fx_*xyz.x) / xyz.z + cx_;
            uv_rect.y = (fy_*xyz.y) / xyz.z + cy_;
            return uv_rect;
        }

        /*
        @brief Project a rectified pixel to a 3d ray.
        Returns the unit vector in the camera coordinate frame in 
        the direction of rectified pixel (u,v) in the image plane. 
        @Param uv_rect	Rectified pixel coordinates
        @Return 3d ray passing through (u,v)
        */

        cv::Point3d projectPixelTo3dRay(const cv::Point2d& uv_rect) const
        {
            cv::Point3d ray;
            ray.x = (uv_rect.x - cx_) / fx_;
            ray.y = (uv_rect.y - cy_) / fy_;
            ray.z = 1.0;
            return ray;
        }
};
#endif
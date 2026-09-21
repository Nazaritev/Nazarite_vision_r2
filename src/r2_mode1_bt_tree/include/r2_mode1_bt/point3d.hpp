#ifndef R2_MODE1_BT_POINT3D_HPP_
#define R2_MODE1_BT_POINT3D_HPP_

namespace r2_mode1_bt {

struct Point3d {
    double x;
    double y;
    double z;
    double yaw;

    Point3d(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0, double yaw_ = 0.0)
        : x(x_), y(y_), z(z_), yaw(yaw_) {}
};

}  // namespace r2_mode1_bt

#endif  // R2_MODE1_BT_POINT3D_HPP_

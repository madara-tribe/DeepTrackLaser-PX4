#include <iostream>
int x_ori = 17; // (+17px)
double pixel_ratio = 0.171837;       // Real-world units per pixel
double max_x = 180.0;

double calculateRealCoordinate(double x, double w) {
    // Compute pixel offset from image center
    double x_center = x + w / 2.0;
    double x_coordinate = x_center + static_cast<double>(x_ori);

    // Convert to real-world distance using pixel-to-meter (or cm) ratio
    double x_real_coord = x_coordinate * pixel_ratio;

    return x_real_coord;
}

float calculateBboxSize(double x, double y, double w, double h) {
    double width = w - x;
    double height = h - y;
    if (width <= 0 || height <= 0) {
        return 0.0f;  // invalid box
    }
    return static_cast<float>(width * height);
}

double quadraticFunction(double x) {
    double a = 0.00143026;
    double b = 0.32379224;
    double c = -56.7942361;
    return a * std::pow(x, 2) + b * x + c;
}

double calculateClampedAngle(double x, double z, double max_x) {
    // Clamp x to the range [0, max_x]
    x = std::max(0.0, std::min(x, max_x));
    // coordinates
    double Ax = 0.0, Ay = 0.0;
    double Px = max_x / 2.0, Py = z;
    double hx = x, hy = 0.0;

    // Vectors AP and HP (origin shifted to point P)
    double vec_AP_x = Ax - Px;
    double vec_AP_y = Ay - Py;

    double vec_hP_x = hx - Px;
    double vec_hP_y = hy - Py;

    // Dot product and vector magnitude
    double dot = vec_AP_x * vec_hP_x + vec_AP_y * vec_hP_y;
    double mag_AP = std::hypot(vec_AP_x, vec_AP_y);
    double mag_hP = std::hypot(vec_hP_x, vec_hP_y);

    // Safety check to avoid division by zero
    if (mag_AP == 0 || mag_hP == 0) return 0.0;

    // cosθ = dot / (|a||b|)
    double cos_theta = dot / (mag_AP * mag_hP);

    // Clamp cos_theta to [-1, 1] to prevent domain errors
    if (cos_theta < -1.0) cos_theta = -1.0;
    if (cos_theta > 1.0) cos_theta = 1.0;

    double angle_rad = std::acos(cos_theta);
    return angle_rad * (180.0 / M_PI);
}


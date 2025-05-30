#include <iostream>
int x_ori = 17; // (+17px)
double pixel_ratio = 0.171837;       // Real-world units per pixel
double max_x = 100.0;

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




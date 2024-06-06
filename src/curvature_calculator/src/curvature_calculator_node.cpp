#define ZERO_THRE 1e-5

#include "curvature_calculator/spline/spline.h"
#include "curvature_calculator/curvature_calculator_node.hpp"
#include <iostream>
#include <vector>
#include <cmath>


Curvature::Curvature(ConfigReader& config_reader) : config_reader_(config_reader) {

}

void Curvature::init(const lanelet::LineString3d& centerline) {

    // Make nodes of centerline to build K-D tree
    resampled_spline_ = makeSmoothSpline(centerline);
    std::vector<SplinePoint> node_points = quantizeSpline(resampled_spline_);

    // Build K-D tree
    std::cout << "Building K-D tree...\n";
    kdtree_ = KDTree(node_points);
    std::cout << "K-D tree is built!\n";
}

void Curvature::run(double& track_curvature, const lanelet::Point2d car_point) {
    SplinePoint projected_point = kdtree_.project(car_point.x() + UTM_EASTING_CORRECTION, car_point.y());
    double dx = resampled_spline_.x_of.deriv(1, projected_point.s);
    double dy = resampled_spline_.y_of.deriv(1, projected_point.s);
    double ddx = resampled_spline_.x_of.deriv(2, projected_point.s);
    double ddy = resampled_spline_.y_of.deriv(2, projected_point.s);
    double square_sum = std::pow(dx, 2) + std::pow(dy, 2);
    if (square_sum < ZERO_THRE) square_sum = ZERO_THRE;
    track_curvature = (dx * ddy - dy * ddx) / std::pow(square_sum, 1.5);
}

//! ####################### FOR DEBUG ####################### 
void Curvature::debugRun(double& track_curvature, const lanelet::Point2d car_point, double& best_distance, double& best_point_easting, double& best_point_northing) {
    NearestSplinePoint projected_point = kdtree_.debugProject(car_point.x() + UTM_EASTING_CORRECTION, car_point.y());
    best_distance = projected_point.distance;
    best_point_easting = projected_point.x;
    best_point_northing = projected_point.y;
    double dx = resampled_spline_.x_of.deriv(1, projected_point.s);
    double dy = resampled_spline_.y_of.deriv(1, projected_point.s);
    double ddx = resampled_spline_.x_of.deriv(2, projected_point.s);
    double ddy = resampled_spline_.y_of.deriv(2, projected_point.s);
    double square_sum = std::pow(dx, 2) + std::pow(dy, 2);
    double zero_thre = 1e-5;
    if (square_sum < zero_thre) square_sum = zero_thre;
    track_curvature = (dx * ddy - dy * ddx) / std::pow(square_sum, 1.5);
}

Spline Curvature::debugGetResampledSpline() {
    return resampled_spline_;
}
//! #########################################################

Spline Curvature::createSpline(std::vector<double> x, std::vector<double> y) {
    // spline parameters
    tk::spline::spline_type type = tk::spline::cspline;
    bool is_closed_curve = false;
    bool make_monotonic = false;

    //setup auxiliary "time grid"
    double tmin = 0.0, tmax = 0.0;
    std::vector<double> T;
    createTimeGrid(T, tmin, tmax, x, y, is_closed_curve);

    // define original spline for each coordinate x, y
    tk::spline sx, sy;
    sx.set_points(T, x, type);
    sy.set_points(T, y, type);
    if(make_monotonic) {
        //adjusts spline coeffs to be piecewise monotinic where possible
        sx.make_monotonic();
        sy.make_monotonic();
    }

    Spline spline = {sx, sy, tmin, tmax};

    return spline;
}

PointSet Curvature::samplePoints(Spline spline_before_sampling) {
    Spline& spline = spline_before_sampling;

    // storage for sampled points
    std::vector<double> sampled_points_x;
    std::vector<double> sampled_points_y;
    std::vector<lanelet::Point3d> sampled_points;

    // read config file to get centerline sampling distance
    double SAMPLING_DISTANCE = config_reader_.readAsDouble("SAMPLING_DISTANCE");

    // sample points from original spline
    for (int t = spline.tmin; t < spline.tmax; t += SAMPLING_DISTANCE) {
        sampled_points_x.push_back(spline.x_of(t));
        sampled_points_y.push_back(spline.y_of(t));
        lanelet::Point3d tmp_sampled_point = lanelet::Point3d(lanelet::utils::getId(), {spline.x_of(t), spline.y_of(t), 0});
        sampled_points.push_back(tmp_sampled_point);
    }
    sampled_points_x.push_back(spline.x_of(spline.tmax));
    sampled_points_y.push_back(spline.y_of(spline.tmax));
    lanelet::Point3d tmp_sampled_point = lanelet::Point3d(lanelet::utils::getId(), {spline.x_of(spline.tmax), spline.y_of(spline.tmax), 0});
    sampled_points.push_back(tmp_sampled_point);

    PointSet sampled_point_set;
    sampled_point_set.x = sampled_points_x;
    sampled_point_set.y = sampled_points_y;
    sampled_point_set.points = sampled_points;

    return sampled_point_set;
}

// double Curvature::calculate_curvature(Spline resampled_spline, )

void Curvature::createTimeGrid(std::vector<double>& T, double& tmin, double& tmax,
                    std::vector<double>& X, std::vector<double>& Y, bool is_closed_curve)
{
    assert(X.size()==Y.size() && X.size()>2);
    // hack for closed curves (so that it closes smoothly):
    //  - append the same grid points a few times so that the spline
    //    effectively runs through the closed curve a few times
    //  - then we only use the last loop
    //  - if periodic boundary conditions were implemented then
    //    simply setting periodic bd conditions for both x and y
    //    splines is sufficient and this hack would not be needed
    int idx_first=-1, idx_last=-1;
    if(is_closed_curve) {
        // remove last point if it is identical to the first
        if(X[0]==X.back() && Y[0]==Y.back()) {
            X.pop_back();
            Y.pop_back();
        }
        const int num_loops=3;  // number of times we go through the closed loop
        std::vector<double> Xcopy, Ycopy;
        for(int i=0; i<num_loops; i++) {
            Xcopy.insert(Xcopy.end(), X.begin(), X.end());
            Ycopy.insert(Ycopy.end(), Y.begin(), Y.end());
        }
        idx_last  = (int)Xcopy.size()-1;
        idx_first = idx_last - (int)X.size();
        X = Xcopy;
        Y = Ycopy;
        // add first point to the end (so that the curve closes)
        X.push_back(X[0]);
        Y.push_back(Y[0]);
    }
    // setup a "time variable" so that we can interpolate x and y
    // coordinates as a function of time: (X(t), Y(t))
    T.resize(X.size());
    T[0]=0.0;
    for(size_t i=1; i<T.size(); i++) {
        // time is proportional to the distance, i.e. we go at a const speed
        T[i] = T[i-1] + sqrt(std::pow((X[i]-X[i-1]), 2) + std::pow((Y[i]-Y[i-1]), 2));
    }
    if(idx_first<0 || idx_last<0) {
        tmin = T[0] - 0.0;
        tmax = T.back() + 0.0;
    } else {
        tmin = T[idx_first];
        tmax = T[idx_last];
    }
}

Spline Curvature::makeSmoothSpline(const lanelet::LineString3d& centerline) {
    basic_centerline_ = centerline.basicLineString();
    std::vector<double> centerline_x;  // x of original centerline
    std::vector<double> centerline_y;  // y of original centerline
    for (int i = 0; i < basic_centerline_.size(); i++) {
        centerline_x.push_back(basic_centerline_[i].x() + UTM_EASTING_CORRECTION);
        centerline_y.push_back(basic_centerline_[i].y());
    }

    // create original spline with every centerline points
    std::cout << "Creating original spline...\n";
    Spline original_spline = createSpline(centerline_x, centerline_y);

    // sample points from original spline
    std::cout << "Sampling points from original spline...\n";
    PointSet sampled_points = samplePoints(original_spline);

    // create resampled spline from sampled points
    std::cout << "Creating resampled spline from sampled points...\n";

    return createSpline(sampled_points.x, sampled_points.y);
}

std::vector<SplinePoint> Curvature::quantizeSpline(Spline& resampled_spline) {
    std::cout << "Quantizing resampled_points and make node points for kd tree...\n";
    double QUANTIZATION_DISTANCE = config_reader_.readAsDouble("QUANTIZATION_DISTANCE");
    std::vector<SplinePoint> node_points;
    for (double t = resampled_spline_.tmin; t <= resampled_spline_.tmax; t += QUANTIZATION_DISTANCE) {
        node_points.push_back(SplinePoint{resampled_spline_.x_of(t), resampled_spline_.y_of(t), t});
    }
    node_points.push_back(SplinePoint{resampled_spline_.x_of(resampled_spline_.tmax), resampled_spline_.y_of(resampled_spline_.tmax), resampled_spline_.tmax});

    return node_points;
}
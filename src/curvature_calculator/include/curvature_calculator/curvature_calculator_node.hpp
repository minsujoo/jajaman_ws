#ifndef CURVATURE_CALCULATOR_NODE_HPP
#define CURVATURE_CALCULATOR_NODE_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include "curvature_calculator/spline/spline.h"

struct Point {
    double x;
    double y;
};

struct SplinePoint {
    double x;
    double y;
    double s;
};

struct NearestSplinePoint {
    double x;
    double y;
    double s;  // spline index. same as Frenet.s
    double distance;
};

struct Spline {
    tk::spline x_of;
    tk::spline y_of;
    double tmin;
    double tmax;
};

struct PointSet {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<lanelet::Point3d> points;
};

class Curvature {

public:
    Curvature(ConfigReader& config_reader);
    void init(const lanelet::LineString3d& centerline);
    void run(double& rt_track_curvature, const lanelet::Point2d car_point);
    void debugRun(double& track_curvature, const lanelet::Point2d car_point, double& best_distance, double& best_point_easting, double& best_point_northing);  //! FOR DEBUG
    Spline debugGetResampledSpline();  //! FOR DEBUG

private:
    Spline createSpline(std::vector<double> x, std::vector<double> y);
    PointSet samplePoints(Spline spline_before_sampling);
    void createTimeGrid(std::vector<double>& T, double& tmin, double& tmax,
                        std::vector<double>& X, std::vector<double>& Y, bool is_closed_curve);

private:
    Spline makeSmoothSpline(const lanelet::LineString3d& centerline);
    std::vector<SplinePoint> quantizeSpline(Spline& resampled_spline);

private:
    lanelet::BasicLineString3d basic_centerline_;
    KDTree kdtree_;
    Spline resampled_spline_;

private:
    ConfigReader& config_reader_;
};

#endif
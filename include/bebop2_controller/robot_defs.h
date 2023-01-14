#pragma once
#include <iostream>
#include <string>
#include <math.h>

//const int STATE_DIM = 10;

struct RobotState
{
    double x;      // x-position (in meters)
    double y;      // y-position (in meters)
    double z;      // z-position (in meters)
    double theta;  // z-orientation (in radians)
//    double vx;     // x-linear velocity (in meters)
//    double vy;     // y-linear velocity (in meters)
//    double vz;     // z-linear velocity (in meters)
//    double wx;     // x-angular velocity (in meters)
//    double wy;     // y-angular velocity (in meters)
//    double wz;     // z-angular velocity (in meters)
};


/* Field location structure in radial distance (r), azimuthal angle (theta), and polar angle (psi)*/
struct MarkerObservation
{
    std::string tagName;     // Index of observed marker [0-n]
    double radial_distance;    // Observed distance to landmark from robot position
    double azimuthal_angle; // Observed bearing to landmark in the XY coordinate frame
    double polar_angle; // Observed bearing to landmark in the positive Z axis,

    friend std::ostream &operator<<(std::ostream &os, const MarkerObservation &observation) {
        double degFactor = (180.0/3.141592653589793238463);
        os << "tagName: " << observation.tagName << " radial_distance: " << observation.radial_distance
           << " azimuthal_angle: " << observation.azimuthal_angle * degFactor << " polar_angle: " << observation.polar_angle * degFactor;
        return os;
    }

};


/* Field location structure */
struct FieldLocation
{
    std::string tagName;
    double x;      // x-position (in meters)
    double y;      // y-position (in meters)
    double z;      // z-position (in meters)

    MarkerObservation toObservation()
    {
        double rho = sqrt(x * x + y * y + z * z);
        double phi = atan2(x, z);
        double theta = atan2(hypot(x, z), y);
        return MarkerObservation{tagName, rho, theta, phi};

    }

    friend std::ostream &operator<<(std::ostream &os, const FieldLocation &location) {
        os << "tagName: " << location.tagName << " x: " << location.x << " y: " << location.y << " z: " << location.z;
        return os;
    }

    FieldLocation operator - (const FieldLocation& other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return FieldLocation{tagName + "-" + other.tagName, dx, dy, dz};
    }

    FieldLocation operator + (const FieldLocation& other) const
    {
        double dx = x + other.x;
        double dy = y + other.y;
        double dz = z + other.z;
        return FieldLocation{tagName, dx, dy, dz};
    }

    FieldLocation operator * (double scale) const
    {
        double dx = x * scale;
        double dy = y * scale;
        double dz = z * scale;
        return FieldLocation{tagName, dx, dy, dz};
    }
};

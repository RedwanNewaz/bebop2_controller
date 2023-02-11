#pragma once
#include <iostream>
#include <string>
#include <math.h>

//const int STATE_DIM = 10;

/// @brief Structure defining the state of the drone.
struct RobotState
{
    /// x-position(in meters)
    double x; 
    /// y-position (in meters)     
    double y;
    /// z-position (in meters)      
    double z;
    /// z-orientation (in radians)      
    double theta;  
//    double vx;     // x-linear velocity (in meters)
//    double vy;     // y-linear velocity (in meters)
//    double vz;     // z-linear velocity (in meters)
//    double wx;     // x-angular velocity (in meters)
//    double wy;     // y-angular velocity (in meters)
//    double wz;     // z-angular velocity (in meters)
};


/// Field location structure in radial distance (r), azimuthal angle (theta), and polar angle (psi)
struct MarkerObservation
{
    std::string tagName;  // Index of observed marker [0-n]
    /// Observed distance to landmark from robot position    
    double radial_distance;    
    /// Observed bearing to landmark in the XY coordinate frame
    double azimuthal_angle; 
    /// Observed bearing to landmark in the positive Z axis
    double polar_angle; 


    /// @brief 
    /// @param os It is an output stream objects that writes sequences of characters
    /// @param observation  Reference to MarkerObservation struct.
    /// @return Returns the output of tagname, radial distance, azimuthal angle and polar angle.
    friend std::ostream &operator<<(std::ostream &os, const MarkerObservation &observation) {
        double degFactor = (180.0/3.141592653589793238463);
        os << "tagName: " << observation.tagName << " radial_distance: " << observation.radial_distance
           << " azimuthal_angle: " << observation.azimuthal_angle * degFactor << " polar_angle: " << observation.polar_angle * degFactor;
        return os;
    }

};


/// Field location structure
struct FieldLocation
{
    std::string tagName;
    /// x-position (in meters)
    double x;  
    /// y-position (in meters)    
    double y; 
    /// z-position (in meters)     
    double z;      

    /// @brief Calculates the radial distance, azimuthal angle and polar angle using x,y,z position.
    /// @return Returns the the structure of MarkerObservation with the spherical coordinates.
    MarkerObservation toObservation()
    {
        double rho = sqrt(x * x + y * y + z * z);
        double phi = atan2(x, z);
        double theta = atan2(hypot(x, z), y);
        return MarkerObservation{tagName, rho, theta, phi};

    }
    /// @param os It is an output stream object that writes sequences of characters.
    /// @param location Reference to the FieldLocation Struct.
    /// @return Returns the output of tagName, x, y and z positions in string format.
    friend std::ostream &operator<<(std::ostream &os, const FieldLocation &location) {
        os << "tagName: " << location.tagName << " x: " << location.x << " y: " << location.y << " z: " << location.z;
        return os;
    }


    /// @brief Calculates the difference between two tags.
    /// @param other is the other AprilTag.
    /// @return the struct with the new tagname, x,y and z position
    FieldLocation operator - (const FieldLocation& other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return FieldLocation{tagName + "-" + other.tagName, dx, dy, dz};
    }

    /// @brief Calculates the sum of two tags.
    /// @param other is the other AprilTag.
    /// @return the struct with the new tagname, x,y and z position
    FieldLocation operator + (const FieldLocation& other) const
    {
        double dx = x + other.x;
        double dy = y + other.y;
        double dz = z + other.z;
        return FieldLocation{tagName, dx, dy, dz};
    }

    /// @brief Calculates the product between two tags.
    /// @param other is the other AprilTag.
    /// @return the struct with the new tagname, x,y and z position
    FieldLocation operator * (double scale) const
    {
        double dx = x * scale;
        double dy = y * scale;
        double dz = z * scale;
        return FieldLocation{tagName, dx, dy, dz};
    }
};

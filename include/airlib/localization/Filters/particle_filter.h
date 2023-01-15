/*
 * particle_filter.h
 *
 * 2D particle filter class.
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <iostream>
#include <utility>
#include <vector>
#include <numeric>
#include "filters_common/helper.h"
#include <Eigen/Dense>



class TagMap {
public:

    struct single_landmark_s{

        int id_i ; // Landmark ID
        double x_d; // Landmark x-position in the map (global coordinates)
        double y_d; // Landmark y-position in the map (global coordinates)
        double z_d; // Landmark z-position in the map (global coordinates)
    };

    std::vector<single_landmark_s> landmark_list ; // List of landmarks in the map

};


class Mvn
{
public:
    Mvn(const TagMap::single_landmark_s & landmark,
        const std::vector<double>& std)
    {
        int N = std.size() - 1;
        mean.resize(N);
        mean(0) = landmark.x_d;
        mean(1) = landmark.y_d;
        mean(2) = landmark.z_d;

        sigma.resize(N, N);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                sigma(i, j) = (i == j) ? std[i] * std[j] : 0.0;
            }
        }

        sigma_inv = sigma.inverse();
        sigma_det = sigma.determinant();

    }
    ~Mvn()
    {

    }
    template <typename T>
    double pdf(const T& obs_m) const
    {
        Eigen::Vector3d x;
        x << obs_m.x, obs_m.y, obs_m.z;

        double n = x.rows();
        double sqrt2pi = std::sqrt(2 * M_PI);
        double quadform  = (x - mean).transpose() * sigma_inv * (x - mean);
        double norm = std::pow(sqrt2pi, - n) *
                      std::pow(sigma_det, - 0.5);

        return norm * exp(-0.5 * quadform);
    }

private:
    Eigen::VectorXd mean;
    Eigen::MatrixXd sigma;
    Eigen::MatrixXd sigma_inv;
    double sigma_det;
};


struct Particle {

    int id;
    double x;
    double y;
    double z;
    double theta;
    double weight;

    friend std::ostream &operator<<(std::ostream &os, const Particle &particle)
    {
        os << "id: " << particle.id << " x: " << particle.x << " y: " << particle.y << " z: " << particle.z << " theta: "
           << particle.theta << " weight: " << particle.weight;
        return os;
    }

};

/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {

    int id;				// Id of matching landmark in the map.
    double x;			// Local (vehicle coordinates) x position of landmark observation [m]
    double y;			// Local (vehicle coordinates) y position of landmark observation [m]
    double z;			// Local (vehicle coordinates) z position of landmark observation [m]
};



class ParticleFilter {

    // Number of particles to draw
    int num_particles;


    // Flag, if filter is initialized
    bool is_initialized;

    // Vector of weights of all particles
    std::vector<double> weights;

    const TagMap tagMap_;
    double delta_t;

    Twist * cmd_;
    std::vector<Mvn*> mvn_;
    std::vector<double> sigma_pos_;
    std::vector<double> xEst_;





public:

    // Set of current particles
    std::vector<Particle> particles;

    // Constructor
    // @param M Number of particles
    ParticleFilter(int num_particles, const TagMap&  tagMap, double delta_t) :
    num_particles(num_particles), tagMap_(tagMap), is_initialized(false), delta_t(0.03) {}
    void operator()(std::vector<double>& state);
    // Destructor
    ~ParticleFilter() {


    }

    void update_cmd(Twist *cmd)
    {
        cmd_ = cmd;
    }

    // interface
    void update(const std::vector<double>& obs, std::vector<double>& result);


    /**
     * init Initializes particle filter by initializing particles to Gaussian
     *   distribution around first position and all the weights to 1.
     * @param x Initial x position [m] (simulated estimate from GPS)
     * @param y Initial y position [m]
     * @param theta Initial orientation [rad]
     * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
     *   standard deviation of yaw [rad]]
     */
    void init(const std::vector<double>& x0, const std::vector<double>& std);

    /**
     * prediction Predicts the state for the next time step
     *   using the process model.
     * @param delta_t Time between time step t and t+1 in measurements [s]
     * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
     *   standard deviation of yaw [rad]]
     * @param velocity Velocity of car from t to t+1 [m/s]
     * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
     */
    void prediction(double delta_t, const std::vector<double>& std_pos, const Twist* cmd);

    /**
     * dataAssociation Finds which observations correspond to which landmarks (likely by using
     *   a nearest-neighbors data association).
     * @param predicted Vector of predicted landmark observations
     * @param observations Vector of landmark observations
     */
    void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations);

    /**
     * updateWeights Updates the weights for each particle based on the likelihood of the
     *   observed measurements.
     * @param sensor_range Range [m] of sensor
     * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
     *   standard deviation of bearing [rad]]
     * @param observations Vector of landmark observations
     * @param map Map class containing map landmarks
     */
    void updateWeights(double sensor_range, const std::vector<double>& std_landmark, std::vector<LandmarkObs> observations,
                       TagMap map_landmarks);

    /**
     * resample Resamples from the updated set of particles to form
     *   the new set of particles.
     */
    void resample();

    /*
     * write Writes particle positions to a file.
     * @param filename File to write particle positions to.
     */
    void write(std::string filename);

    /**
     * initialized Returns whether particle filter is initialized yet or not.
     */
    const bool initialized() const {
        return is_initialized;
    }

protected:


/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
    inline double dist(double x1, double y1, double z1, double x2, double y2, double z2) {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1) );
    }

    inline double * getError(double gt_x, double gt_y, double gt_z, double gt_theta, double pf_x, double pf_y, double pf_z, double pf_theta) {
        static double error[3];
        error[0] = fabs(pf_x - gt_x);
        error[1] = fabs(pf_y - gt_y);
        error[2] = fabs(pf_z - gt_z);
        error[3] = fabs(pf_theta - gt_theta);
        error[3] = fmod(error[2], 2.0 * M_PI);
        if (error[3] > M_PI) {
            error[3] = 2.0 * M_PI - error[3];
        }
        return error;
    }

};



#endif /* PARTICLE_FILTER_H_ */
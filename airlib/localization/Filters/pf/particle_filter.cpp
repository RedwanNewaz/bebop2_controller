//
// Created by Redwan Newaz on 12/27/22.
//

/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include "bebop2_controller/localization/Filters/particle_filter.h"

using namespace std;

void ParticleFilter::init(const std::vector<double>& x0, const std::vector<double>& std) {
    // Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    weights.resize(num_particles);
    particles.resize(num_particles);

    double std_x, std_y, std_z, std_theta; // Standard deviations for x, y, and theta
    std_x = std[0];
    std_y = std[1];
    std_z = std[2];
    std_theta = std[3];

    // Normal distribution for x, y and theta
    normal_distribution<double> dist_x(x0[0], std_x); // mean is centered around the new measurement
    normal_distribution<double> dist_y(x0[1], std_y);
    normal_distribution<double> dist_z(x0[2], std_z);
    normal_distribution<double> dist_theta(x0[3], std_theta);

    default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

    // create particles and set their values
    for(int i=0; i<num_particles; ++i){
        Particle p;
        p.id = i;
        p.x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
        p.y = dist_y(gen);
        p.z = dist_z(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0 / num_particles;

        particles[i] = p;
        weights[i] = p.weight;
    }

    sigma_pos_.clear();
    sigma_pos_.resize(std.size());

    std::copy(std.begin(), std.end(), sigma_pos_.begin());

    for(const auto& landmark: tagMap_.landmark_list)
        mvn_.emplace_back(new Mvn(landmark, sigma_pos_));

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, const std::vector<double>& std_pos, const Twist* cmd) {
    // Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    double std_x, std_y, std_z, std_theta; // Standard deviations for x, y, and theta
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_z = std_pos[2];
    std_theta = std_pos[3];

    default_random_engine gen;

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i]; // get address of particle to update

        // use the prediction equations from the Lesson 14
        double new_x = p->x + delta_t * cmd->linear.x;
        double new_y = p->y - delta_t * cmd->linear.y;
        double new_z = p->z + delta_t * cmd->linear.z;
        double new_theta = p->theta + delta_t * cmd->angular.z;

        // add Gaussian Noise to each measurement
        // Normal distribution for x, y and theta
        normal_distribution<double> dist_x(new_x, std_x);
        normal_distribution<double> dist_y(new_y, std_y);
        normal_distribution<double> dist_z(new_z, std_z);
        normal_distribution<double> dist_theta(new_theta, std_theta);

        // update the particle attributes
        p->x = dist_x(gen);
        p->y = dist_y(gen);
        p->z = dist_z(gen);
        p->theta = dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.
    for(auto pred : predicted){
        double dist_min = std::numeric_limits<double>::max();

        for(auto observation : observations){
            double distance = dist(observation.x, observation.y, observation.z, pred.x, pred.y, pred.z); // distance b/w obs and landmark
            if(distance < dist_min){
                observation.id = pred.id;

            }
            dist_min = distance;
        }
    }



}

void ParticleFilter::updateWeights(double sensor_range, const std::vector<double>& std_landmark,
                                   std::vector<LandmarkObs> observations, TagMap map_landmarks) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account
    //   for the fact that the map's y-axis actually points downwards.)
    //   http://planning.cs.uiuc.edu/node99.html


    double weights_sum = 0;

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i];
        double wt = 1.0;

        // step 1: collect valid landmarks
        std::vector<LandmarkObs> predictions;
        for(const auto& lm: map_landmarks.landmark_list){
            double distance = dist(p->x, p->y, p->z, lm.x_d, lm.y_d, lm.z_d);
            if( distance < sensor_range){ // if the landmark is within the sensor range, save it to predictions
                predictions.push_back(LandmarkObs{lm.id_i, lm.x_d, lm.y_d, lm.z_d});
            }
        }

        // convert observation from vehicle's to map's coordinate system
        std::vector<LandmarkObs> observations_map;

        for(const auto &current_obs : observations){
            LandmarkObs transformed_obs;
            // A yaw is a counterclockwise rotation of $ \theta about the $ z$-axis. The rotation matrix is given by
            // http://msl.cs.uiuc.edu/planning/node102.html
            // (cos(p->theta), -sin(p->theta), 0)
            // (sin(p->theta), cos(p->theta), 0)
            // (0, 0, 1)

            transformed_obs.x = (current_obs.x * cos(p->theta)) - (current_obs.y * sin(p->theta)) + p->x;
            transformed_obs.y = (current_obs.x * sin(p->theta)) + (current_obs.y * cos(p->theta)) + p->y;
            transformed_obs.z =  current_obs.z + p->z;
            transformed_obs.id = current_obs.id;
            observations_map.push_back(transformed_obs);

        }

        // step 3: find landmark index for each observation
        // find the predicted measurement that is closest to each observed measurement and assign
        // the observed measurement to this particular landmark
        dataAssociation(predictions, observations_map);
        // step 4: compute the particle's weight:
        // update weights using Multivariate Gaussian Distribution
        // implement multivariate gaussian
        for(const auto& obs_m: observations_map){
            wt *= mvn_[obs_m.id]->pdf(obs_m);
            weights_sum += wt;
            p->weight = wt;
        }
    }
    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        Particle *p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }
}

void ParticleFilter::resample() {
    // Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    default_random_engine gen;

    // Random integers on the [0, n) range
    // the probability of each individual integer is its weight of the divided by the sum of all weights.
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles;

    for (int i = 0; i < num_particles; i++){
        resampled_particles.push_back(particles[distribution(gen)]);
    }

//    particles = resampled_particles;
    std::copy(resampled_particles.begin(), resampled_particles.end(), particles.begin());
}

void ParticleFilter::write(std::string filename) {
    // You don't need to modify this file.
    std::ofstream dataFile;
    dataFile.open(filename, std::ios::app);
    for (int i = 0; i < num_particles; ++i) {
        dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
    }
    dataFile.close();
}

void ParticleFilter::update(const std::vector<double>& obs, std::vector<double>& result) {

    assert(initialized() && "particle filter is not initialized");
    assert(obs.size() >= 3 && "observation vector must be greater than 3");

    std::vector<LandmarkObs> noisy_observations;
    int count = 0;
    for (const auto& landmark: tagMap_.landmark_list) {
       std::vector<double> landmarkVec{landmark.x_d, landmark.y_d, landmark.z_d};
       std::vector<double> rel_obs(obs.size() + 1);

       rel_obs[0] = count++;
       int i = 0;
       std::transform(landmarkVec.begin(), landmarkVec.end(), rel_obs.begin() + 1,[&](double v){ return v - obs[i++];});
       noisy_observations.push_back({(int)rel_obs[0], rel_obs[1], rel_obs[2], rel_obs[3]});
    }





    // Predict the vehicle's next state (noiseless).
    /*
   * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
   * if you used fused data from multiple sensors, it's difficult to find
   * these uncertainties directly.
   */

//    double sigma_pos [4] = {0.015, 0.015, 0.015, 0.01}; // GPS measurement uncertainty [x [m], y [m], z [m], theta [rad]]


    prediction(delta_t, sigma_pos_, cmd_);

    // Update the weights and resample
    double sensor_range = 15; // Sensor range [m]


    updateWeights(sensor_range, sigma_pos_, noisy_observations, tagMap_);
    resample();


    // Calculate and output the average weighted error of the particle filter over all time steps so far.
    int countParticles = 0;
    for(const auto&p : particles)
    {
        if(p.weight < 1e-3)
            continue;


        result[0] += p.x;
        result[1] += p.y;
        result[2] += p.z;
        result[3] += p.theta;
        ++countParticles;
    }



    countParticles = (countParticles == 0) ? 1 : countParticles;
    std::transform(result.begin(), result.end(), result.begin(), [&countParticles](double p){ return p /(double) countParticles;});

    if(countParticles > 1)
    {
        xEst_.clear();
        std::copy(result.begin(), result.end(), std::back_inserter(xEst_));
    }
}

void ParticleFilter::operator()(std::vector<double> &state) {
    std::copy(xEst_.begin(), xEst_.end(), state.begin());

}


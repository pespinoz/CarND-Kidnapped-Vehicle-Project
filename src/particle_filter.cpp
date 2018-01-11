/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    // Number of particles
    num_particles = 100;
    // Initialize random generator
    default_random_engine gen;

    // Set normal distributions to draw particles' pose from.
    normal_distribution<double> N_x(x, std[0]);
    normal_distribution<double> N_y(y, std[1]);
    normal_distribution<double> N_theta(theta, std[2]);

    // Drawing from previous distributions, initialize the Particle structure fields.
    for(int i = 0; i < num_particles; i++){
        Particle particle;
        particle.id = i;
        particle.x = N_x(gen);
        particle.y = N_y(gen);
        particle.theta = N_theta(gen);
        particle.weight = 1;

        particles.push_back(particle);
        //weights.push_back(particle.weight);
    }
    // Flag, this will only run in the first time step
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // Initialize random generator
    default_random_engine gen;

    // For each particle, we want to follow its trajectory through the following state transition:
    for(int i = 0; i < num_particles; i++){
        double x;
        double y;
        double theta;

        // There are two cases, i) for yaw rate = 0 (or very small in c++, if not I could get singularities)
        if(fabs(yaw_rate) < 0.0001){
            x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
            theta = particles[i].theta;
        }
        // And ii) for yaw_rate != 0. These equations were derived in the Motion Models lesson in the classroom.
        else{
            x = particles[i].x + velocity/yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            y = particles[i].y + velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
            theta = particles[i].theta + yaw_rate * delta_t;
        }

        // This evolution through the state transition is probabilistic:
        // We add the GPS noise to the particles's pose again. If not, PF will not work (and there wouldn't
        // really be any uncertainty in the prediction step, defeating the purpose of a Bayesian-based filter)
        normal_distribution<double> N_x(x, std_pos[0]);
        normal_distribution<double> N_y(y, std_pos[1]);
        normal_distribution<double> N_theta(theta, std_pos[2]);
        particles[i].x = N_x(gen);
        particles[i].y = N_y(gen);
        particles[i].theta = N_theta(gen);
    }
}

void ParticleFilter::dataAssociation(Particle &particle, double range, std::vector<LandmarkObs> predicted,
                                     const Map &map_landmarks) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

/*
    for (int j = 0; j < predicted.size(); j++){
        double min_distance = range;
        int associate = 0;
        for (int k = 0; k < map_landmarks.landmark_list.size(); k++){
            double distance;
            distance = sqrt(pow(predicted[j].x - map_landmarks.landmark_list[k].x_f, 2.0) +
                            pow(predicted[j].y - map_landmarks.landmark_list[k].y_f, 2.0));
            if (distance < min_distance){
                min_distance = distance;
                associate = k;
            }
        }
        particle.associations.push_back(associate);
    }
*/
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    // TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html

    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    vector<LandmarkObs> observations_map;

    for (int i = 0; i < num_particles; i++) {

        // For each particle I transform the observations from the car (as seen by the particle)
        // to the map coordinate system. This is done via a rotation by the particle's orientation and
        // a translation to the particle's (x,y) coordinates.
        for (int j = 0; j < observations.size(); j++){
            LandmarkObs obs_m;
            obs_m.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
            obs_m.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
            observations_map.push_back(obs_m);
        }

        // Now that I have the observations transformed in the map coordinate system, I look
        // for the corresponding landmark for each observation (by nearest neighbor)
        // I could also have implemented this in the ParticleFilter::dataAssociation()
        particles[i].weight = 1.0;

        for (int j = 0; j < observations_map.size(); j++){
            double min_distance = sensor_range;
            int associate = 0;
            for (int k = 0; k < map_landmarks.landmark_list.size(); k++){
                double distance;
                distance = sqrt(pow(observations_map[j].x - map_landmarks.landmark_list[k].x_f, 2.0) +
                                pow(observations_map[j].y - map_landmarks.landmark_list[k].y_f, 2.0));
                if (distance < min_distance){
                    min_distance = distance;
                    associate = k;
                }
            }
            // This is the vector that contains the map's landmark indices that correspond to each observation.
            associations.push_back(associate);
            sense_x.push_back(observations_map[j].x);
            sense_y.push_back(observations_map[j].y);

            double meas_x = observations_map[j].x;
            double meas_y = observations_map[j].y;
            double mu_x = map_landmarks.landmark_list[associations[j]].x_f;
            double mu_y = map_landmarks.landmark_list[associations[j]].y_f;

            // Calculate the measurement probability:
            long double mvgd = 1/(2 * M_PI * std_landmark[0] * std_landmark[1]) * exp(-(pow(meas_x - mu_x, 2) / pow(M_SQRT2 * std_landmark[0], 2) + pow(meas_y - mu_y, 2) / pow(M_SQRT2 * std_landmark[1], 2)));

            // The particle's final weight is given by multiplying all the probabilities together.
            if(mvgd > 0.0) {
                particles[i].weight *= mvgd;
            }
        }
        // Important, didn't see this for a while
        associations.clear();
        observations_map.clear();
    }
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // Initialize random generator
    default_random_engine gen;

    // Put together a weights vector for a generation of particles (100 particles at a given time step)
    vector<double> weights;
    for (int i=0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }

    // Build the discrete distribution:
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles;

    // And then draw from the distribution with probability proportional to the original weights.
    for(int i = 0; i < num_particles; i++){
        resampled_particles.push_back(particles[distribution(gen)]);
    }
    particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    /*
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    */
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

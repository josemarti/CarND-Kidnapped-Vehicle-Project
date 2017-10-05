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

//Random engine
default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    
    
    num_particles = 150;  //To be tuned
    
    
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    for(int i = 0; i<num_particles; i++){
        
        Particle p;
        p.id = i;
        p.weight = 1.0;

        
        //To add noise
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        
        weights.push_back(1.0);//****************
        particles.push_back(p);
    }
    
    
    is_initialized = true;
    
    
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    
    
    
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
    
    for(int i = 0; i<num_particles; i++){
        double x0 = particles[i].x;
        double y0 = particles[i].y;
        double theta0 = particles[i].theta;
        double thetaf = theta0 + yaw_rate * delta_t;
        
        
        //CAlculates the new state
        if(fabs(yaw_rate) > 0.00001){
            particles[i].x = x0 + velocity/yaw_rate * (sin(thetaf)-sin(theta0));
            particles[i].y = y0 + velocity/yaw_rate * (cos(theta0)-cos(thetaf));
            particles[i].theta = thetaf;
        }else{
            particles[i].x = x0 + velocity * sin(theta0) * delta_t;
            particles[i].y = y0 + velocity * cos(theta0) * delta_t;
            particles[i].theta = theta0;
        }
        
        
        //Add noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
    
    
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    
    
    weights.clear();
    
    //every particle
    for(int i = 0; i < num_particles; ++i){
        
  
        
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;
        
        //to store locations in range
        vector<LandmarkObs> predicted;
        
        
        
        //Perform the space transformation from vehicle to map
        particles[i].associations.clear();
        particles[i].sense_x.clear();
        particles[i].sense_y.clear();
        double weight = 1;
    


        for(int j = 0; j<observations.size(); j++){
            double meas_x = observations[j].x;
            double meas_y = observations[j].y;
            double meas_x_map = meas_x * cos(p_theta) - meas_y * sin(p_theta) + p_x;
            double meas_y_map = meas_x * sin(p_theta) + meas_y * cos(p_theta) + p_y;
            
            if(pow(pow(meas_x_map-p_x,2)+pow(meas_y_map-p_y,2),0.5) > sensor_range) continue;
            

            particles[i].sense_x.push_back(meas_x_map);
            particles[i].sense_y.push_back(meas_y_map);
            

            double min_range = 1000000; 
            int min_k=-1;
            
            for(int k = 0; k<map_landmarks.landmark_list.size(); k++){

                double mu_x = map_landmarks.landmark_list[k].x_f;
                double mu_y = map_landmarks.landmark_list[k].y_f;
                double diff_x = mu_x - meas_x_map;
                double diff_y = mu_y - meas_y_map;
                double range = pow(pow(diff_x,2)+pow(diff_y,2),0.5);
               
                if(range < min_range){
                    min_range = range;
                    min_k = k;
                }
            }
            
            
            
            
            double mu_x = map_landmarks.landmark_list[min_k].x_f;  
            double mu_y = map_landmarks.landmark_list[min_k].y_f;
            
            particles[i].associations.push_back(map_landmarks.landmark_list[min_k].id_i);
            
            weight = weight * exp(-0.5 * (pow((mu_x - meas_x_map) / std_landmark[0],2) + pow((mu_y - meas_y_map) / std_landmark[1],2))) / (2*M_PI*std_landmark[0]*std_landmark[1]);

    
        }
        particles[i].weight=weight;
        weights.push_back(weight);
    }
    

    
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    
    
    default_random_engine gen;
    discrete_distribution<int> distribution(weights.begin(), weights.end());

    vector<Particle> resampled_particles;
    
    weights.clear();
    
    for(int i=0; i < num_particles; i++){

        int chosen = distribution(gen); 

        resampled_particles.push_back(particles[chosen]);
        weights.push_back(particles[chosen].weight); 
    }
    
    particles=resampled_particles;
    

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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

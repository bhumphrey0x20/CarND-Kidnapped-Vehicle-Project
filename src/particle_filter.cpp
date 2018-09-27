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

	default_random_engine gen;

	normal_distribution<double> dist_x(x,std[0]);
	normal_distribution<double> dist_y(y,std[1]);
	normal_distribution<double> dist_theta(theta,std[2]);


	for(int i = 0; i< num_particles; i++){
		double x = dist_x(gen);
		double y = dist_y(gen);
		double theta = dist_theta(gen);
		struct Particle part; 
		part.id = i;
		part.x = x;
		part.y = y;
		part.theta = theta;
		part.weight = 1.0;
		particles.push_back(part);

		weights.push_back(1.0);
	}
/* debug 
	cout << "weights.size(): " << weights.size() << endl;

	cout << "Particles: "<< endl;
	for(int i = 0; i< num_particles; i++){
		cout << "Particle " << particles[i].id << '\t' << particles[i].x << '\t' <<particles[i].y << '\t' <<particles[i].theta << '\t' << endl;
	}
*/
	is_initialized = true;
} // init()



void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


//debug
static int cnt = 0;


	/*** calc new pos x, y and new theta based on kinematic eq'ns ***/

	// first check for divide-by-zero
	if (yaw_rate == 0){
		for(int i = 0; i< num_particles; i++){
			double x_0 = particles[i].x;
			double y_0 = particles[i].y;
			double theta_0 = particles[i].theta;

			double x_f = x_0 + ( (velocity * delta_t) * sin(theta_0) );
			double y_f = y_0 + ( (velocity * delta_t) * cos(theta_0) );

			default_random_engine gen;
			normal_distribution<double> dist_x(x_f,std_pos[0]);
			normal_distribution<double> dist_y(y_f,std_pos[1]);
			normal_distribution<double> dist_theta(theta_0,std_pos[2]);

			particles[i].x = dist_x(gen); 
			particles[i].y = dist_y(gen); 
			particles[i].theta = dist_theta(gen);

		}
	}else{
		for(int i = 0; i< num_particles; i++){
			double x_0 = particles[i].x;
			double y_0 = particles[i].y;
			double theta_0 = particles[i].theta;
			double coeff = velocity/yaw_rate;

			double x_f = x_0 + coeff * ( sin(theta_0 + yaw_rate * delta_t) - sin(theta_0) );
			double y_f = y_0 + coeff * ( cos(theta_0) - cos(theta_0 + yaw_rate * delta_t) );
			double theta_f = theta_0 + yaw_rate * delta_t;
			default_random_engine gen;

			normal_distribution<double> dist_x(x_f,std_pos[0]);
			normal_distribution<double> dist_y(y_f,std_pos[1]);
			normal_distribution<double> dist_theta(theta_f,std_pos[2]);


			particles[i].x = dist_x(gen); 
			particles[i].y = dist_y(gen); 
			particles[i].theta = dist_theta(gen); 
		}
		
// debug
if(cnt < 3){
	for(int i = 0; i< num_particles; i++){
		cout << "Particle " << particles[i].id << '\t' << particles[i].x << '\t' <<particles[i].y << '\t' <<particles[i].theta << '\t' << endl;
	}
cout << endl;
cnt++;
}


	}


} //prediction




void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	cout << "predicted.size(): " << predicted.size() << endl;
	cout << "observations.size(): " << observations.size() << endl;
	
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

	// Filter out objects farther than sensor_range
	vector<LandmarkObs> near_objects;  
	LandmarkObs lm; 

	for(unsigned int i = 0; i< observations.size(); i++){
		double x = observations[i].x;
		double y = observations[i].y;
		double radius = sqrt(x*x + y*y); 
		if(radius <= sensor_range){
			lm.id = i;
			lm.x = x;
			lm.y = y; 
			near_objects.push_back(lm);
		}
	}
	cout << "near_objects.size(): " << observations.size() << endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	/*** taken from https://www.youtube.com/watch?v=-3HI3Iw3Z9g&feature=youtu.be min 15:15 ***/

	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles;
	for(int i = 0; i< num_particles; i++){
		resample_particles.push_back(particles[distribution(gen)]);
	}
	
	particles = resample_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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

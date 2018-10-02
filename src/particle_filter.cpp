/*
 * particle_filter.cpp
 *
 *  Starter code created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *	
 *  Project code creaded by BH 2018-9-25
 *
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



/*** comparator function for sort() algorithm 
		borrowed from https://www.geeksforgeeks.org/sorting-a-vector-in-c/
***/
bool compareVector(LandmarkObs l1, LandmarkObs l2){
	return l1.vector < l2.vector;
}


/*** gaussian_pdf(), 
	returns probability of a bi-variate gaussian probability distribution function
***/

double gaussian_pdf(double x, double y, double mu_x, double mu_y, double sigma_x, double sigma_y){
		
	double norm = 1/(2.0 * M_PI * sigma_x * sigma_y);	
	double prob = exp(-( pow((x-mu_x),2)/(2.0*pow(sigma_x,2)) +  pow((y-mu_y),2)/(2.0*pow(sigma_y, 2)) )  );

	return norm * prob;
}




void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// 

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
	}

	is_initialized = true;
} // init()



void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


//*** calc new pos x, y and new theta based on kinematic eq'ns **

	default_random_engine gen;
	normal_distribution<double> dist_x(0,std_pos[0]);
	normal_distribution<double> dist_y(0,std_pos[1]);
	normal_distribution<double> dist_theta(0,std_pos[2]);


	for(unsigned int i = 0; i< particles.size(); i++){

			double x_0 = particles[i].x;
			double y_0 = particles[i].y;
			double theta_0 = particles[i].theta;
			double x_f(0.0), y_f(0.0);

		// first check for divide-by-zero
		if (yaw_rate == 0){
			x_f = x_0 + ( (velocity * delta_t) * cos(theta_0) );
			y_f = y_0 + ( (velocity * delta_t) * sin(theta_0) );
		}else{
			double coeff = velocity/yaw_rate;

			x_f = x_0 + coeff * ( sin(theta_0 + yaw_rate * delta_t) - sin(theta_0) );
			y_f = y_0 + coeff * ( cos(theta_0) - cos(theta_0 + yaw_rate * delta_t) );
			theta_0 = theta_0 + yaw_rate * delta_t;
		}

			particles[i].x = x_f + dist_x(gen); 
			particles[i].y = y_f + dist_y(gen); 
			particles[i].theta = theta_0 + dist_theta(gen); 
		
	}

} //prediction




void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	//  Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.

		for(unsigned int i = 0; i< observations.size(); i++){
			double o_x = observations[i].x; 
			double o_y = observations[i].y; 
			int id =0; 
			double curr_dist = 0.0;
			double prev_dist = numeric_limits<double>::max();
			for (unsigned int j= 0; j< predicted.size(); j++){
				double x = predicted[j].x; 
				double y = predicted[j].y; 
				curr_dist = dist(x, y, o_x, o_y); 
				if (curr_dist < prev_dist){
					prev_dist = curr_dist;
					id = predicted[j].id;
				}
			}
			observations[i].id = id; 
	}
} // dataAssociation



void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. 


// *** this function was tested, debugged, and in the end modeled after the equivalent function here:
//  	https://github.com/JunshengFu/kidnapped-vehicle/blob/master/src/particle_filter.cpp 
//    the final result is code that was reworked so much it is a virtual copy of the referenced code- bh


	for(unsigned int i= 0; i< particles.size(); i++){
    particles[i].weight = 1.0;
		double part_theta = particles[i].theta;

		// filter out landmarks beyond sensor range
    vector<LandmarkObs> near_landmarks;
		for(unsigned int j = 0; j< map_landmarks.landmark_list.size(); j++){

      double lm_vector = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
      if( lm_vector < sensor_range){ // if the landmark is within the sensor range, save it to predictions
        near_landmarks.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f});
      }
    }

		// convert observation measurements to particle coordinates
    vector<LandmarkObs> tx_observations;
		for(unsigned int j=0; j < observations.size(); j++){
      LandmarkObs lm;
			lm.x = observations[j].x * cos(part_theta) - observations[j].y * sin(part_theta) + particles[i].x;
			lm.y = observations[j].x * sin(part_theta) + observations[j].y * cos(part_theta) + particles[i].y;

      tx_observations.push_back(lm);
    }

		// Associate observtaions with landmarks
    dataAssociation(near_landmarks, tx_observations);

		// Calculation weights for each particle-observation match using bivariate gaussian pdf
		double prob = 1.0;
		for(unsigned int k = 0; k < tx_observations.size(); k++){

      Map::single_landmark_s landmark = map_landmarks.landmark_list[tx_observations[k].id-1];

			prob *=  gaussian_pdf(tx_observations[k].x, tx_observations[k].y, landmark.x_f, landmark.y_f, std_landmark[0], std_landmark[1]);		

    }
      particles[i].weight =  prob;
    weights.push_back(particles[i].weight);

	}
} // updateWeights()



void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


//*** taken from https://www.youtube.com/watch?v=-3HI3Iw3Z9g&feature=youtu.be at min. 15:15 

	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles;
	for(unsigned int i = 0; i< particles.size(); i++){
		resample_particles.push_back(particles[distribution(gen)]);
	}
	
	particles = resample_particles;
	weights.clear();

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

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
	num_particles = 1e3;  // TODO: Set the number of particles
	std::default_random_engine gen; // from <random>
	std::normal_distribution<double> dist_x(x, std[0]); // from <random>
	std::normal_distribution<double> dist_y(y, std[1]); // from <random>
	std::normal_distribution<double> dist_theta(theta, std[2]); // from <random>

	//initialize particles and their weights
	for(int i = 0; i<num_particles; i++)
	{
		Particle new_particle;
		new_particle.id = i;
		new_particle.x = dist_x(gen);
		new_particle.y = dist_y(gen);
		new_particle.theta = dist_theta(gen);
		new_particle.weight = 1.0;
		particles.push_back(new_particle);
		weights.push_back(1.0);
	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine gen; // from <random>
	std::normal_distribution<double> dist_x(0, std_pos[0]); // from <random>
	std::normal_distribution<double> dist_y(0, std_pos[1]); // from <random>
	std::normal_distribution<double> dist_theta(0, std_pos[2]); // from <random>

	//debug
//	std::cout<<yaw_rate<<std::endl;

	for(int i = 0; i<num_particles; i++)
	{
		if(fabs(yaw_rate)<=std::numeric_limits<double>::epsilon())
		{
			particles[i].x += velocity*cos(particles[i].theta)*delta_t + dist_x(gen);
			particles[i].y += velocity*sin(particles[i].theta)*delta_t + dist_y(gen);
		}
		else
		{
			particles[i].x += velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta)) + dist_x(gen);
			particles[i].y += velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t)) + dist_y(gen);
		}
		particles[i].theta += yaw_rate*delta_t + dist_theta(gen);

	}


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	double dist_min; //store the minimum distance between landmark and observation
	int id_match; //store the matched id
	for(uint i = 0; i< observations.size(); i++)
	{
		//obs = observations[i];
		dist_min = std::numeric_limits<double>::max();
		for(uint j = 0; j<predicted.size(); j++)
		{
			double distance;
			//distance = (sqrt(pow(predicted[j].x-observations[i].x, 2)+pow(predicted[j].y-observations[i].y, 2)));
			distance = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
			if (distance<dist_min)
			{
				dist_min = distance;
				id_match = predicted[j].id;
			}
		}
		observations[i].id = id_match;
	}

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
	double sum_weight = 0;

	//debug
	//cout<<"observation vector size: "<<observations.size()<<endl;

	for(int p = 0; p < num_particles; p++) //loop through particles to update each particle weight
	{
		// current particle is: particles[p]
		double xp, yp, theta; //particle's x, y, and theta in map coordinates
		xp = particles[p].x;
		yp = particles[p].y;
		theta = particles[p].theta;

		//convert coordinate in observations to map coordinates
		vector<LandmarkObs> obs_newcoordinate;
		for(uint i = 0; i< observations.size(); i++)
		{
			LandmarkObs obs;
			obs = observations[i];
			double new_x, new_y;
			new_x = obs.x*cos(theta) - obs.y*sin(theta) +xp;
			new_y = obs.x*sin(theta) + obs.y*cos(theta) +yp;
			obs.x = new_x;
			obs.y = new_y;
			obs_newcoordinate.push_back(obs);
		}

		//data association: assign landmark id to each observations
		//first only get landmarks within range of car, to make search more efficient
		std::vector<LandmarkObs> predicted;
		for(uint i = 0; i< map_landmarks.landmark_list.size(); i++)
		{
			int id;
			double x, y;
			//if(pow(map_landmarks.landmark_list[i].x_f-xp, 2)+pow(map_landmarks.landmark_list[i].y_f-yp, 2)< pow(sensor_range, 2))
			if(dist(xp, yp, map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f)<sensor_range)
			{
				id = map_landmarks.landmark_list[i].id_i;
				x = map_landmarks.landmark_list[i].x_f;
				y = map_landmarks.landmark_list[i].y_f;
				predicted.push_back({id, x, y});
			}
		}

		dataAssociation(predicted, obs_newcoordinate);

		double weight;
		weight = 1; //initialize weight to 1


		for(uint i=0; i< obs_newcoordinate.size(); i++)
		{
			//calculate weight based on multi-variate gaussian
			double landmarkx = map_landmarks.landmark_list[obs_newcoordinate[i].id-1].x_f;
			double landmarky = map_landmarks.landmark_list[obs_newcoordinate[i].id-1].y_f;

			weight *= multiv_prob(std_landmark[0], std_landmark[1], obs_newcoordinate[i].x, obs_newcoordinate[i].y, landmarkx, landmarky);
		}

		//update particle weight to the measurement likelihood X prior (previous weight)
		particles[p].weight = weight;
		weights[p] = particles[p].weight;
		sum_weight += weight;
	}

//	//debug
//	cout<<"sum_weight: "<<sum_weight<<endl;

//	//normalize the weight, this seems unnecessary since the resample step will normalize the probability anyway
//	for(int p = 0; p < num_particles; p++)
//	{
//		particles[p].weight /=sum_weight;
//		weights[p] = particles[p].weight; //store all weights in vector
//	}


}

double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
        double mu_x, double mu_y)
{
	// calculate normalization term
	  double gauss_norm;
	  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

	  // calculate exponent
	  double exponent;
	  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
	               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

	  // calculate weight using normalization terms and exponent
	  double weight;
	  weight = gauss_norm * exp(-exponent);

	  return weight;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<Particle> newParticles;

//	//==========Method A: my implementation of Sebastian's turning wheel method===========
//	double maxWeight, beta;
//	maxWeight = *max_element(weights.begin(), weights.end()); //get maximum weight value
//	beta = 0;
//	int index = int((double) rand() / (RAND_MAX)*num_particles); //random starting index
//	for(int i=0; i< num_particles; i++)
//	{
//		beta += ((double) rand() / (RAND_MAX)) * 2.0 * maxWeight;  //randomly add 2*maxWeight*alpha, alpha ~(0, 1) uniform distribution
//		while(beta>weights[index])
//		{
//			beta -= weights[index];
//			index = (index+1)%num_particles;
//		}
//		newParticles.push_back(particles[index]);
//	}
//	particles = newParticles;

	//==========Method B: use std::discrete_distribution=================
	std::random_device rd;
	std::mt19937 gen(rd());
	std::discrete_distribution<> d(weights.begin(), weights.end());
	for(int p = 0; p < num_particles; p++) //loop through particles to update each particle weight
	{
		int index;
		index = d(gen);
		newParticles.push_back(particles[index]);
	}
	particles = newParticles;
	//=============================================================

//	//reset all weights back to 1
//	for(int p = 0; p < num_particles; p++)
//	{
//		particles[p].weight = 1.0;
//		weights[p] = 1.0;
//	}



}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
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

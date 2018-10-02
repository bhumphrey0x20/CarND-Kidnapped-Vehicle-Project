 // MINE
/*
// *** this function was tested, debugged, and modeled after the equivalent function here:
//  	https://github.com/JunshengFu/kidnapped-vehicle/blob/master/src/particle_filter.cpp  

	LandmarkObs lm; 
	vector<LandmarkObs> near_landmarks; 	
	vector<LandmarkObs> tx_observations; 	

	for(unsigned int i= 0; i< particles.size(); i++){
		double part_x = particles[i].x;
		double part_y = particles[i].y;
		double part_theta = particles[i].theta;
		particles[i].weight = 1.0;

		// filter out landmarks beyond sensor range
		for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++){
			lm.id = map_landmarks.landmark_list[j].id_i;
			lm.x = map_landmarks.landmark_list[j].x_f;
			lm.y = map_landmarks.landmark_list[j].y_f;
			double vector = dist(part_x, part_y, lm.x, lm.y);
			if( vector < sensor_range){
				near_landmarks.push_back(lm); 
			}
		} // done filtering landmarks




		// convert observation measurements to particle coordinates
		for(unsigned int j = 0; j < observations.size(); j++){
			LandmarkObs tx_ob; 
			double ob_x = observations[j].x; 
			double ob_y = observations[j].y;
			
			tx_ob.x = part_x + (ob_x * cos(part_theta) ) - (ob_y * sin(part_theta));
			tx_ob.y = part_y + (ob_x * sin(part_theta) ) + (ob_y * cos(part_theta));

			tx_observations.push_back(tx_ob); 
		}// end coor transformation


		double cos_theta = cos(part_theta);
    double sin_theta = sin(part_theta);

    for(const auto& obs: observations){
      LandmarkObs tmp;
      tmp.x = obs.x * cos_theta - obs.y * sin_theta + part_x;
      tmp.y = obs.x * sin_theta + obs.y * cos_theta + part_y;
      //tmp.id = obs.id; // maybe an unnecessary step, since the each obersation will get the id from dataAssociation step.
      tx_observations.push_back(tmp);
		}

		// Associate observtaions with landmarks
		dataAssociation(near_landmarks, tx_observations); 
	
		double prob = 1.0;
		// Calculation weights for each particle-observation match using bivariate gaussian cdf
		for(const auto& obs_m: tx_observations){

      Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_m.id-1);


			prob = gaussian_cdf(obs_m.x, obs_m.y, landmark.x_f, landmark.y_f, std_landmark[0], std_landmark[1]);
      particles[i].weight *=  prob;


		}// for tx_obserations

//		particles[i].weight = prob; 
		weights.push_back(particles[i].weight); 

	}// end for particles
*/

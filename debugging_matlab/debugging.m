clear all; close all;

% This helps for the debugging of the project in C++. For the FIRST time
% step. Place stops in main.cpp and particle_filter.cpp

% INPUT: standard deviations used in the 2d gaussian distribution
sigmax = 0.3;
sigmay = 0.3;

% INPUT: observations from the car, in the car's system.
% i can get this from strings in the main.cpp, if i place a stop before the
% updateWeights() function
noisy_observations_x = [2.3588 11.5482 -20.0740 1.4307 14.3349 29.8290 -13.4155 29.7009 -35.9887 22.2778 -45.7963];
noisy_observations_y = [5.7484 -7.0705 -2.4903 -23.0386 -22.7605 13.0467 -36.6965 -30.9334 -25.1533 -41.6549 -15.1071]; 

% INPUT: pose for a given particle:
x= 5.9670545867215932;
y= 1.5997432551445911;
theta = 0.011348355977227824;

% INPUT: landmarks in global (map) coordinates.
A = load('map.txt');
landmark_x = A(:,1);
landmark_y = A(:,2);
clear A aux i;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% perform rotation and translation to put the car's observations in the map
% coordinates:

M = [cos(theta) -sin(theta) x; sin(theta) cos(theta) y; 0 0 1];
for i=1:1:11
    aux = M*[noisy_observations_x(i) noisy_observations_y(i) 1]';
    obs_map_x(i) = aux(1);
    obs_map_y(i) = aux(2);
end
    

figure;
plot(landmark_x, landmark_y, 'b.'); hold on;         % plot landmarks
plot(obs_map_x, obs_map_y, 'r.'); hold on;           % plot obs in map system
  
% in c++ i computed the associations (by nearest neighbor). I want to plot
% them here to make sure I'm associating observations to landmarks
% correctly.
% given a stop in particle_filter.cpp, I obtain these associations:
inds = [31 2 39 38 37 11 3 23 16 15 12];
plot(landmark_x(inds+1)+0.1, landmark_y(inds+1)+0.1, 'g.'); hold on;

% finally I just compute the weight for the particle:
for i=1:1:11
    x = obs_map_x(i);
    y = obs_map_y(i);
    mux = landmark_x(inds(i)+1);
    muy = landmark_y(inds(i)+1);
    % each observation from particle's perspective carries a weight.
    multi(i) = (1/(2*pi*sigmax*sigmay))*exp(-(((x-mux)/(sqrt(2)*sigmax))^2 + ((y-muy)/(sqrt(2)*sigmay))^2));
end    
prod(multi) % particle's final weight, just multiply all of them   
    








% /*
% // Let's update weights
% for (int j = 0; j < observations_map.size(); j++){
%     double meas_x = observations_map[j].x;
%     double meas_y = observations_map[j].y;
%     double mu_x = map_landmarks.landmark_list[p.associations[j]].x_f;
%     double mu_y = map_landmarks.landmark_list[p.associations[j]].y_f;
% 
% 
%     long double multipler = 1/(2 * M_PI * std_landmark[0] * std_landmark[1]) * exp(-(pow(meas_x - mu_x, 2) / pow(M_SQRT2 * std_landmark[0], 2) + pow(meas_y - mu_y, 2) / pow(M_SQRT2 * std_landmark[1], 2)));
% 
%     if(multipler > 0){
%         p.weight *= multipler;
%     }
% }
% 
% 
% /*
%     vector<int> associations;
%     vector<double> sense_x;
%     vector<double> sense_y;
%     vector<LandmarkObs> trans_observations;
%     LandmarkObs obs;
% 
%     for(int i = 0; i < observations.size(); i++){
%         LandmarkObs trans_obs;
%         obs = observations[i];
% 
%         trans_obs.x = particles[p].x + (obs.x * cos(particles[p].theta) - obs.y * sin(particles[p].theta));
%         trans_obs.y = particles[p].y + (obs.x * sin(particles[p].theta) + obs.y * cos(particles[p].theta));
%         trans_observations.push_back(trans_obs);
%     }
% 
%     particles[p].weight = 1.0;
% 
%     for(int i = 0; i < trans_observations.size(); i++){
%         double closet_dis = sensor_range;
%         int association = 0;
% 
%             for(int j = 0; j < map_landmarks.landmark_list.size(); j++){
%                 double landmark_x = map_landmarks.landmark_list[j].x_f;
%                 double landmark_y = map_landmarks.landmark_list[j].y_f;
% 
% 
%                 double calc_dist = sqrt(pow(trans_observations[i].x - landmark_x, 2.0) + pow(trans_observations[i].y - landmark_y, 2.0));
%                 if(calc_dist < closet_dis){
%                     closet_dis = calc_dist;
%                     association = j;
%                 }
% 
% 
%                 double meas_x = trans_observations[i].x;
%                 double meas_y = trans_observations[i].y;
%                 double mu_x = map_landmarks.landmark_list[association].x_f;
%                 double mu_y = map_landmarks.landmark_list[association].y_f;
%                 long double multipler = 1/(2 * M_PI * std_landmark[0] * std_landmark[1]) * exp(-(pow(meas_x - mu_x, 2) / pow(M_SQRT2 * std_landmark[0] , 2) + pow(meas_y - mu_y, 2) / pow(M_SQRT2 * std_landmark[1] , 2)));
%                 if(multipler > 0){
%                     particles[p].weight *= multipler;
%                 }
%             }
%             associations.push_back(association + 1);
%             sense_x.push_back(trans_observations[i].x);
%             sense_y.push_back(trans_observations[i].y);
%         }
%         particles[p] = SetAssociations(particles[p], associations, sense_x, sense_y);
%         weights[p] = particles[p].weight;
%     }
% }
% */
% 
% /*
% void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
% 		std::vector<LandmarkObs> observations, Map map_landmarks) {
%     float s_x = std_landmark[0];
%     float s_y = std_landmark[1];
% 
%     for (int i=0; i < num_particles; i++) {
%         Particle p = particles[i];
% 
%         std::vector<LandmarkObs> predicted_landmark_positions;
%         for (int j=0; j < map_landmarks.landmark_list.size(); j++) {
%             LandmarkObs p_o;
%             float x_map = map_landmarks.landmark_list[j].x_f;
%             float y_map = map_landmarks.landmark_list[j].y_f;
%             p_o.id = map_landmarks.landmark_list[j].id_i;
% 
%             // Translate then rotate to get map landmark in map space
%             // to be in vehicle coordinate space from perspective of our particle
%             p_o.x = (x_map - p.x) * cos(-p.theta) - (y_map - p.y) * sin(-p.theta);
%             p_o.y = (x_map - p.x) * sin(-p.theta) + (y_map - p.y) * cos(-p.theta);
%             // predicted_landmark_positions in particle vehicle coordinate space
%             predicted_landmark_positions.push_back(p_o);
%         }
%         // Assigns id values to LandmarkObs in observations variable, passed by reference
%         ParticleFilter::dataAssociation(predicted_landmark_positions, observations);
% 
%         double w = 1;
%         for (int j=0; j < observations.size(); j++) {
%             LandmarkObs a = observations[j];
%             LandmarkObs b;
%             for (int k=0; k < predicted_landmark_positions.size(); k++) {
%                 if (predicted_landmark_positions[k].id == a.id) {
%                     b = predicted_landmark_positions[k];
%                     break;
%                 }
%             }
%             double c = (1 / (2 * M_PI * s_x * s_y));
%             double x_part = pow(a.x - b.x, 2.0) / (2 * s_x * s_x);
%             double y_part = pow(a.y - b.y, 2.0) / (2 * s_y * s_y);
%             w *= c * exp(-(x_part + y_part));
%         }
% 
%         p.weight = w;
%         particles[i] = p;
%     }
% 
%     float sum = 0;
%     for (int i=0; i < num_particles; i++) {
%         sum += particles[i].weight;
%         weights.push_back(particles[i].weight);
%     }
%     for (int i=0; i < num_particles; i++) {
%         particles[i].weight /= sum;
%     }
% 
% }
% */

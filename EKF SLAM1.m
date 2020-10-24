deltaT = .02; 
alphas = [.2 .03 .09 .08 0 0]; % robot-dependent motion noise parameters
                               % see equation 3
 
% robot-dependent sensor noise parameters
 
% see equation 9
 
sigma_range = 2;
sigma_bearing = 3;
sigma_id = 1;
 
Q_t = [sigma_range^2 0 0;
       0 sigma_bearing^2 0;
       0 0 sigma_id^2];
 
measurement_prob = 0;
n_robots = 1;
robot_num = 1;
 
% load and resample raw data from UTIAS data set
 
% The robot's position groundtruth, odometry, and measurements 
 
% are stored in Robots
 
[Barcodes, Landmark_Groundtruth, Robots] = loadMRCLAMdataSet(n_robots);
[Robots, timesteps] = sampleMRCLAMdataSet(Robots, deltaT);
 
% add pose estimate matrix to Robots
 
% data will be added to this as the program runs
 
Robots{robot_num}.Est = zeros(size(Robots{robot_num}.G,1), 4);
 
% initialize time, and pose estimate
 
start = 600; % start index is set to 600 because earlier data was 
             % found to cause problems
t = Robots{robot_num}.G(start, 1); % set start time
 
% set starting pose mean to pose groundtruth at start time
 
poseMean = [Robots{robot_num}.G(start,2);
            Robots{robot_num}.G(start,3);
            Robots{robot_num}.G(start,4)];
poseCov = [0.01 0.01 0.01;
           0.01 0.01 0.01;
           0.01 0.01 0.01];
       
% tracks which measurement is next received
 
measurementIndex = 1;
 
% set up map between barcodes and landmark IDs
 
codeDict = containers.Map(Barcodes(:,2),Barcodes(:,1));
 
% advance measurement index until the next measurement time 
 
% is greater than the starting time
 
while (Robots{robot_num}.M(measurementIndex, 1) < t - .05)
        measurementIndex = measurementIndex + 1;
end
 
% loop through all odometry and measurement samples
 
% updating the robot's pose estimate with each step
 
% reference table 7.2 in Probabilistic Robotics
 
for i = start:size(Robots{robot_num}.G, 1)
    theta = poseMean(3, 1);
    % update time
    t = Robots{robot_num}.G(i, 1);
    % update movement vector per equation 1
    u_t = [Robots{robot_num}.O(i, 2); Robots{robot_num}.O(i, 3)];
 
    rot = deltaT * u_t(2);
    halfRot = rot / 2;
    trans = u_t(1) * deltaT;
    
    % calculate the movement Jacobian per equation 2
    G_t = [1 0 trans * -sin(theta + halfRot);
           0 1 trans * cos(theta + halfRot);
           0 0 1];
    % calculate motion covariance in control space per equation 3
    M_t = [(alphas(1) * abs(u_t(1)) + alphas(2) * abs(u_t(2)))^2 0;
           0 (alphas(3) * abs(u_t(1)) + alphas(4) * abs(u_t(2)))^2];
       
    % calculate Jacobian to transform motion covariance to state space
    % per equation 4
    V_t = [cos(theta + halfRot) -0.5 * sin(theta + halfRot);
           sin(theta + halfRot) 0.5 * cos(theta + halfRot);
           0 1];
    
    % calculate pose update
    poseUpdate = [trans * cos(theta + halfRot);
                  trans * sin(theta + halfRot);
                  rot];
              
    % calculate estimated pose mean per equation 1
    poseMeanBar = poseMean + poseUpdate;
    % calculate estimated pose covariance per equation 5
    poseCovBar = G_t * poseCov * G_t' + V_t * M_t * V_t';
    
    
    % get measurements for the current timestep, if any exist
    [z, measurementIndex] = getObservations(Robots, robot_num, t, measurementIndex, codeDict);
    
    % create two matrices for expected measurement and measurement
    % covariance
    S = zeros(size(z,2),3,3);
    zHat = zeros(3, size(z,2));
    
    % if any measurements are available
    if z(3,1) > 1
        for k = 1:size(z, 2) % loop over every measurement
            j = z(3,k);
 
            % get coordinates of the measured landmark
            m = Landmark_Groundtruth(j, 2:3);
 
            % compute the expected measurement per equations 6 and 7
            xDist = m(1) - poseMeanBar(1);
            yDist = m(2) - poseMeanBar(2);
            q = xDist^2 + yDist^2;
            
            % constrains expected bearing to between 0 and 2*pi
            pred_bear = conBear(atan2(yDist, xDist) - poseMeanBar(3));
            
            zHat(:,k) = [sqrt(q);
                         pred_bear;
                         j];
 
            % calculate Jacobian of the measurement model per equation 8
            H = [(-1 * (xDist / sqrt(q))) (-1 * (yDist / sqrt(q))) 0;
                 (yDist / q) (-1 * (xDist / q)) -1;
                 0 0 0];
             
            % compute S per equation 9
            S(k,:,:) = H * poseCovBar * H' + Q_t;
 
            % compute Kalman gain per equation 10
            K = poseCov * H' * inv(squeeze(S(k,:,:)));
                
            % update pose mean and covariance estimates
            % per equations 11 and 12
            poseMeanBar = poseMeanBar + K * (z(:,k) - zHat(:,k));
            poseCovBar = (eye(3) - (K * H)) * poseCovBar;
        end
    end
    
    % update pose mean and covariance
    % constrains heading to between 0 and 2*pi
    poseMean = poseMeanBar;
    poseMean(3) = conBear(poseMean(3));
    poseCov = poseCovBar;
    
    % add pose mean to estimated position vector
    Robots{robot_num}.Est(i,:) = [t poseMean(1) poseMean(2) poseMean(3)];
    
    % calculate error between mean pose and groundtruth
    % for testing only
    %{
    groundtruth = [Robots{robot_num}.G(i, 2); 
                   Robots{robot_num}.G(i, 3); 
                   Robots{robot_num}.G(i, 4)];
    error = groundtruth - poseMean;
    %}
   
end
 
% display results!
 
animateMRCLAMdataSet(Robots, Landmark_Groundtruth, timesteps, deltaT);
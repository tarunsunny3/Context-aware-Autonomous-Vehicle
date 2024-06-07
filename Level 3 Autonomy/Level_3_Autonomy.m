%% Calling simulink model and security

clc
clear all
close all


decel_lim_lcw = -200;
decel_lim_hcw = -150;
controller_gain_lcw = 15000;
controller_gain_hcw = 90000;
[A, B, C, D, Kess, Kr, Ke, uD] = designControl(secureRand(), controller_gain_hcw);

% Data for LCW and HCW

% Calculate mean and standard deviation for LCW and HCW for HR and RR
average_lwc_hr = 61;
std_lwc_hr = 14;
average_lwc_rr = 17;
std_lwc_rr = 8;

average_hwc_hr = 92;
std_hwc_hr = 23;
average_hwc_rr = 26;
std_hwc_rr = 16;


rng('shuffle');
% Define a range for HR and RR values
range_hr = linspace(min([average_lwc_hr - 3 * std_lwc_hr, average_hwc_hr - 3 * std_hwc_hr]), ...
                    max([average_lwc_hr + 3 * std_lwc_hr, average_hwc_hr + 3 * std_hwc_hr]), 1000);
range_rr = linspace(min([average_lwc_rr - 3 * std_lwc_rr, average_hwc_rr - 3 * std_hwc_rr]), ...
                    max([average_lwc_rr + 3 * std_lwc_rr, average_hwc_rr + 3 * std_hwc_rr]), 1000);

% Create Gaussian models for LCW HR and RR
HR_LCW_pdf = normpdf(range_hr, average_lwc_hr, std_lwc_hr);
RR_LCW_pdf = normpdf(range_rr, average_lwc_rr, std_lwc_rr);

% Create Gaussian models for HCW HR and RR
HR_HCW_pdf = normpdf(range_hr, average_hwc_hr, std_hwc_hr);
RR_HCW_pdf = normpdf(range_rr, average_hwc_rr, std_hwc_rr);


% Number of steps in each sequence
num_steps = 100;

% Number of sequences to generate
num_sequences = 1;


speed_min = 20;
speed_max = 60;

% Gaussian model parameters
speed_mean = (speed_max + speed_min) / 2; % Mean speed
speed_stddev = (speed_max - speed_min) / 6; % Standard deviation of speed
% 

decel_limits = [decel_lim_lcw, decel_lim_hcw];
controller_gains = [controller_gain_lcw, controller_gain_hcw];
reaction_times = [0, 0];

roads_data = cell(num_sequences, 1);
step_reaction_factor = 0.01;
reaction_factor_min = 0.00;
reaction_factor_max = 0.05;

reaction_factor = reaction_factor_min;

collision_switch_map = containers.Map('KeyType', 'double', 'ValueType', 'any');
seq = 0;

gain_min = 90000;
gain_max = 450000;

gain_step = 10000;
gain = gain_min;


speeds = sampleSpeeds(speed_min, speed_max, num_steps);

while gain <= gain_max
    seq = seq + 1;
    fprintf("Sequence %d is running..., Gain is %d\n", seq, gain);
    road_conditions_sequence = generateRoadConditions(num_steps);
    
    collision_switch_map(gain) = struct('collisions', 0,'switches', 0, 'saves', 0);
    for step=1:num_steps
        % fprintf("Step = %d,%d\n", seq, step);
        current_road_condition = road_conditions_sequence{step};
        index = 1;
        if strcmp(current_road_condition, 'HCW')
            index = 2;
        end
        current_speed = speeds(step);
        result = Advisory_Control(current_speed, current_road_condition, reaction_factor);
   
        existing_data = collision_switch_map(gain);
        if(strcmp(result, 'Safe'))
            existing_data.saves = existing_data.saves + 1;
        elseif(strcmp(result, 'Collision'))
            existing_data.collisions =  existing_data.collisions + 1;
        else
            rng('shuffle');
            sampled_HR_LCW = randsample(range_hr, num_steps, true, HR_LCW_pdf);
            sampled_RR_LCW = randsample(range_rr, num_steps, true, RR_LCW_pdf);
            sampled_HR_HCW = randsample(range_hr, num_steps, true, HR_HCW_pdf);
            sampled_RR_HCW = randsample(range_rr, num_steps, true, RR_HCW_pdf);
        
            sampled_HR_LCW(sampled_HR_LCW < 0) = average_lwc_hr;
            sampled_RR_LCW(sampled_RR_LCW < 0) = average_lwc_rr;
            sampled_HR_HCW(sampled_HR_HCW < 0) = average_hwc_hr;
            sampled_RR_HCW(sampled_RR_HCW < 0) = average_hwc_rr;
            % Calculate mean HR and RR values for LCW and HCW
            mean_HR_LCW = mean(sampled_HR_LCW);
            mean_RR_LCW = mean(sampled_RR_LCW);
            mean_HR_HCW = mean(sampled_HR_HCW);
            mean_RR_HCW = mean(sampled_RR_HCW);
        
            reaction_times(1) = 0.01 * (mean_HR_LCW/mean_RR_LCW);
            reaction_times(2) = 0.01 * (mean_HR_HCW/mean_RR_HCW);

            % shouldSwitch = shouldSwitchToHuman(collisionTime, decel_limits{index}, reaction_times{index});
            canstop = canHumanStop(current_speed, decel_limits(index), reaction_times(index));
            if canstop
                existing_data.switches = existing_data.switches  + 1;
            else
                existing_data.collisions = existing_data.collisions + 1;
            end
        end
        collision_switch_map(gain) = existing_data;
    end

    gain = gain + gain_step;

end
% Retrieve all keys (numbers) from the map
keys_in_map = keys(collision_switch_map);

% Iterate through the keys and display the data
for i = 1:length(keys_in_map)
    current_key = keys_in_map{i};
    current_data = collision_switch_map(current_key);

    fprintf('Gain: %d\n', current_key);
    fprintf('Collisions: %d\n', current_data.collisions);
    fprintf('Switches: %d\n', current_data.switches);
    fprintf('Saves: %d\n\n', current_data.saves);
end



function collisionTime = detectCollision(initialSpeed, controllerGain, decelerationLimit)
        % Design the control system
        [A, B, C, D, Kess, Kr, Ke, uD] = designControl(secureRand(), controllerGain);

        % Set model parameters
        open_system('LaneMaintainSystem.slx', 'loadonly');
        set_param('LaneMaintainSystem/VehicleKinematics/Saturation', 'LowerLimit', num2str(decelerationLimit));
        set_param('LaneMaintainSystem/VehicleKinematics/vx', 'InitialCondition', num2str(initialSpeed));

        % Simulate the model
        simModel = sim('LaneMaintainSystem.slx');
        simOut = simModel.get('sx1');

        % Extract the time and data vectors
        time = simOut.time;
        distance = simOut.data;
        colliding_index = 0;
        for index = 1:length(distance)
            if distance(index) >= 0
                colliding_index = index;
                break
            end
        end
        collisionTime = 0;
        
        if(colliding_index ~= 0)
            collidingTime1 = time(colliding_index);
            collisionTime = collidingTime1;
        end
    
        close all

        close_system('LaneMaintainSystem.slx', 0)

end

function result = canHumanStop(speed, decel_limit, reaction_time)
        result = false;
        % Set the parameters for the human model
        open_system("HumanActionModel.slx", 'loadonly');
        % open_system("HumanActionModel.slx");
        set_param('HumanActionModel/Human', 'Time', num2str(reaction_time));
        set_param('HumanActionModel/Human', 'After', num2str(decel_limit * 1.1));
        set_param('HumanActionModel/VehicleKinematics/vx', 'InitialCondition', num2str(speed))
        
        % Simulate the human model
        humanModel = sim("HumanActionModel.slx");
        human_distance = humanModel.get('sx1').data;
        
        if human_distance(end) < 0
            result = true;
        end
        
        % Close the Simulink model
        close_system("HumanActionModel", 0);
  
end


function shouldSwitch = shouldSwitchToHuman(speed, decel_limit, reaction_time, collision_time)

        % Set the parameters for the human model
        open_system("HumanActionModel.slx", 'loadonly');
        set_param('HumanActionModel/Human', 'Time', num2str(reaction_time));
        set_param('HumanActionModel/Human', 'After', num2str(decel_limit * 1.1));
        set_param('HumanActionModel/VehicleKinematics/vx', 'InitialCondition', num2str(speed))
        
        % Simulate the human model
        humanModel = sim("HumanActionModel.slx");
        human_stop = humanModel.get("sx1").time(end);
        shouldSwitch = human_stop < collision_time;
        
        % Close the Simulink model
        close_system("HumanActionModel", 0);
  
end

function road_conditions_sequence = generateRoadConditions(num_steps)
   % Define the transition matrix
    transition_matrix = [0.6, 0.4; 0.85, 0.15];
    
    mc = dtmc(transition_matrix);
    
    % Simulate the Markov Chain with the specified initial state
    road_conditions_indices = simulate(mc, num_steps); % 2 is HCW, 1 is LCW
    
    % Convert the numeric indices to 'LCW' and 'HCW'
    road_conditions_sequence = cell(1, num_steps);
    for i = 1:num_steps
        if road_conditions_indices(i) == 1
            road_conditions_sequence{i} = 'LCW';
        elseif road_conditions_indices(i) == 2
            road_conditions_sequence{i} = 'HCW';
        end
    end
end



function speed_samples = sampleSpeeds(speed_min, speed_max, num_samples)
    % Set the seed of the random number generator to shuffle it
    rng("shuffle");
    
    % Define the mean and standard deviation for the Gaussian distribution
    mean_speed = (speed_max + speed_min) / 2;
    std_dev = (speed_max - speed_min) / 4;  % You can adjust this value
    
    % Sample num_samples speeds from the Gaussian distribution
    speed_samples = mean_speed + std_dev * randn(1, num_samples);
    
    % Clip the sampled speeds to the specified range
    speed_samples(speed_samples < speed_min) = speed_min;
    speed_samples(speed_samples > speed_max) = speed_max;
end


function advisory_result = Advisory_Control(speed, current_road_condition, current_gain)


close all
% Calling the functions with error handling
current_speed = speed;
decel_lim_lcw = -200;
decel_lim_hcw = -150;
controller_gain_lcw = current_gain;
controller_gain_hcw = current_gain;

[~, ~, ~, ~, ~, ~, ~, uD] = designControl(secureRand(), current_gain);


% Data for LCW and HCW
lwc_hrs = [80, 94, 65, 80, 61, 75];
hwc_hrs = [95, 121, 71, 92, 92, 115];

lwc_rrs = [16, 22, 13, 17, 17, 25];
hwc_rrs = [21, 35, 14, 19, 26, 42];

% Calculate mean and standard deviation for LCW and HCW for HR and RR
average_lwc_hr = mean(lwc_hrs);
std_lwc_hr = std(lwc_hrs);
average_lwc_rr = mean(lwc_rrs);
std_lwc_rr = std(lwc_rrs);

average_hwc_hr = mean(hwc_hrs);
std_hwc_hr = std(hwc_hrs);
average_hwc_rr = mean(hwc_rrs);
std_hwc_rr = std(hwc_rrs);


% Define a range for HR and RR values
range_hr = linspace(min([average_lwc_hr - 3 * std_lwc_hr, average_hwc_hr - 3 * std_hwc_hr]), ...
                    max([average_lwc_hr + 3 * std_lwc_hr, average_hwc_hr + 3 * std_hwc_hr]), 1000);
range_rr = linspace(min([average_lwc_rr - 3 * std_lwc_rr, average_hwc_rr - 3 * std_hwc_rr]), ...
                    max([average_lwc_rr + 3 * std_lwc_rr, average_hwc_rr + 3 * std_hwc_rr]), 1000);

% Create Gaussian models for LCW HR and RR
HR_LCW_pdf = normpdf(range_hr, average_lwc_hr, std_lwc_hr);
RR_LCW_pdf = normpdf(range_rr, average_lwc_rr, std_lwc_rr);

% Create Gaussian models for HCW HR and RR
HR_HCW_pdf = normpdf(range_hr, average_hwc_hr, std_hwc_hr);
RR_HCW_pdf = normpdf(range_rr, average_hwc_rr, std_hwc_rr);

num_samples_to_take = 100;


rng('shuffle');
% Sample 100 values from HR and RR Gaussian distributions for LCW and HCW
sampled_HR_LCW = randsample(range_hr, num_samples_to_take, true, HR_LCW_pdf);
sampled_RR_LCW = randsample(range_rr, num_samples_to_take, true, RR_LCW_pdf);
sampled_HR_HCW = randsample(range_hr, num_samples_to_take, true, HR_HCW_pdf);
sampled_RR_HCW = randsample(range_rr, num_samples_to_take, true, RR_HCW_pdf);

% Replace negative values with zeroes
sampled_HR_LCW(sampled_HR_LCW < 0) = average_lwc_hr;
sampled_RR_LCW(sampled_RR_LCW < 0) = average_lwc_rr;
sampled_HR_HCW(sampled_HR_HCW < 0) = average_hwc_hr;
sampled_RR_HCW(sampled_RR_HCW < 0) = average_hwc_rr;

mean_sampled_HR_LCW = mean(sampled_HR_LCW);
mean_sampled_RR_LCW = mean(sampled_RR_LCW);
mean_sampled_HR_HCW = mean(sampled_HR_HCW);
mean_sampled_RR_HCW = mean(sampled_RR_HCW);

reaction_quotient_LCW = (mean_sampled_HR_LCW / mean_sampled_RR_LCW);
reaction_quotient_HCW = (mean_sampled_HR_HCW / mean_sampled_RR_HCW);

reaction_time_factor = 0.00045;
reaction_time_LCW = reaction_time_factor * reaction_quotient_LCW;
reaction_time_HCW = reaction_time_factor * reaction_quotient_HCW;


lcw_data = [controller_gain_lcw, decel_lim_lcw, reaction_time_LCW];
hcw_data = [controller_gain_hcw, decel_lim_hcw, reaction_time_HCW];


% By default it's LCW
current_data = lcw_data;
% But if the current road condition is HCW change it
if(strcmp(current_road_condition, 'HCW'))

     current_data = hcw_data;
end
   

current_controller_gain = current_data(1);
current_decel_lim = current_data(2);
reaction_time = current_data(3);

% Calculate collision time for the current step
collision_time = detectCollision(current_speed, current_controller_gain, current_decel_lim);

% Check if a collision is predicted
if collision_time ~= 0
    % Determine whether to switch to human or not based on reaction time
    
    shouldSwitch = shouldSwitchToHuman(current_speed, current_decel_lim, reaction_time, collision_time);
    if shouldSwitch
        advisory_result = "Switch";
    else
        advisory_result = "Collision";
    end
else
    advisory_result = "Safe";
end

end



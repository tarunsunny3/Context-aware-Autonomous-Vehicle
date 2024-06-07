clc
speed_min = 20;
speed_max = 40;
step_speed = 0.1;
% Iterate over speed values from speed_min to speed_max with step increment


% Initial Gain
gain_hcw = 50000;
gain_lcw = 50000;

gain_step = 10000;
gain_max = 200000;

while  gain_hcw <= gain_max

    current_speed = speed_min;
    safe = 0;
    collisions = 0;
    switches = 0;
    total = 0;

    while current_speed <= speed_max
        % Call the Advisory_Control function for the current speed
        
        result = Advisory_Control_Task1(current_speed, 'HCW', gain_hcw);
        fprintf('Speed: %.1f, Result: %s, Gain = %d\n', current_speed, result, gain_hcw);
        % Display the result for the current speed
        
        if(strcmp(result, "Safe"))
            safe = safe + 1;
        elseif(strcmp(result, "Collision"))
            collisions = collisions + 1 ;
        else
            switches = switches + 1;
        end
        % Increment the current_speed by the step value
        current_speed = current_speed + step_speed;
        total = total +  1;
    end
    fprintf('Gain HCW = %d, Saves = %d, Switches = %d, Collisions = %d, Total = %d\n',gain_hcw,safe,switches, collisions, total );
    gain_hcw = gain_hcw + gain_step;
end




while  gain_lcw <= gain_max

    current_speed = speed_min;
    safe = 0;
    collisions = 0;
    switches = 0;
    total = 0;

    while current_speed <= speed_max
        % Call the Advisory_Control function for the current speed
        result = Advisory_Control_Task1(current_speed, 'LCW', gain_lcw);
        % Display the result for the current speed
        % fprintf('Speed: %.1f, Result: %s, Gain = %d\n', current_speed, result, gain_hcw);

        if(strcmp(result, "Safe"))
            safe = safe + 1;
        elseif(strcmp(result, "Collision"))
            collisions = collisions + 1 ;
        else
            switches = switches + 1;
        end
        % Increment the current_speed by the step value
        current_speed = current_speed + step_speed;
        total = total +  1;
    end
    fprintf('Gain LCW = %d, Saves = %d, Switches = %d, Collisions = %d, Total = %d\n',gain_lcw,safe,switches, collisions, total );
    gain_lcw = gain_lcw + gain_step;
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


function advisory_result = Advisory_Control_Task1(speed, current_road_condition, controller_gain)
close all
% Calling the functions with error handling
current_speed = speed;
decel_lim_lcw = -200;
decel_lim_hcw = -150;

[~, ~, ~, ~, ~, ~, ~, uD] = designControl(secureRand(), controller_gain);

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

num_samples_to_take = 100;


rng('shuffle');
% Sample 100 values from HR and RR Gaussian distributions for LCW and HCW
sampled_HR_LCW = randsample(range_hr, num_samples_to_take, true, HR_LCW_pdf);
sampled_RR_LCW = randsample(range_rr, num_samples_to_take, true, RR_LCW_pdf);
sampled_HR_HCW = randsample(range_hr, num_samples_to_take, true, HR_HCW_pdf);
sampled_RR_HCW = randsample(range_rr, num_samples_to_take, true, RR_HCW_pdf);

% Replace negative values with zeroes
sampled_HR_LCW(sampled_HR_LCW < 0) = 0;
sampled_RR_LCW(sampled_RR_LCW < 0) = 0;
sampled_HR_HCW(sampled_HR_HCW < 0) = 0;
sampled_RR_HCW(sampled_RR_HCW < 0) = 0;

mean_sampled_HR_LCW = mean(sampled_HR_LCW);
mean_sampled_RR_LCW = mean(sampled_RR_LCW);
mean_sampled_HR_HCW = mean(sampled_HR_HCW);
mean_sampled_RR_HCW = mean(sampled_RR_HCW);

reaction_quotient_LCW = (mean_sampled_HR_LCW / mean_sampled_RR_LCW);
reaction_quotient_HCW = (mean_sampled_HR_HCW / mean_sampled_RR_HCW);

reaction_time_factor = 0.01;

reaction_time_LCW = reaction_time_factor * reaction_quotient_LCW;
reaction_time_HCW = reaction_time_factor * reaction_quotient_HCW;


lcw_data = [decel_lim_lcw, reaction_time_LCW];
hcw_data = [decel_lim_hcw, reaction_time_HCW];


% By default it's LCW
current_data = lcw_data;
% But if the current road condition is HCW change it
if(strcmp(current_road_condition, 'HCW'))

     current_data = hcw_data;
end
   

current_controller_gain = controller_gain;
current_decel_lim = current_data(1);
reaction_time = current_data(2);

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



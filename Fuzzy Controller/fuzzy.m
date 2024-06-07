function [isSwitch, timeToSwitchToHuman] = TarunChinthakindi(va, sAB)
   
    load('MemberDecel200.mat', "-mat", "decelLim");
    isSwitch = false;
    timeToSwitchToHuman = -1;
    timeStep = 0.01;


    numSamples = length(va);

    currentTime = 0;
    
    rng('shuffle');

    %% Instead of using the hardcoded values which we get from TASK 1, which
    %% is just a speed difference of a random road path, I am using a speed range of my choice
    %% both for input membership function's range and as well as sampling it to use it as input
    %% for the 3rd membership function
    

    % Sample the speeds of the same size as the input vectors

    min_range = -30;
    max_range = 30;
    
    meanValue = (min_range + max_range) / 2;
    range_width = max_range - min_range;
    stdDev = range_width / 4;
    speedValues = meanValue + stdDev * randn(numSamples, 1);
    speedValues(speedValues < min_range) = min_range;
    speedValues(speedValues > max_range) = max_range;


    disp("Calculating.......")



    for sampleIndex = 2:numSamples
        vAT2 = va(sampleIndex);
        vAT1 = va(sampleIndex - 1);
        deceleration = abs((vAT2 - vAT1)) / timeStep;
        
        distance = sAB(sampleIndex);
        roadCondition = speedValues(sampleIndex);

        fis = readfis('TarunChinthakindi.fis');

        inputValues = [deceleration, distance, roadCondition];
        decelerationB = evalfis(fis, inputValues);
        

        if decelerationB > 0.75 * abs(decelLim)
            % Switch To human
            isSwitch = true;
            timeToSwitchToHuman = currentTime;
            break

        end
        
        currentTime = currentTime + timeStep;
        
    end

    if(timeToSwitchToHuman == -1)
        timeToSwitchToHuman = "N/A";
    end

    if isSwitch
    fprintf("Did it switch?\n\tYes, It switched\n");
    fprintf("Time to switch to human is %.2f\nTotal steps = %d and Steps it took to switch = %.2f\n" + ...
        "Perecentage of steps it took to Switch = %.2f%%\n", timeToSwitchToHuman, numSamples, timeToSwitchToHuman/0.01,timeToSwitchToHuman*10000/numSamples);

    else
        fprintf("Did it switch?\n\tNo, It didn't switch\nTime took to switch to Human is %s\n", timeToSwitchToHuman);
    end


end
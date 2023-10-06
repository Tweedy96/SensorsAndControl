% Initialization
clear all
close all
set(0,'DefaultFigureWindowStyle','docked')

rosshutdown
rosinit
warning('off')

node = ros.Node('/red_object_detection');

% Create ROS subscribers and publishers
rgbSub = rossubscriber('turtlebot1/camera/rgb/image_raw');
cmdPub = rospublisher('/turtlebot1/cmd_vel', 'geometry_msgs/Twist');
msg = rosmessage(cmdPub);

% Main loop to detect red objects
while true
    % Get the RGB image from the ROS topic
    rgbMsg = receive(rgbSub);
    rgbImg = readImage(rgbMsg);
    
    % Extract the RGB and Alpha channels from your RGBA image
    redChannel = rgbImg(:,:,1);
    greenChannel = rgbImg(:,:,2);
    blueChannel = rgbImg(:,:,3);
    
    % Define thresholds
    redThreshold = 0.99;  % Close to 1 for strong red
    otherThreshold = 0.01;
 
    
    % Threshold using both color and alpha information
    binaryRed = (redChannel > redThreshold) & ...
                (greenChannel < otherThreshold) & ...  % Assuming other channels are close to 0 for red
                (blueChannel <otherThreshold);

% Continue with labeling and regionprops as before...

    
    % Label the red regions
   labeledRed = bwlabel(binaryRed);
stats = regionprops(labeledRed, 'Area', 'Centroid');

if ~isempty(stats)
    [~, idx] = max([stats.Area]);
    targetCentroid = stats(idx).Centroid;

    % Calculate relative angle (assuming the center of the image is directly in front)
    imgCenterX = size(rgbImg,2) / 2;
    angleError = (targetCentroid(1) - imgCenterX) / imgCenterX;

    % Decide on a rotation speed based on the error. 
    rotationSpeed = 0.1; % This is a proportional controller, you can adjust the factor.

    % Estimate distance based on the size of the detected red object
    estimatedDistance = 1 / sqrt(stats(idx).Area);  % We use the square root to linearize the relationship between area and distance.

    % Set the robot's motion commands
    if estimatedDistance < 0.005
        disp('very close');
        msg.Linear.X = 0;
        msg.Angular.Z = 0;
    elseif abs(angleError) < 0.1  % 0.1 is a threshold for when the object is "centered" and can be adjusted
        disp('moving towards object');
        msg.Linear.X = 0.1; % Forward speed
        msg.Angular.Z = 0;  % No rotation
    else
        % Rotate in place to center the object in view
        disp('large angle error');
        msg.Linear.X = 0;
        msg.Angular.Z = rotationSpeed;  % Rotate based on error
    end
    send(cmdPub, msg);

    % Display the result
    imshow(rgbImg);
    hold on;
    plot(targetCentroid(1), targetCentroid(2), 'ro');
    hold off;
else
    % Spin in place if no red object is detected
    msg.Linear.X = 0;
    msg.Angular.Z = 0.5;  % Adjust rotation speed as necessary
    send(cmdPub, msg);
end

pause(0.1); % Adjust this based on desired update rate
end

rosshutdown

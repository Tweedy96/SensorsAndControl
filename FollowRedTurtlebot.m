classdef FollowRedTurtlebot
    % This class enables a turtlebot to follow a red object
    properties
        rgbSub
        cmdPub
        msg

        depthSub
        odomSub

    end

    methods
        function self = FollowRedTurtlebot()
            clc;
            close all;
            set(0,'DefaultFigureWindowStyle','docked');
            
            rosshutdown;
            rosinit;
            warning('off');
            
            node = ros.Node('/red_object_detection');
           
            % Create ROS subscribers and publishers
            self.rgbSub = rossubscriber('turtlebot1/camera/rgb/image_raw');
            self.cmdPub = rospublisher('/turtlebot1/cmd_vel', 'geometry_msgs/Twist');
            self.msg = rosmessage(self.cmdPub);
            self.depthSub = rossubscriber('turtlebot1/camera/depth/image_raw');
            self.odomSub = rossubscriber('turtlebot1/odom');
            
            % input('Turtlebot loaded, press enter to start')
            while true
                self.ProcessImage();
                pause(0.1);
            end

        end
%%
        function ProcessImage(self)
            % Get the RGB image from the ROS topic
            rgbMsg = receive(self.rgbSub);
            rgbImg = readImage(rgbMsg);
            
            % Extract the RGB channels from the image
            redChannel = rgbImg(:,:,1);
            greenChannel = rgbImg(:,:,2);
            blueChannel = rgbImg(:,:,3);
            
            % Define thresholds
            redThreshold = 0.99;
            otherThreshold = 0.01;
            
            % Threshold using color information
            binaryRed = (redChannel > redThreshold) & ...
                        (greenChannel < otherThreshold) & ...
                        (blueChannel < otherThreshold);

            imshow(rgbImg);
        
            % Compute the weighted centroid
            [rows, cols] = find(binaryRed);
            if isempty(rows) || isempty(cols)
                self.SearchRobot();
                return; % No red detected
            end

            weights = double(redChannel(binaryRed));
            weightedCentroidX = sum(double(cols) .* weights) / sum(weights);
            weightedCentroidY = sum(double(rows) .* weights) / sum(weights);
            targetCentroid = [weightedCentroidX, weightedCentroidY];
        
        
            % Label the red regions
            stats = regionprops(binaryRed, 'Area', 'Centroid');
            
            % Filter out regions that are below the minimum area threshold
            minRedArea = 5;
            stats = stats([stats.Area] > minRedArea);
        
            % Get depth image
            depthMsg = receive(self.depthSub);
            depthImg = readImage(depthMsg);
            
            


        
            if ~isempty(stats)
                % Calculate relative angle (assuming the center of the image is directly in front)
                imgCenterX = size(rgbImg,2) / 2;
                angleError = (targetCentroid(1) - imgCenterX) / imgCenterX;
            
                % Decide on a rotation speed based on the error. 
                rotationSpeed = -0.1 * sign(angleError); % Gives position or negative value based on direction of error
            
                % Estimate distance based on the size of the detected red object
                estimatedDistance = 1 / sqrt(stats(1).Area);  % Square root to linearise the relationship between area and distance.
                        
                % Get depth value at the target centroid
                depthValue = depthImg(round(targetCentroid(2)), round(targetCentroid(1)));
                disp(['Depth to lead robot: ', num2str(depthValue), ' meters']);
                
                % Get turtlebot1's position in global frame
                odomMsg = receive(self.odomSub);
                tb1Position = [odomMsg.Pose.Pose.Position.X, odomMsg.Pose.Pose.Position.Y];
                
               % Convert angleError to actual angle based on camera's FOV
                rgbCamFov = deg2rad(77); 
                actualAngle = angleError * rgbCamFov / 2;
        
                self.TrackRobot(estimatedDistance, rotationSpeed, angleError, depthValue);
            
                % Display the result
                hold on;
                plot(targetCentroid(1), targetCentroid(2), 'yo', 'MarkerSize', 25);
                hold off;

            end
        end
%%
        function SearchRobot(self)
            disp('searching for robot')
            % Spin in place if no red object is detected
                self.msg.Linear.X = 0;
                self.msg.Angular.Z = 0.2;  % Adjust rotation speed as necessary
                send(self.cmdPub, self.msg);
        end

%% 
        function TrackRobot(self, estimatedDistance, rotationSpeed, angleError, depthValue)
            % Set the robot's motion commands
        
            % If the depth value is available and indicates the robot is too close
            if ~isnan(depthValue) && depthValue < 1
                disp('Close to the lead robot');
                self.msg.Linear.X = 0; % Stop moving forward
                
                if abs(angleError) > deg2rad(10)
                    self.msg.Angular.Z = rotationSpeed * 2;  % Rotate based on error
                else
                self.HoldRobot();
                end
            elseif abs(angleError) < 0.2  
                disp('moving towards object');
                self.msg.Linear.X = 0.1; % Forward speed
                self.msg.Angular.Z = rotationSpeed;  % Rotate based on error
            else
                disp('large angle error');
                self.msg.Linear.X = 0.05; % Move forward at a slower speed when there's a large angle error
                self.msg.Angular.Z = rotationSpeed * 2;  % Rotate based on error
            end
    
            
        
            send(self.cmdPub, self.msg);
            % pause(0.1);
        end
%%
        function HoldRobot(self)
            self.msg.Linear.X = 0;
            self.msg.Angular.Z = 0;
            self.msg.Angular.Z = 0;  
        end


    end
   
end

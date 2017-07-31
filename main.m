function main()
    % youbot Illustrates the V-REP Matlab bindings.

    % (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017, Mathieu Baijot 2017.
    % Distributed under the GNU General Public License.
    % (See http://www.gnu.org/copyleft/gpl.html)
   
    %% Initiate the connection to the simulator. 
    
    disp('Program started');
    % Use the following line if you had to recompile remoteApi
    %vrep = remApi('remoteApi', 'extApi.h');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    
    % If you get an error like: 
    %   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
    % Make sure your code is within a function! You cannot call V-REP from a script. 

    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;x
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % This will only work in "continuous remote API server service". 
    % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Let a few cycles pass to make sure there's a value waiting for us next time
    % we try to get a joint angle or the robot pose with the simx_opmode_buffer
    % option.
    pause(.2);
    
    %% test
% Get initial position and put Youbot on the map
[res, youbotInitPos] = vrep.simxGetObjectPosition(id, h.ref, h.ref, vrep.simx_opmode_buffer)

vrchk(vrep, res, true);
[res, youbotInitEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer)
vrchk(vrep, res, true);

    %% Youbot constants
    timestep = .05;

    % Minimum and maximum angles for all joints. Only useful to implement custom IK. 
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Definition of the starting pose of the arm.
    startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

    %% Preset values for the demo. 
    disp('Starting robot');
    
    % Define the preset pickup pose. 
    pickupJoints = [90 * pi / 180, 19.6 * pi / 180, 113 * pi / 180, - 41 * pi / 180, 0];

    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;
    prevOri = 0; 
    prevLoc = 0;

    % Set the arm to its starting configuration. 
    res = vrep.simxPauseCommunication(id, true); % Send order to the simulator through vrep object. 
    vrchk(vrep, res); % Check the return value and exit in case of error. 
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
    res = vrep.simxPauseCommunication(id, false); 
    vrchk(vrep, res);

    % Initialise the plot. 
    plotData = true;
    if plotData
        subplot(211);
        drawnow;
        
        % Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
        % see, by selecting the points within this mesh that are within the visibility range. 
        [X, Y] = meshgrid(-5:.25:5, -5.5:.25:2.5);
        X = reshape(X, 1, []);
        Y = reshape(Y, 1, []);
    end

    % Make sure everything is settled before we start. 
    pause(2);

    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Initialise the state machine. 
    fsm = 'rotate';

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        %% Plot something if required. 
        if plotData
            % Read data from the Hokuyo sensor.
            [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
            disp('go')
            pts(:,1:5)
            pts(:,680:end)
            
            % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo. 
            in = inpolygon(X, Y, [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)],...
                          [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);

            % Plot those points. Green dots: the visible area for the Hokuyo. Red starts: the obstacles. Red lines: the
            % visibility range from the Hokuyo sensor. 
            % The youBot is indicated with two dots: the blue one corresponds to the rear, the red one to the Hokuyo
            % sensor position. 
            subplot(211)
            plot(X(in), Y(in), '.g',...
                 pts(1, contacts), pts(2, contacts), '*r',...
                 [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)], [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)], 'r',...
                 0, 0, 'ob',...
                 h.hokuyo1Pos(1), h.hokuyo1Pos(2), 'or',...
                 h.hokuyo2Pos(1), h.hokuyo2Pos(2), 'or');
            axis([-5.5, 5.5, -5.5, 2.5]);
            axis equal;
            drawnow;
        end
        angl = -pi/2;

        %% Apply the state machine. 
        if strcmp(fsm, 'rotate')
            %% First, rotate the robot to go to one table.             % The rotation velocity depends on the difference between the current angle and the target. 
            rotVel = angdiff(angl, youbotEuler(3));
            
            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOri, youbotEuler(3))) < .01 / 180 * pi)
                rotVel = 0;
                fsm = 'drive';
            end
            
            prevOri = youbotEuler(3);
        elseif strcmp(fsm, 'drive')
            %% Then, make it move straight ahead until it reaches the table. 
            % The further the robot, the faster it drives. (Only check for the first dimension.)
            %forwBackVel = -(youbotPos(1) + 3.167);
            forwBackVel=-10;
            % If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to 
            % a specific location before moving on to the next state.
            if (youbotPos(1) + 3.167 < .001) && (abs(youbotPos(1) - prevLoc) < .001)
                forwBackVel = 0;
                
                % Change the orientation of the camera
                vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0 0 pi/4], vrep.simx_opmode_oneshot);
                % Move the arm to the preset pose.
                for i = 1:5
                    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i),...
                                                          vrep.simx_opmode_oneshot);
                    vrchk(vrep, res, true);
                end

                fsm = 'snapshot';
            end
            prevLoc = youbotPos(1);
        elseif strcmp(fsm, 'snapshot')
            %% Read data from the range camera
            % Reading a 3D image costs a lot to VREP (it has to simulate the image). It
            % also requires a lot of bandwidth, and processing a 3D point cloud (for
            % instance, to find one of the boxes or cylinders that the robot has to
            % grasp) will take a long time in MATLAB. In general, you will only want to
            % capture a 3D image at specific times, for instance when you believe you're
            % facing one of the tables.

            % Reduce the view angle to better see the objects. Indeed, the number of rays the Hokuyo sends is limited;
            % if this number is used on a smaller angle, then the resolution is better. 
            res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

            % Ask the sensor to turn itself on, take A SINGLE 3D IMAGE, and turn itself off again. 
            res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

            % Then use the depth sensor. 
            fprintf('Capturing point cloud...\n');
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
            % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
            % the output data. To get a correct plot, you should invert the y and z dimensions. 
    
            % Here, we only keep points within 1 meter, to focus on the table. 
            pts = pts(1:3, pts(4, :) < 1);

            if plotData
                subplot(223)
                plot3(pts(1, :), pts(3, :), pts(2, :), '*');
                axis equal;
                view([-169 -46]);
            end

            % Save the pointcloud to pc.xyz. (This file can be displayed with http://www.meshlab.net/.)
            fileID = fopen('pc.xyz','w');
            fprintf(fileID,'%f %f %f\n',pts);
            fclose(fileID);
            fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));

            %% Read data from the RGB camera
            % This is very similar to reading from the 3D camera. The comments in the 3D camera section directly apply 
            % to this section.

            res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fprintf('Capturing image...\n');
            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));

            if plotData
                subplot(224)
                imshow(image);
                drawnow;
            end

            % Next state. 
            fsm = 'extend';
        elseif strcmp(fsm, 'extend')
            %% Move the arm to face the object.
            % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % If the arm has reached the wanted position, move on to the next state. 
            if norm(tpos - [0.3259 -0.0010 0.2951]) < .002
                % Set the inverse kinematics (IK) mode to position AND orientation. 
                res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res, true);
                fsm = 'reachout';
            end
        elseif strcmp(fsm, 'reachout')
            %% Move the gripper tip along a line so that it faces the object with the right angle.
            % Get the arm tip position. (It is driven only by this position, except if IK is disabled.)
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);

            % If the tip is at the right position, go on to the next state. 
            if tpos(1) > .39
                fsm = 'grasp';
            end

            % Move the tip to the next position (it moves along a line). 
            tpos(1) = tpos(1) + .01;
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
        elseif strcmp(fsm, 'grasp')
            %% Grasp the object by closing the gripper on it.
            % Close the gripper. Please pay attention that it is not possible to determine the force to apply and 
            % object will sometimes slips from the gripper!
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            pause(2);
            
            % Disable IK; this is used at the next state to move the joints manually. 
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fsm = 'backoff';
        elseif strcmp(fsm, 'backoff')
            %% Go back to rest position.
            % Set each joint to their original angle. 
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            
            % Get the gripper position and check whether it is at destination.
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            if norm(tpos - homeGripperPosition) < .02
                % Open the gripper. 
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep, res);
            end
            
            if norm(tpos - homeGripperPosition) < .002
                fsm = 'finished';
            end
        elseif strcmp(fsm, 'finished')
            %% Demo done: exit the function. 
            pause(3);
            break;
        else
            error('Unknown state %s.', fsm);
        end

        % Update wheel velocities using the global values (whatever the state is). 
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

        % Make sure that we do not go faster that the simulator (each iteration must take 50 ms). 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end

end % main function
   

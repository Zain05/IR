function main2()
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

    % Make sure everything is settled before we start. 
    pause(2);

    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Initialise the state machine. 
    fsm = 'drive';
    stop=0;
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
      if strcmp(fsm, 'drive')

        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
       
        if abs(youbotPos(2))<0.002
            forwBackVel =0;
            stop=1;
        elseif youbotPos(2)>0
            forwBackVel = -5;
        else
            forwBackVel =5;
        end
        
        if stop==1
            fsm='finished';
        end
       youbotPos
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

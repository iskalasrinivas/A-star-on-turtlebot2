vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997,true,true,5000,5);
vrep.simxSynchronous(clientID,true);
dt=0.001;
vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,dt,vrep.simx_opmode_blocking);
[returnCode,robot]=vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking);
[returnCode,dummy]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_blocking);
vrep.simxSetObjectPosition(clientID,robot,dummy,[start_node(1)/100,start_node(2)/100,0],vrep.simx_opmode_oneshot);
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
timeStep = 1;
    
    if (clientID ~= -1)
		disp('Main Script Started')
        [ret,rightWheelHandle] = vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_oneshot);
		[ret,leftWheelHandle] = vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_oneshot);
		% This is in radians per second
		vrep.simxSetJointTargetVelocity(clientID,rightWheelHandle,0,vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetVelocity(clientID,leftWheelHandle,0,vrep.simx_opmode_oneshot)
        i=1;
        while vrep.simxGetConnectionId((clientID) ~=-1)
            [ret,leftWheelHandle] = vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_oneshot);
			[ret,rightWheelHandle] = vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_oneshot);
            [ret,time]=vrep.simxGetFloatSignal(clientID,'SimulationTime',vrep.simx_opmode_streaming);
            if(i<size(path_rpm,1))
                i=floor(time/timeStep)+1;
                if(i>=size(path_rpm,1))
                    i=size(path_rpm,1);
                end
                vrep.simxSetJointTargetVelocity(clientID,leftWheelHandle,path_rpm(i,1)*(pi/30),vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID,rightWheelHandle,path_rpm(i,2)*(pi/30),vrep.simx_opmode_blocking);
                vrep.simxSynchronousTrigger(clientID);
                [ret,pos]=vrep.simxGetObjectPosition(clientID,robot,dummy,vrep.simx_opmode_streaming);
            else
                vrep.simxSetJointTargetVelocity(clientID,leftWheelHandle,0,vrep.simx_opmode_oneshot);
                vrep.simxSetJointTargetVelocity(clientID,rightWheelHandle,0,vrep.simx_opmode_oneshot);
                pause(10);
                vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
                vrep.simxFinish(clientID)
                
            end
        end
    else
		disp("Failed to connect to remote API Server")
		vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
		vrep.simxFinish(clientID)
    end
    
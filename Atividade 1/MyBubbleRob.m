%% Clearing workspace
clear all
close all
clc
%% Loading V-REP remote interface - client side
vrep=remApi('remoteApi');
%% Closing any previously opened connections
vrep.simxFinish(-1);
%% Connecting to remote V-REP API server
retCod=0;
connAdd='127.0.0.1';
connPor=19997;
wuc=true;
dnrod=true;
toi=5000;
ctci=5;
while(retCod == 0)
    [clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
    if(clientID > -1),
        fprintf('Starting\n');
        retCod=1;
    else
        fprintf ('Waiting\n');
    end
end
%% Getting robot handles
[~,RobBody]=vrep.simxGetObjectHandle(clientID,'BubbleRobBody',vrep.simx_opmode_blocking);
[~,leftMotor]=vrep.simxGetObjectHandle(clientID,'BubbleRobLeftMotor',vrep.simx_opmode_blocking);
[~,rightMotor]=vrep.simxGetObjectHandle(clientID,'BubbleRobRightMotor',vrep.simx_opmode_blocking);
[~,noseSensor]=vrep.simxGetObjectHandle(clientID,'BubbleRobSensingNose',vrep.simx_opmode_blocking);

[~,pBody] = vrep.simxGetObjectHandle(clientID,'Pionee',vrep.simx_opmode_blocking);
[~,pLM] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
[~,pRM] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
%% Defining V-REP client side controller parameters
np=500;
hd=50e-3;
tf=np*hd;
tc=0;
td=0;
id=1;
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);

xp2=zeros(np,1);
yp2=zeros(np,1);
fp2=zeros(np,1);
%% Starting V-REP simulation
[retCodeS]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
e=1;
while (e~=0),
    [e,result,~,~,~]=vrep.simxReadProximitySensor(clientID,noseSensor,vrep.simx_opmode_streaming);
end
%% Main control loop - V-REP client side
driveBackTime=0;
t0=cputime;
while tc < tf,
    tc=cputime-t0;
    %% Current sampling instant
    if tc > td,
        t(id)=tc;
        %% Measuring
        [~,RobBodyPos]=vrep.simxGetObjectPosition(clientID,RobBody,-1,vrep.simx_opmode_streaming);
        [~,RobBodyOri]=vrep.simxGetObjectOrientation(clientID,RobBody,-1,vrep.simx_opmode_streaming);
        % Robot pose
        xa=RobBodyPos(1,1);
        ya=RobBodyPos(1,2);
        fa=RobBodyOri(1,3);
        %% Saving
        xp(id)=xa;
        yp(id)=ya;
        fp(id)=fa;
        
        [~,RobBodyPos]=vrep.simxGetObjectPosition(clientID,pBody,-1,vrep.simx_opmode_streaming);
        [~,RobBodyOri]=vrep.simxGetObjectOrientation(clientID,pBody,-1,vrep.simx_opmode_streaming);
        xa=RobBodyPos(1,1);
        ya=RobBodyPos(1,2);
        fa=RobBodyOri(1,3);
        xp2(id)=xa;
        yp2(id)=ya;
        fp2(id)=fa;
        %%
        if driveBackTime>0,
            driveBackTime=driveBackTime-td;
        end
        [~,result,~,~,~]=vrep.simxReadProximitySensor(clientID,noseSensor,vrep.simx_opmode_buffer);
        if result>0,
            driveBackTime=60;
            vrep.simxSetJointTargetVelocity(clientID,leftMotor,-pi*0.5,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rightMotor,-pi*0.25,vrep.simx_opmode_oneshot);
        end
        if driveBackTime<=0,
            vrep.simxSetJointTargetVelocity(clientID,leftMotor,pi,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rightMotor,pi,vrep.simx_opmode_oneshot);
        end
            vrep.simxSetJointTargetVelocity(clientID,pLM,1.05*pi,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,pRM,pi,vrep.simx_opmode_oneshot);
        %% Next sampling instant
        td=td+hd;
        id=id+1;
    end
end
%% Stoping V-REP simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
vrep.simxFinish(clientID);
vrep.delete();
%% Plotting results
figure(1)
plot(t,xp,t,yp,t,fp),grid
legend('x_p(t)','y_p(t)','\phi_p(t)')
xlabel('t [s]')
figure(2)
plot(t,xp2,t,yp2,t,fp2),grid
legend('x_p(t)','y_p(t)','\phi_p(t)')
xlabel('t [s]')
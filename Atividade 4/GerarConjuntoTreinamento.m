% for count = 1:3000
% 
% clc, close all;
% 
% %% Loading V-REP remote interface - client side
% vrep=remApi('remoteApi');
% %% Closing any previously opened connections
% vrep.simxFinish(-1);
% %% Connecting to remote V-REP API server
% retCod=0;
% connAdd='127.0.0.1';
% connPor=19997;
% wuc=true;
% dnrod=true;
% toi=5000;
% ctci=5;
% %[clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
% while(retCod == 0)
%     [clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
%     if(clientID > -1)
%         fprintf('Starting\n');
%         retCod=1;
%     else
%         fprintf ('Waiting\n');
%     end
% end
% %% Getting robot handles
% [~,p3dxBody] = vrep.simxGetObjectHandle(clientID,'p3dx',vrep.simx_opmode_blocking);
% [~,leftMotor] = vrep.simxGetObjectHandle(clientID,'p3dx_leftMotor',vrep.simx_opmode_blocking);
% [~,rightMotor] = vrep.simxGetObjectHandle(clientID,'p3dx_rightMotor',vrep.simx_opmode_blocking);
% [~,goal] = vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_blocking);
% 
% 
% %% Starting V-REP simulation
% [retCodeS]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
% e=1;
% % while (e~=0),
% %     [e,result,~,~,~]=vrep.simxReadProximitySensor(clientID,noseSensor,vrep.simx_opmode_streaming);
% % end
% %% Main control loop - V-REP client side
% driveBackTime=0;
% t0=cputime;
% 
% np=25; %Tempo máximo de espera
% hd=100e-3;
% tf=np*hd;
% 
% tc=0;
% td=0;
% id=1;
% t=zeros(np,1);
% xp=zeros(np,1);
% yp=zeros(np,1);
% fp=zeros(np,1);
% xRef = zeros(np,1);
% yRef = zeros(np,1);
% fRef = zeros(np,1);
% intensity1 = zeros(1,7);
% intensity2 = zeros(1,7);
% gam = zeros(np,1);
% tet = zeros(np,1);
% 
% 
% 
% %% Load path
% load path.mat
% 
% %% Measure Dist
% 
% distLim = 0.1;
% dist = 0;
% 
%         [~,intensity1(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS01',vrep.simx_opmode_streaming);
%         [~,intensity1(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS02',vrep.simx_opmode_streaming);
%         [~,intensity1(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS03',vrep.simx_opmode_streaming);
%         [~,intensity1(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS04',vrep.simx_opmode_streaming);
%         [~,intensity1(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS05',vrep.simx_opmode_streaming);
%         [~,intensity1(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS06',vrep.simx_opmode_streaming);
%         [~,intensity1(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS07',vrep.simx_opmode_streaming);
%         
%         [~,intensity2(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS08',vrep.simx_opmode_streaming);
%         [~,intensity2(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS09',vrep.simx_opmode_streaming);
%         [~,intensity2(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS10',vrep.simx_opmode_streaming);
%         [~,intensity2(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS11',vrep.simx_opmode_streaming);
%         [~,intensity2(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS12',vrep.simx_opmode_streaming);
%         [~,intensity2(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS13',vrep.simx_opmode_streaming);
%         [~,intensity2(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS14',vrep.simx_opmode_streaming);
%         
%        load rngVectors.mat; 
%        vrep.simxSetObjectPosition(clientID,p3dxBody,-1,[-1.15+0.28*randX(count), -1.4+0.95*randY(count), 0.1388], vrep.simx_opmode_streaming);
%        vrep.simxSetObjectOrientation(clientID,p3dxBody,-1,[deg2rad(0.0), deg2rad(0.0), deg2rad(50+100*randF(count))], vrep.simx_opmode_streaming);
% 
% while ((tc < tf) || (id < 20))
%     tc=cputime-t0;
%     %% Current sampling instant
%     if tc > td
%         t(id)=tc;
%         %% Measuring
%         [~,p3dxBodyPos]=vrep.simxGetObjectPosition(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
%         [~,p3dxBodyOri]=vrep.simxGetObjectOrientation(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
%         [~,goalDist]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
%         % Intensity
%         [~,intensity1(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS01',vrep.simx_opmode_buffer);
%         [~,intensity1(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS02',vrep.simx_opmode_buffer);
%         [~,intensity1(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS03',vrep.simx_opmode_buffer);
%         [~,intensity1(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS04',vrep.simx_opmode_buffer);
%         [~,intensity1(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS05',vrep.simx_opmode_buffer);
%         [~,intensity1(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS06',vrep.simx_opmode_buffer);
%         [~,intensity1(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS07',vrep.simx_opmode_buffer);
%         
%         [~,intensity2(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS08',vrep.simx_opmode_buffer);
%         [~,intensity2(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS09',vrep.simx_opmode_buffer);
%         [~,intensity2(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS10',vrep.simx_opmode_buffer);
%         [~,intensity2(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS11',vrep.simx_opmode_buffer);
%         [~,intensity2(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS12',vrep.simx_opmode_buffer);
%         [~,intensity2(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS13',vrep.simx_opmode_buffer);
%         [~,intensity2(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS14',vrep.simx_opmode_buffer);
%         
%         % Robot pose
%         xa=p3dxBodyPos(1,1);
%         ya=p3dxBodyPos(1,2);
%         fa=p3dxBodyOri(1,3);
%         % Goal location
%         xg = goalDist(1,1);
%         yg = goalDist(1,2);
% 
%         %% Saving
%         xp(id)=xa;
%         yp(id)=ya;
%         fp(id)=fa;
%         xRef(id) = xg;
%         yRef(id) = yg;
% 
%         %% Velocity Calculus
%         dist = measureDist(xa,ya,xg,yg); % Distance to goal
%         
%         [gam(id), tet(id)] = SensorIdeal(xa, ya, fa, pathX, pathY);
%             vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_oneshot);
%             vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_oneshot);
% 
%         %% Next sampling instant
%         td=td+hd;
%         id=id+1;
%     end
% end
% 
% % t=t(1:id-20);
% % xp=xp(20:id-1);
% % yp=yp(20:id-1);
% % fp=fp(20:id-1);
% % xRef=xRef(20:id-1);
% % yRef=yRef(20:id-1);
% % fRef=fRef(20:id-1);
% % gam = gam(20:id-1);
% % tet = tet(20:id-1);
% 
% % figure;
% % subplot(211);
% % plot(t,gam);
% % subplot(212);
% % plot(t,rad2deg(tet));
% 
% load DadosRede.mat;
% input = [input; intensity1, intensity2];
% output = [output; gam(end), tet(end)];
% save DadosRede.mat input output;
% 
% 
% 
% %% Stoping V-REP simulation
% vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
% fprintf('Ending\n');
% vrep.simxFinish(clientID);
% vrep.delete();
% 
% vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
% fprintf('Ending\n');
% vrep.simxFinish(clientID);
% 
% end
% %%
% for count = 1:1000
% 
% clc, close all;
% 
% %% Loading V-REP remote interface - client side
% vrep=remApi('remoteApi');
% %% Closing any previously opened connections
% vrep.simxFinish(-1);
% %% Connecting to remote V-REP API server
% retCod=0;
% connAdd='127.0.0.1';
% connPor=19997;
% wuc=true;
% dnrod=true;
% toi=5000;
% ctci=5;
% %[clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
% while(retCod == 0)
%     [clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
%     if(clientID > -1)
%         fprintf('Starting\n');
%         retCod=1;
%     else
%         fprintf ('Waiting\n');
%     end
% end
% %% Getting robot handles
% [~,p3dxBody] = vrep.simxGetObjectHandle(clientID,'p3dx',vrep.simx_opmode_blocking);
% [~,leftMotor] = vrep.simxGetObjectHandle(clientID,'p3dx_leftMotor',vrep.simx_opmode_blocking);
% [~,rightMotor] = vrep.simxGetObjectHandle(clientID,'p3dx_rightMotor',vrep.simx_opmode_blocking);
% [~,goal] = vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_blocking);
% 
% 
% %% Starting V-REP simulation
% [retCodeS]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
% e=1;
% % while (e~=0),
% %     [e,result,~,~,~]=vrep.simxReadProximitySensor(clientID,noseSensor,vrep.simx_opmode_streaming);
% % end
% %% Main control loop - V-REP client side
% driveBackTime=0;
% t0=cputime;
% 
% np=25; %Tempo máximo de espera
% hd=100e-3;
% tf=np*hd;
% 
% tc=0;
% td=0;
% id=1;
% t=zeros(np,1);
% xp=zeros(np,1);
% yp=zeros(np,1);
% fp=zeros(np,1);
% xRef = zeros(np,1);
% yRef = zeros(np,1);
% fRef = zeros(np,1);
% intensity1 = zeros(1,7);
% intensity2 = zeros(1,7);
% gam = zeros(np,1);
% tet = zeros(np,1);
% 
% 
% 
% %% Load path
% load path.mat
% 
% %% Measure Dist
% 
% distLim = 0.1;
% dist = 0;
% 
%         [~,intensity1(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS01',vrep.simx_opmode_streaming);
%         [~,intensity1(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS02',vrep.simx_opmode_streaming);
%         [~,intensity1(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS03',vrep.simx_opmode_streaming);
%         [~,intensity1(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS04',vrep.simx_opmode_streaming);
%         [~,intensity1(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS05',vrep.simx_opmode_streaming);
%         [~,intensity1(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS06',vrep.simx_opmode_streaming);
%         [~,intensity1(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS07',vrep.simx_opmode_streaming);
%         
%         [~,intensity2(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS08',vrep.simx_opmode_streaming);
%         [~,intensity2(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS09',vrep.simx_opmode_streaming);
%         [~,intensity2(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS10',vrep.simx_opmode_streaming);
%         [~,intensity2(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS11',vrep.simx_opmode_streaming);
%         [~,intensity2(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS12',vrep.simx_opmode_streaming);
%         [~,intensity2(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS13',vrep.simx_opmode_streaming);
%         [~,intensity2(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS14',vrep.simx_opmode_streaming);
%         
%        load rngVectors.mat; 
%        vrep.simxSetObjectPosition(clientID,p3dxBody,-1,[-2.1+1.5*randX(count), -1.7+0.5*randY(count), 0.1388], vrep.simx_opmode_streaming);
%        vrep.simxSetObjectOrientation(clientID,p3dxBody,-1,[deg2rad(0.0), deg2rad(0.0), deg2rad(-50+100*randF(count))], vrep.simx_opmode_streaming);
% 
% while ((tc < tf) || (id < 20))
%     tc=cputime-t0;
%     %% Current sampling instant
%     if tc > td
%         t(id)=tc;
%         %% Measuring
%         [~,p3dxBodyPos]=vrep.simxGetObjectPosition(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
%         [~,p3dxBodyOri]=vrep.simxGetObjectOrientation(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
%         [~,goalDist]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
%         % Intensity
%         [~,intensity1(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS01',vrep.simx_opmode_buffer);
%         [~,intensity1(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS02',vrep.simx_opmode_buffer);
%         [~,intensity1(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS03',vrep.simx_opmode_buffer);
%         [~,intensity1(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS04',vrep.simx_opmode_buffer);
%         [~,intensity1(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS05',vrep.simx_opmode_buffer);
%         [~,intensity1(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS06',vrep.simx_opmode_buffer);
%         [~,intensity1(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS07',vrep.simx_opmode_buffer);
%         
%         [~,intensity2(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS08',vrep.simx_opmode_buffer);
%         [~,intensity2(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS09',vrep.simx_opmode_buffer);
%         [~,intensity2(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS10',vrep.simx_opmode_buffer);
%         [~,intensity2(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS11',vrep.simx_opmode_buffer);
%         [~,intensity2(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS12',vrep.simx_opmode_buffer);
%         [~,intensity2(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS13',vrep.simx_opmode_buffer);
%         [~,intensity2(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS14',vrep.simx_opmode_buffer);
%         
%         % Robot pose
%         xa=p3dxBodyPos(1,1);
%         ya=p3dxBodyPos(1,2);
%         fa=p3dxBodyOri(1,3);
%         % Goal location
%         xg = goalDist(1,1);
%         yg = goalDist(1,2);
% 
%         %% Saving
%         xp(id)=xa;
%         yp(id)=ya;
%         fp(id)=fa;
%         xRef(id) = xg;
%         yRef(id) = yg;
% 
%         %% Velocity Calculus
%         dist = measureDist(xa,ya,xg,yg); % Distance to goal
%         
%         [gam(id), tet(id)] = SensorIdeal(xa, ya, fa, pathX, pathY);
%             vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_oneshot);
%             vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_oneshot);
% 
%         %% Next sampling instant
%         td=td+hd;
%         id=id+1;
%     end
% end
% 
% % t=t(1:id-20);
% % xp=xp(20:id-1);
% % yp=yp(20:id-1);
% % fp=fp(20:id-1);
% % xRef=xRef(20:id-1);
% % yRef=yRef(20:id-1);
% % fRef=fRef(20:id-1);
% % gam = gam(20:id-1);
% % tet = tet(20:id-1);
% 
% % figure;
% % subplot(211);
% % plot(t,gam);
% % subplot(212);
% % plot(t,rad2deg(tet));
% 
% load DadosRede.mat;
% input = [input; intensity1, intensity2];
% output = [output; gam(end), tet(end)];
% save DadosRede.mat input output;
% 
% 
% 
% %% Stoping V-REP simulation
% vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
% fprintf('Ending\n');
% vrep.simxFinish(clientID);
% vrep.delete();
% 
% vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
% fprintf('Ending\n');
% vrep.simxFinish(clientID);
% 
% end

for count = 1:500

clc, close all;

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
%[clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
while(retCod == 0)
    [clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
    if(clientID > -1)
        fprintf('Starting\n');
        retCod=1;
    else
        fprintf ('Waiting\n');
    end
end
%% Getting robot handles
[~,p3dxBody] = vrep.simxGetObjectHandle(clientID,'p3dx',vrep.simx_opmode_blocking);
[~,leftMotor] = vrep.simxGetObjectHandle(clientID,'p3dx_leftMotor',vrep.simx_opmode_blocking);
[~,rightMotor] = vrep.simxGetObjectHandle(clientID,'p3dx_rightMotor',vrep.simx_opmode_blocking);
[~,goal] = vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_blocking);


%% Starting V-REP simulation
[retCodeS]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
e=1;
% while (e~=0),
%     [e,result,~,~,~]=vrep.simxReadProximitySensor(clientID,noseSensor,vrep.simx_opmode_streaming);
% end
%% Main control loop - V-REP client side
driveBackTime=0;
t0=cputime;

np=25; %Tempo máximo de espera
hd=100e-3;
tf=np*hd;

tc=0;
td=0;
id=1;
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);
xRef = zeros(np,1);
yRef = zeros(np,1);
fRef = zeros(np,1);
intensity1 = zeros(1,7);
intensity2 = zeros(1,7);
gam = zeros(np,1);
tet = zeros(np,1);



%% Load path
load path.mat

%% Measure Dist

distLim = 0.1;
dist = 0;

        [~,intensity1(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS01',vrep.simx_opmode_streaming);
        [~,intensity1(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS02',vrep.simx_opmode_streaming);
        [~,intensity1(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS03',vrep.simx_opmode_streaming);
        [~,intensity1(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS04',vrep.simx_opmode_streaming);
        [~,intensity1(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS05',vrep.simx_opmode_streaming);
        [~,intensity1(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS06',vrep.simx_opmode_streaming);
        [~,intensity1(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS07',vrep.simx_opmode_streaming);
        
        [~,intensity2(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS08',vrep.simx_opmode_streaming);
        [~,intensity2(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS09',vrep.simx_opmode_streaming);
        [~,intensity2(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS10',vrep.simx_opmode_streaming);
        [~,intensity2(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS11',vrep.simx_opmode_streaming);
        [~,intensity2(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS12',vrep.simx_opmode_streaming);
        [~,intensity2(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS13',vrep.simx_opmode_streaming);
        [~,intensity2(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS14',vrep.simx_opmode_streaming);
        
       load rngVectors.mat; 
       vrep.simxSetObjectPosition(clientID,p3dxBody,-1,[-1.2+1.1*randX(count), -0.6+0.5*randY(count), 0.1388], vrep.simx_opmode_streaming);
       vrep.simxSetObjectOrientation(clientID,p3dxBody,-1,[deg2rad(0.0), deg2rad(0.0), deg2rad(-50+100*randF(count))], vrep.simx_opmode_streaming);

while ((tc < tf) || (id < 20))
    tc=cputime-t0;
    %% Current sampling instant
    if tc > td
        t(id)=tc;
        %% Measuring
        [~,p3dxBodyPos]=vrep.simxGetObjectPosition(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,p3dxBodyOri]=vrep.simxGetObjectOrientation(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,goalDist]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
        % Intensity
        [~,intensity1(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS01',vrep.simx_opmode_buffer);
        [~,intensity1(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS02',vrep.simx_opmode_buffer);
        [~,intensity1(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS03',vrep.simx_opmode_buffer);
        [~,intensity1(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS04',vrep.simx_opmode_buffer);
        [~,intensity1(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS05',vrep.simx_opmode_buffer);
        [~,intensity1(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS06',vrep.simx_opmode_buffer);
        [~,intensity1(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS07',vrep.simx_opmode_buffer);
        
        [~,intensity2(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS08',vrep.simx_opmode_buffer);
        [~,intensity2(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS09',vrep.simx_opmode_buffer);
        [~,intensity2(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS10',vrep.simx_opmode_buffer);
        [~,intensity2(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS11',vrep.simx_opmode_buffer);
        [~,intensity2(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS12',vrep.simx_opmode_buffer);
        [~,intensity2(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS13',vrep.simx_opmode_buffer);
        [~,intensity2(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS14',vrep.simx_opmode_buffer);
        
        % Robot pose
        xa=p3dxBodyPos(1,1);
        ya=p3dxBodyPos(1,2);
        fa=p3dxBodyOri(1,3);
        % Goal location
        xg = goalDist(1,1);
        yg = goalDist(1,2);

        %% Saving
        xp(id)=xa;
        yp(id)=ya;
        fp(id)=fa;
        xRef(id) = xg;
        yRef(id) = yg;

        %% Velocity Calculus
        dist = measureDist(xa,ya,xg,yg); % Distance to goal
        
        [gam(id), tet(id)] = SensorIdeal(xa, ya, fa, pathX, pathY);
            vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_oneshot);

        %% Next sampling instant
        td=td+hd;
        id=id+1;
    end
end

% t=t(1:id-20);
% xp=xp(20:id-1);
% yp=yp(20:id-1);
% fp=fp(20:id-1);
% xRef=xRef(20:id-1);
% yRef=yRef(20:id-1);
% fRef=fRef(20:id-1);
% gam = gam(20:id-1);
% tet = tet(20:id-1);

% figure;
% subplot(211);
% plot(t,gam);
% subplot(212);
% plot(t,rad2deg(tet));

load DadosRede.mat;
input = [input; intensity1, intensity2];
output = [output; gam(end), tet(end)];
save DadosRede.mat input output;



%% Stoping V-REP simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
vrep.simxFinish(clientID);
vrep.delete();

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
vrep.simxFinish(clientID);

end
%%
for count = 1:500

clc, close all;

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
%[clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
while(retCod == 0)
    [clientID]=vrep.simxStart(connAdd,connPor,wuc,dnrod,toi,ctci);
    if(clientID > -1)
        fprintf('Starting\n');
        retCod=1;
    else
        fprintf ('Waiting\n');
    end
end
%% Getting robot handles
[~,p3dxBody] = vrep.simxGetObjectHandle(clientID,'p3dx',vrep.simx_opmode_blocking);
[~,leftMotor] = vrep.simxGetObjectHandle(clientID,'p3dx_leftMotor',vrep.simx_opmode_blocking);
[~,rightMotor] = vrep.simxGetObjectHandle(clientID,'p3dx_rightMotor',vrep.simx_opmode_blocking);
[~,goal] = vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_blocking);


%% Starting V-REP simulation
[retCodeS]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait);
e=1;
% while (e~=0),
%     [e,result,~,~,~]=vrep.simxReadProximitySensor(clientID,noseSensor,vrep.simx_opmode_streaming);
% end
%% Main control loop - V-REP client side
driveBackTime=0;
t0=cputime;

np=25; %Tempo máximo de espera
hd=100e-3;
tf=np*hd;

tc=0;
td=0;
id=1;
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);
xRef = zeros(np,1);
yRef = zeros(np,1);
fRef = zeros(np,1);
intensity1 = zeros(1,7);
intensity2 = zeros(1,7);
gam = zeros(np,1);
tet = zeros(np,1);



%% Load path
load path.mat

%% Measure Dist

distLim = 0.1;
dist = 0;

        [~,intensity1(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS01',vrep.simx_opmode_streaming);
        [~,intensity1(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS02',vrep.simx_opmode_streaming);
        [~,intensity1(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS03',vrep.simx_opmode_streaming);
        [~,intensity1(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS04',vrep.simx_opmode_streaming);
        [~,intensity1(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS05',vrep.simx_opmode_streaming);
        [~,intensity1(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS06',vrep.simx_opmode_streaming);
        [~,intensity1(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS07',vrep.simx_opmode_streaming);
        
        [~,intensity2(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS08',vrep.simx_opmode_streaming);
        [~,intensity2(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS09',vrep.simx_opmode_streaming);
        [~,intensity2(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS10',vrep.simx_opmode_streaming);
        [~,intensity2(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS11',vrep.simx_opmode_streaming);
        [~,intensity2(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS12',vrep.simx_opmode_streaming);
        [~,intensity2(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS13',vrep.simx_opmode_streaming);
        [~,intensity2(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS14',vrep.simx_opmode_streaming);
        
       load rngVectors.mat; 
       vrep.simxSetObjectPosition(clientID,p3dxBody,-1,[-0.15+0.5*randX(count), -1.7+1.5*randY(count), 0.1388], vrep.simx_opmode_streaming);
       vrep.simxSetObjectOrientation(clientID,p3dxBody,-1,[deg2rad(0.0), deg2rad(0.0), deg2rad(-150+100*randF(count))], vrep.simx_opmode_streaming);

while ((tc < tf) || (id < 20))
    tc=cputime-t0;
    %% Current sampling instant
    if tc > td
        t(id)=tc;
        %% Measuring
        [~,p3dxBodyPos]=vrep.simxGetObjectPosition(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,p3dxBodyOri]=vrep.simxGetObjectOrientation(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,goalDist]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
        % Intensity
        [~,intensity1(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS01',vrep.simx_opmode_buffer);
        [~,intensity1(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS02',vrep.simx_opmode_buffer);
        [~,intensity1(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS03',vrep.simx_opmode_buffer);
        [~,intensity1(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS04',vrep.simx_opmode_buffer);
        [~,intensity1(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS05',vrep.simx_opmode_buffer);
        [~,intensity1(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS06',vrep.simx_opmode_buffer);
        [~,intensity1(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS07',vrep.simx_opmode_buffer);
        
        [~,intensity2(1)]=vrep.simxGetFloatSignal(clientID,'IntensityLS08',vrep.simx_opmode_buffer);
        [~,intensity2(2)]=vrep.simxGetFloatSignal(clientID,'IntensityLS09',vrep.simx_opmode_buffer);
        [~,intensity2(3)]=vrep.simxGetFloatSignal(clientID,'IntensityLS10',vrep.simx_opmode_buffer);
        [~,intensity2(4)]=vrep.simxGetFloatSignal(clientID,'IntensityLS11',vrep.simx_opmode_buffer);
        [~,intensity2(5)]=vrep.simxGetFloatSignal(clientID,'IntensityLS12',vrep.simx_opmode_buffer);
        [~,intensity2(6)]=vrep.simxGetFloatSignal(clientID,'IntensityLS13',vrep.simx_opmode_buffer);
        [~,intensity2(7)]=vrep.simxGetFloatSignal(clientID,'IntensityLS14',vrep.simx_opmode_buffer);
        
        % Robot pose
        xa=p3dxBodyPos(1,1);
        ya=p3dxBodyPos(1,2);
        fa=p3dxBodyOri(1,3);
        % Goal location
        xg = goalDist(1,1);
        yg = goalDist(1,2);

        %% Saving
        xp(id)=xa;
        yp(id)=ya;
        fp(id)=fa;
        xRef(id) = xg;
        yRef(id) = yg;

        %% Velocity Calculus
        dist = measureDist(xa,ya,xg,yg); % Distance to goal
        
        [gam(id), tet(id)] = SensorIdeal(xa, ya, fa, pathX, pathY);
            vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_oneshot);

        %% Next sampling instant
        td=td+hd;
        id=id+1;
    end
end

% t=t(1:id-20);
% xp=xp(20:id-1);
% yp=yp(20:id-1);
% fp=fp(20:id-1);
% xRef=xRef(20:id-1);
% yRef=yRef(20:id-1);
% fRef=fRef(20:id-1);
% gam = gam(20:id-1);
% tet = tet(20:id-1);

% figure;
% subplot(211);
% plot(t,gam);
% subplot(212);
% plot(t,rad2deg(tet));

load DadosRede.mat;
input = [input; intensity1, intensity2];
output = [output; gam(end), tet(end)];
save DadosRede.mat input output;



%% Stoping V-REP simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
vrep.simxFinish(clientID);
vrep.delete();

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
vrep.simxFinish(clientID);

end
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
[~,p3dxBody] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
[~,leftMotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
[~,rightMotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
[~,sensorL1] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',vrep.simx_opmode_blocking);
[~,sensorL2] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_blocking);
[~,sensorR1] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',vrep.simx_opmode_blocking);
[~,sensorR2] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_blocking);
[~,sensorF1] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
[~,sensorF2] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4',vrep.simx_opmode_blocking);
[~,sensorF3] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3',vrep.simx_opmode_blocking);
[~,sensorF4] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor6',vrep.simx_opmode_blocking);
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

np=3000; %Tempo máximo de espera
hd=50e-3;
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
dS = zeros(np,1);
ab = zeros(np,1);

%% Robot parameters
D=195e-3;
R=D/2;
L=381e-3;


%% Fuzzy Controller
goalSeekController = readfis('GoalSeeking.fis');
obstacleAvoidController = readfis('ObstacleAvoidance.fis');
trackingController = readfis('Tracking.fis');

%% Measure Dist

distLim = 0.1;
dist = 0;

while (((dist > distLim) && (tc < tf)) || (id < 40))
    tc=cputime-t0;
    %% Current sampling instant
    if tc > td
        t(id)=tc;
        %% Measuring
        [~,p3dxBodyPos]=vrep.simxGetObjectPosition(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,p3dxBodyOri]=vrep.simxGetObjectOrientation(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,goalDist]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
        [~,sinalSensL1,sensDistL1,~,~]=vrep.simxReadProximitySensor(clientID,sensorL1,vrep.simx_opmode_streaming);
        [~,sinalSensL2,sensDistL2,~,~]=vrep.simxReadProximitySensor(clientID,sensorL2,vrep.simx_opmode_streaming);
        [~,sinalSensR1,sensDistR1,~,~]=vrep.simxReadProximitySensor(clientID,sensorR1,vrep.simx_opmode_streaming);
        [~,sinalSensR2,sensDistR2,~,~]=vrep.simxReadProximitySensor(clientID,sensorR2,vrep.simx_opmode_streaming);
        [~,sinalSensF1,sensDistF1,~,~]=vrep.simxReadProximitySensor(clientID,sensorF1,vrep.simx_opmode_streaming);
        [~,sinalSensF2,sensDistF2,~,~]=vrep.simxReadProximitySensor(clientID,sensorF2,vrep.simx_opmode_streaming);
        [~,sinalSensF3,sensDistF3,~,~]=vrep.simxReadProximitySensor(clientID,sensorF3,vrep.simx_opmode_streaming);
        [~,sinalSensF4,sensDistF4,~,~]=vrep.simxReadProximitySensor(clientID,sensorF4,vrep.simx_opmode_streaming);
        % Robot pose
        xa=p3dxBodyPos(1,1);
        ya=p3dxBodyPos(1,2);
        fa=p3dxBodyOri(1,3);
        % Goal location
        xg = goalDist(1,1);
        yg = goalDist(1,2);
        % Sensors
        Df1 = ajusteSensor(sqrt(sensDistF1(2)^2 + sensDistF1(3)^2), sinalSensF1);
        Df2 = ajusteSensor(sqrt(sensDistF2(2)^2 + sensDistF2(3)^2), sinalSensF2);
        Df3 = ajusteSensor(sqrt(sensDistF3(2)^2 + sensDistF3(3)^2), sinalSensF3);
        Df4 = ajusteSensor(sqrt(sensDistF4(2)^2 + sensDistF4(3)^2), sinalSensF4);
        Dr1 = ajusteSensor(sqrt(sensDistR1(2)^2 + sensDistR1(3)^2), sinalSensR1);
        Dr2 = ajusteSensor(sqrt(sensDistR2(2)^2 + sensDistR2(3)^2), sinalSensR2);
        Dl1 = ajusteSensor(sqrt(sensDistL1(2)^2 + sensDistL1(3)^2), sinalSensL1);
        Dl2 = ajusteSensor(sqrt(sensDistL2(2)^2 + sensDistL2(3)^2), sinalSensL2);
%         Df1 = ajusteSensor(sensDistF1(3), sinalSensF1);
%         Df2 = ajusteSensor(sensDistF2(3), sinalSensF2);
%         Df3 = ajusteSensor(sensDistF3(3), sinalSensF3);
%         Df4 = ajusteSensor(sensDistF4(3), sinalSensF4);
%         Dr1 = ajusteSensor(sensDistR1(3), sinalSensR1);
%         Dr2 = ajusteSensor(sensDistR2(3), sinalSensR2);
%         Dl1 = ajusteSensor(sensDistL1(3), sinalSensL1);
%         Dl2 = ajusteSensor(sensDistL2(3), sinalSensL2);
        Df = min([Df1, Df2, Df3, Df4]);
        Dr = min([Dr1, Dr2]);
        Dl = min([Dl1, Dl2]);
        %% Saving
        xp(id)=xa;
        yp(id)=ya;
        fp(id)=fa;
        xRef(id) = xg;
        yRef(id) = yg;

        %% Velocity Calculus
        dist = measureDist(xa,ya,xg,yg); % Distance to goal
        
        if(dist > distLim && id > 20)
            fd=atan2(yg-ya,xg-xa); 
            steeringAng=fa-fd;
            steeringAng=rad2deg(atan2(sin(steeringAng),cos(steeringAng)));          
            
            controllerGoal = evalfis([double(dist), double(steeringAng)], goalSeekController);
                        
            if(Dr <= Dl)
                Ds = -Dr;
            else
                Ds = Dl;
            end
            %fprintf('Dr = %.2f, Dl = %.2f, Df = %.2f\n', Dr, Dl, Df);
            controllerObstacle = evalfis([double(Df), double(Ds)], obstacleAvoidController);
            
            controllerTrack = evalfis([double(Dr), double(Dl), double(steeringAng)], trackingController);
            
            [W,V] = BehaviorController(controllerGoal, controllerObstacle, controllerTrack, Dr, Dl, Df);
            
            
            vr=(2*V-W*L)/2/R;
            vl=(2*V+W*L)/2/R;
            
            vrep.simxSetJointTargetVelocity(clientID,leftMotor,vl,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rightMotor,vr,vrep.simx_opmode_oneshot);
        else
            vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_oneshot);
        end
        %% Next sampling instant
        td=td+hd;
        id=id+1;
    end
end

t=t(1:id-20);
xp=xp(20:id-1);
yp=yp(20:id-1);
fp=fp(20:id-1);
xRef=xRef(20:id-1);
yRef=yRef(20:id-1);
fRef=fRef(20:id-1);
dS = dS(20:id-1);
ab = ab(20:id-1);

%plot(t,fp, t,dS, t,ab);

% figure(1)
% plot(t,xp,t,yp,t,fp),grid
% legend('x_p(t)','y_p(t)','\phi_p(t)')
% xlabel('t [s]')
% 
% figure(2)
% plot(t,xp,t,xRef),grid
% legend('x_p(t)')
% xlabel('t [s]')
% 
% figure(3)
% plot(t,yp,t,yRef),grid
% legend('y_p(t)')
% xlabel('t [s]')
% 
% figure(4)
% plot(t,fp,t,fRef),grid
% legend('f_p(t)')
% xlabel('t [s]')
% 
% 
% Eq. da reta ligando xp, yp a xRef yRef
figure(5)
plot(xp,yp, 'o',xg(end),yg(end),'x'),grid
axis([-2.5 2.5 -2.5 2.5]);
%legend('f_p(t)')
ylabel('eixo y (m)');
xlabel('eixo x (m)');
% 
% %save simControladorInicial.mat t xp yp fp xRef yRef fRef yReta;
% %save simFminconITSE.mat t xp yp fp xRef yRef fRef yReta;
% %save simGaITSE.mat t xp yp fp xRef yRef fRef yReta;
% save simFminconReta.mat t xp yp fp xRef yRef fRef yReta;

%% Stoping V-REP simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
vrep.simxFinish(clientID);
vrep.delete();

vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
vrep.simxFinish(clientID);
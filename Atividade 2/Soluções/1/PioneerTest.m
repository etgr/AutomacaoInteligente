clc, clear, close all;


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
[~,p3dxBody] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
[~,leftMotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
[~,rightMotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
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

np=400; %Tempo máximo de espera
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

%% Robot parameters
D=195e-3;
R=D/2;
L=381e-3;

v0=0.12;

%% PID controller
load Controlador.mat

errorPrev = 0;
integralSum = 0;
derivativePrev = 0; % se usado com filtro derivativo

%% Measure Dist

distLim = 0.01;
dist = 0;

while ((dist > distLim) && (tf < t0) || (id < 40))
    tc=cputime-t0;
    %% Current sampling instant
    if tc > td
        t(id)=tc;
        %% Measuring
        [~,p3dxBodyPos]=vrep.simxGetObjectPosition(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,p3dxBodyOri]=vrep.simxGetObjectOrientation(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,goalDist]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
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
        dist = measureDist(xa,ya,xg,yg);
        if(dist > distLim)
            [errorCurrent, fRef(id)] = calculateRefError(xa,ya,xg,yg,fa);
            if (Ki > 0)
                integralSum = Ki*(integralSum + errorCurrent*td);
            end
            %derivative = Kd*(errorCurrent-errorPrev)/td;    
            derivative = (derivativePrev + Kd*N*(errorCurrent - errorPrev))/(1+N*td);

            pidOut = Kp*errorCurrent + saturate(integralSum, 1e5)+ derivative;
            pidOut = saturate(pidOut,1e5);
            
            errorPrev = errorCurrent;
            derivativePrev = derivative;
            
            vr=(2*v0+pidOut*L)/2/R;
            vl=(2*v0-pidOut*L)/2/R;
            
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

t=t(1:id-3);
xp=xp(3:id-1);
yp=yp(3:id-1);
fp=fp(3:id-1);
xRef=xRef(3:id-1);
yRef=yRef(3:id-1);
fRef=fRef(3:id-1);

figure(1)
plot(t,xp,t,yp,t,fp),grid
legend('x_p(t)','y_p(t)','\phi_p(t)')
xlabel('t [s]')

figure(2)
plot(t,xp,t,xRef),grid
legend('x_p(t)')
xlabel('t [s]')

figure(3)
plot(t,yp,t,yRef),grid
legend('y_p(t)')
xlabel('t [s]')

figure(4)
plot(t,fp,t,fRef),grid
legend('f_p(t)')
xlabel('t [s]')


% Eq. da reta ligando xp, yp a xRef yRef
yReta = (yRef(1) - yp(1))/(xRef(1) - xp(1))*xp; 
figure(5)
plot(xp,yp,xp,yReta),grid
%legend('f_p(t)')
ylabel('eixo y');
xlabel('eixo x');

%% Stoping V-REP simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
vrep.simxFinish(clientID);
vrep.delete();
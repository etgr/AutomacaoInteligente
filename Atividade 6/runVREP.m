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

% Fotodiodos
[~,p3dxLS01] = vrep.simxGetObjectHandle(clientID,'p3dx_LS14',vrep.simx_opmode_blocking);
[~,p3dxLS02] = vrep.simxGetObjectHandle(clientID,'p3dx_LS13',vrep.simx_opmode_blocking);
[~,p3dxLS03] = vrep.simxGetObjectHandle(clientID,'p3dx_LS12',vrep.simx_opmode_blocking);
[~,p3dxLS04] = vrep.simxGetObjectHandle(clientID,'p3dx_LS11',vrep.simx_opmode_blocking);
[~,p3dxLS05] = vrep.simxGetObjectHandle(clientID,'p3dx_LS10',vrep.simx_opmode_blocking);
[~,p3dxLS06] = vrep.simxGetObjectHandle(clientID,'p3dx_LS09',vrep.simx_opmode_blocking);
[~,p3dxLS07] = vrep.simxGetObjectHandle(clientID,'p3dx_LS08',vrep.simx_opmode_blocking);
[~,p3dxLS08] = vrep.simxGetObjectHandle(clientID,'p3dx_LS07',vrep.simx_opmode_blocking);
[~,p3dxLS09] = vrep.simxGetObjectHandle(clientID,'p3dx_LS06',vrep.simx_opmode_blocking);
[~,p3dxLS10] = vrep.simxGetObjectHandle(clientID,'p3dx_LS05',vrep.simx_opmode_blocking);
[~,p3dxLS11] = vrep.simxGetObjectHandle(clientID,'p3dx_LS04',vrep.simx_opmode_blocking);
[~,p3dxLS12] = vrep.simxGetObjectHandle(clientID,'p3dx_LS04',vrep.simx_opmode_blocking);
[~,p3dxLS13] = vrep.simxGetObjectHandle(clientID,'p3dx_LS02',vrep.simx_opmode_blocking);
[~,p3dxLS14] = vrep.simxGetObjectHandle(clientID,'p3dx_LS01',vrep.simx_opmode_blocking);


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
intensity1 = zeros(1,7);
intensity2 = zeros(1,7);
gam = zeros(np,1);
tet = zeros(np,1);
gamA = zeros(np,1);
tetA = zeros(np,1);
gamB = zeros(np,1);
tetB = zeros(np,1);
gamC = zeros(np,1);
tetC = zeros(np,1);
testInten = zeros(np,1);

%% Robot parameters
D=195e-3;
R=D/2;
L=381e-3;
Ds=0.0954;
Ps=1;
Ls=0.1;


vp = 0.2;
% vp = 0.1;

%% Proportional controller
b = 2*L;
Rt = 0.2;
gammaC = 0.05;
%gammaC = 0.01;
Kgamma = 2*vp*b/(R*((Rt+2*gammaC)^2 - Rt^2));
Ktheta = sqrt(2*vp*Kgamma*b/R);

%% Nonlinear controller
alphaCont = 1/(gammaC*(gammaC+Rt));
betaCont = 2*vp*sqrt(alphaCont);
dtet = 0;

%% Fuzzy controller
GammaNorm = 0.15;
ThetaNorm = deg2rad(30);
dwNorm = 4;

ContFuzzy = readfis('ContFuzzy.fis');

%evalfis([double(dist), double(steeringAng)], goalSeekController);
%% Load path
load path.mat

%% Measure Dist

distLim = 0.1;
dist = 0;

[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS01,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS02,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS03,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS04,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS05,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS06,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS07,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS08,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS09,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS10,vrep.simx_opmode_streaming);
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS11,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS12,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS13,vrep.simx_opmode_streaming); 
[~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS14,vrep.simx_opmode_streaming); 

while (((dist > distLim) && (tc < tf)) || (id < 40))
    tc=cputime-t0;
    %% Current sampling instant
    if tc > td
        t(id)=tc;
        %% Measuring
        [~,p3dxBodyPos]=vrep.simxGetObjectPosition(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,p3dxBodyOri]=vrep.simxGetObjectOrientation(clientID,p3dxBody,-1,vrep.simx_opmode_streaming);
        [~,goalDist]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
        % Intensity
        intensity1 = zeros(1,7);
        intensity2 = zeros(1,7);
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS01,vrep.simx_opmode_streaming); 
        if(ds)
            intensity1(1) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS02,vrep.simx_opmode_streaming);
        if(ds)
            intensity1(2) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS03,vrep.simx_opmode_streaming);
        if(ds)
            intensity1(3) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS04,vrep.simx_opmode_streaming);
        if(ds)
            intensity1(4) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS05,vrep.simx_opmode_streaming);
        if(ds)
            intensity1(5) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS06,vrep.simx_opmode_streaming);
        if(ds)
            intensity1(6) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS07,vrep.simx_opmode_streaming);
        if(ds)
            intensity1(7) = double(inten(11));
        end
        %fprintf('%.2f\n', intensity1(3));
        
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS08,vrep.simx_opmode_streaming);
        if(ds)
            intensity2(1) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS09,vrep.simx_opmode_streaming);
        if(ds)
            intensity2(2) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS10,vrep.simx_opmode_streaming);
        if(ds)
            intensity2(3) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS11,vrep.simx_opmode_streaming);
        if(ds)
            intensity2(4) = double(inten(11));
        end
        testInten(id) = ds;%intensity2(4);
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS12,vrep.simx_opmode_streaming);
        if(ds)
            intensity2(5) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS13,vrep.simx_opmode_streaming);
        if(ds)
            intensity2(6) = double(inten(11));
        end
        [~,ds,inten,~] = vrep.simxReadVisionSensor(clientID,p3dxLS14,vrep.simx_opmode_streaming);
        if(ds)
            intensity2(7) = double(inten(11));
        end
        
        %fprintf('%.2f\n', intensity2(3));
        
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
        
        if(dist > distLim && id > 20)
            % Sensor Ideal
            [gam(id), tet(id)] = SensorIdeal(xa, ya, fa, pathX, pathY);
            
            % Algoritmo A
            [~,index1] = sort(intensity1);
            intensityIndex1a = index1(1);
            [~,index2] = sort(intensity2);
            intensityIndex2a = index2(1);
            pind1 = (intensityIndex1a - 4)/3;
            pind2 = ((intensityIndex2a + 7) - 11)/3;
            gamA(id) = (pind1 + pind2)*Ds/2;
            tetA(id) = atan((pind2 - pind1)*Ds/Ls);
            
            % Algorito B
            intensityIndex1b = index1(2);
            intensityIndex2b = index2(2);
            intensityIndex1 = (intensityIndex1a*(1-intensity1(intensityIndex1a)) + intensityIndex1b*(1-intensity1(intensityIndex1b)))/((1-intensity1(intensityIndex1a)) + (1-intensity1(intensityIndex1b)));
            intensityIndex2 = ((intensityIndex2a+7)*(1-intensity2(intensityIndex2a)) + (intensityIndex2b+7)*(1-intensity2(intensityIndex2b)))/((1-intensity2(intensityIndex2a)) + (1-intensity2(intensityIndex2b)));
            pind1 = (intensityIndex1 - 4)/3;
            pind2 = (intensityIndex2 - 11)/3;
            gamB(id) = (pind1 + pind2)*Ds/(2);
            tetB(id) = atan((pind2 - pind1)*Ds/Ls);
            
            % Algorito C
            intensityIndex1c = index1(3);
            intensityIndex2c = index2(3);
            intensityIndex1 = (intensityIndex1a*(1-intensity1(intensityIndex1a)) + intensityIndex1b*(1-intensity1(intensityIndex1b)) + intensityIndex1c*(1-intensity1(intensityIndex1c)))/((1-intensity1(intensityIndex1a)) + (1-intensity1(intensityIndex1b)) + (1-intensity1(intensityIndex1c)));
            intensityIndex2 = ((intensityIndex2a+7)*(1-intensity2(intensityIndex2a)) + (intensityIndex2b+7)*(1-intensity2(intensityIndex2b)) + (intensityIndex2c+7)*(1-intensity2(intensityIndex2c)))/((1-intensity2(intensityIndex2a)) + (1-intensity2(intensityIndex2b)) + (1-intensity2(intensityIndex2c)));
            pind1 = (intensityIndex1 - 4)/3;
            pind2 = (intensityIndex2 - 11)/3;
            gamC(id) = (pind1 + pind2)*Ds/(2);
            tetC(id) = atan((pind2 - pind1)*Ds/Ls);
            
            
            Gamma = gam(id);
            Theta = tet(id);
            
            % Cont Linear

%             epsilon = Gamma*cos(Theta);
%             Ke = Kgamma/cos(Theta);
%             dw = Ke*epsilon + Ktheta*Theta;
            
            % Cont não linear
            %dw = (b/(2*R))* ((alphaCont*Gamma*vp*sin(Theta)*cos(Theta)/Theta) + betaCont*Theta);
            
            % Cont Fuzzy
            if(Theta > 1.5*ThetaNorm)
                tetFuzzy = 1.5;
            elseif(Theta < -1.5*ThetaNorm)
                tetFuzzy = -1.5;
            else
                tetFuzzy = Theta/ThetaNorm;
            end
            if(Gamma > 1.5*GammaNorm)
                gamFuzzy = 1.5;
            elseif(Gamma < -1.5*GammaNorm)
                gamFuzzy = -1.5;
            else
                gamFuzzy = Gamma/GammaNorm;
            end
            dwFuzzy = evalfis([double(gamFuzzy), double(tetFuzzy)], ContFuzzy);
            dw = dwNorm*dwFuzzy;
            
            % Cont híbrido
            %dtetprev = dtet;
            %dtet = ContHibrido(Gamma,dtetprev);
            %K2 = 32;
            %dtet = K2 * Gamma;
            
            vr = (vp/R) + dw;
            vl = (vp/R) - dw;
            %vr = (vp/R) + dtet*L/(2*R);
            %vl = (vp/R) - dtet*L/(2*R);
            
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
gam = gam(20:id-1);
tet = tet(20:id-1);
gamA = gamA(20:id-1);
tetA = tetA(20:id-1);
gamB = gamB(20:id-1);
tetB = tetB(20:id-1);
gamC = gamC(20:id-1);
tetC = tetC(20:id-1);
testInten = testInten(20:id-1);
figure;
subplot(211);
plot(t,gam,t,gamA);
xlabel('Tempo (s)');
ylabel('\Gamma (cm)');
legend('Real', 'Algoritmo A');
title('Algoritmo A');
subplot(212);
plot(t,rad2deg(tet),t,rad2deg(tetA));
xlabel('Tempo (s)');
ylabel('\Theta (º)');
title('\Theta');
legend('Real', 'Algoritmo A');

figure;
subplot(211);
plot(t,gam,t,gamB);
xlabel('Tempo (s)');
ylabel('\Gamma (cm)');
legend('Real', 'Algoritmo B');
title('Algoritmo B');
subplot(212);
plot(t,rad2deg(tet),t,rad2deg(tetB));
xlabel('Tempo (s)');
ylabel('\Theta (º)');
title('\Theta');
legend('Real', 'Algoritmo B');

figure;
subplot(211);
plot(t,gam,t,gamC);
xlabel('Tempo (s)');
ylabel('\Gamma (cm)');
legend('Real', 'Algoritmo C');
title('Algoritmo C');
subplot(212);
plot(t,rad2deg(tet),t,rad2deg(tetC));
xlabel('Tempo (s)');
ylabel('\Theta (º)');
title('\Theta');
legend('Real', 'Algoritmo C');

figure;
plot(t,testInten);

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
% figure(5)
% plot(xp,yp, 'o',xg(end),yg(end),'o'),grid
% axis([-2.5 2.5 -2.5 2.5]);
% %legend('f_p(t)')
% ylabel('eixo y');
% xlabel('eixo x');
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
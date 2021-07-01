clc, clear, close all;

yg = 2;
xg = 2;
x0 = -2;
y0 = -2;

% Controlador Inicial

load Controlador.mat

[xpI,ypI,tI,fpI] = PioneerTest(Kp,Ki,Kd,50);

e=sqrt((xg-xpI).^2+(yg-ypI).^2);

aR = -(yg-y0)/(xg-x0);
cR = -y0 - aR*x0;
bR = 1;
dI=abs(aR*xpI + bR*ypI + cR)/sqrt(aR^2+bR^2);

distI=max(dI);

itseI=sum(tI.*e.^2);

%% Teste ITSE
% Controlador Fmincon

load ControladorFminconITSE.mat

[xp1,yp1,t1,fp1,fRef] = PioneerTest(Kp,Ki,Kd,50);
yReta = (yg - y0)/(xg - x0)*(xp1 - x0) + y0;

e=sqrt((xg-xp1).^2+(yg-yp1).^2);

itse1=sum(t1.*e.^2);

% Controlador GA

load ControladorGAITSE.mat

[xp2,yp2,t2,fp2] = PioneerTest(Kp,Ki,Kd,50);

e=sqrt((xg-xp2).^2+(yg-yp2).^2);

itse2=sum(t2.*e.^2);

figure(1);
plot(xp1, yReta, '--', xpI, ypI, xp1,yp1, xp2, yp2);
ylabel('Eixo y');
xlabel('Eixo x');
legend('Referência', 'Inicial', 'fmincon', 'ga');

figure(2);
plot(t1, fRef, '--', tI, fpI, t1,fp1, t2, fp2);
ylabel('\phi_p(t)');
xlabel('t(s)');
legend('Referência', 'Inicial', 'fmincon', 'ga');

%% Testes Distancia

% Controlador Fmincon

load ControladorFminconDist.mat

[xp1,yp1,t1,fp1,fRef] = PioneerTest(Kp,Ki,Kd,50);

aR = -(yg-y0)/(xg-x0);
cR = -y0 - aR*x0;
bR = 1;
d1=abs(aR*xp1 + bR*yp1 + cR)/sqrt(aR^2+bR^2);

dist1=max(d1);

% Controlador GA

load ControladorGADist.mat


[xp2,yp2,t2,fp2] = PioneerTest(Kp,Ki,Kd,50);

aR = -(yg-y0)/(xg-x0);
cR = -y0 - aR*x0;
bR = 1;
d2=abs(aR*xp2 + bR*yp2 + cR)/sqrt(aR^2+bR^2);

dist2=max(d2);
yReta = (yg - y0)/(xg - x0)*(xp2 - x0) + y0;
figure(3);
plot(xp2, yReta, '--', xpI, ypI, xp1,yp1, xp2, yp2);
ylabel('Eixo y');
xlabel('Eixo x');
legend('Referência', 'Inicial', 'fmincon', 'ga');

figure(4);
plot(t1, fRef, '--', tI, fpI, t1,fp1, t2, fp2);
ylabel('\phi_p(t)');
xlabel('t(s)');
legend('Referência', 'Inicial', 'fmincon', 'ga');

figure(5);
plot(tI, dI, t1, d1, t2, d2);
ylabel('d(t)');
xlabel('t(s)');
legend('Inicial', 'fmincon', 'ga');

%% Testes Robustez

% Controlador Fmincon

load ControladorFminconRobust.mat

[xp1,yp1,t1,fp1] = PioneerTest(Kp,Ki,Kd,50);

% Controlador GA

load ControladorGARobust.mat

[xp2,yp2,t2,fp2,fRef] = PioneerTest(Kp,Ki,Kd,50);
yReta = (yg - y0)/(xg - x0)*(xp2 - x0) + y0;
figure(6);
plot(xp2, yReta, '--', xpI, ypI, xp1,yp1, xp2, yp2);
ylabel('Eixo y');
xlabel('Eixo x');
legend('Referência', 'Inicial', 'fmincon', 'ga');

figure(7);
plot(t2, fRef, '--', tI, fpI, t1,fp1, t2, fp2);
ylabel('\phi_p(t)');
xlabel('t(s)');
legend('Referência', 'Inicial', 'fmincon', 'ga');





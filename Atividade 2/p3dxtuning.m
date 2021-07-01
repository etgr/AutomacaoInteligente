clear all
close all
%
Kp=10; 
Ki=0.01;
Kd=0.001;
y0=[0; 0; 0; 0; 0];
endt=10;
[t,y] = ode45(@(t,y)ucyc(t,y,Kp,Ki,Kd),[0 endt],y0);
figure(1)
subplot(2,1,1)
plot(t,y(:,3:4))
subplot(2,1,2)
plot(t,y(:,5),t,atan2(3,1)*ones(size(t)))
str = sprintf('P = %d, I = %d e D = %d',Kp,Ki,Kd);
disp(str);
%
%options = optimset('display','iter','TolX',1e-6,'TolFun',1e-6);
%PID=fminsearch(@fobj,[Kp Ki Kd],options);

function ydot = ucyc(t,y,Kp,Ki,Kd)
%
xp=y(3,1);
yp=y(4,1);
fp=y(5,1);
%
v=0.25;
%
A=[0,1;0,-Kd/50];
B=[0;1];
C=[Kd/50,-(Kp+Kd*Kd/50)*(Kd/50)+(Ki+Kd*Kd/50)];
D=[(Kp+Kd*Kd/10)];
% Goal
xg=1;
yg=3;
fd=atan2(yg-yp,xg-xp);
% PID
e=fd-fp;
e=atan2(sin(e),cos(e));
xPID=A*[y(1,1);y(2,1)]+B*e;
w=C*[y(1,1);y(2,1)]+D*e;
%
ydot(1:2,1)=xPID;
ydot(3,1)=v*cos(y(5,1));
ydot(4,1)=v*sin(y(5,1));
ydot(5,1)=w;
end
%
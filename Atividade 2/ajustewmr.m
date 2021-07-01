function ajustewmr
%
clear all
close all
%
%fmincon
oldopts=optimset(@fmincon);
options1=optimset(oldopts,'display','iter');
%oldopts=gaoptimset(@ga);
%x0 = [2.474099e+00, 3.948139e-03, 1.186181e-07];
%opts = optimoptions('ga','PlotFcn', @gaplotbestf, 'display','iter','MutationFcn',@mutationadaptfeasible);
%opts.InitialPopulationMatrix = x0;
%options2=gaoptimset(oldopts,'display','iter','MutationFcn',@mutationadaptfeasible, 'PopulationSize',50,'InitialPopulationMatrix',x0);
%ode45
optionsODE=odeset('RelTol',1e-6);
%
kp0=1; ki0=0.1; kd0=0.01; N=50;
%
g = tf([(kp0 + kd0*N),(ki0 + kp0*N),ki0*N],[(kp0 + kd0*N),N*(kp0 + kd0*N),0]);
%
s1=-(ki0 - (ki0^2 - 2*ki0*kp0*N - 4*kd0*ki0*N^2 + kp0^2*N^2)^(1/2) + kp0*N)/(2*(kp0 + kd0*N));
s2=-(ki0 + (ki0^2 - 2*ki0*kp0*N - 4*kd0*ki0*N^2 + kp0^2*N^2)^(1/2) + kp0*N)/(2*(kp0 + kd0*N));
%
tfin=50;
xini=[0;0;-2;-2;-60*(pi/180)];
[t0,x0]=ode45(@(t,x)smf(t,x,kp0,ki0,kd0,N),[0 tfin],xini,optionsODE);
%
%fun=@fobj;
fun = @fobjLineDist;
A=[-1,0,0;0,-1,0;0,0,-1];b=[0;0;0];
Aeq=[];beq=[];
lb=[0,0,0];ub=[4,1,1];
nonlcon=@mycon;
nvars=3;
%
PID=fmincon(fun,[kp0 ki0 kd0],A,b,Aeq,beq,lb,ub,nonlcon,options1);
%PID=ga(fun,nvars,A,b,Aeq,beq,lb,ub,nonlcon,opts)
Kp=PID(1); Ki=PID(2); Kd=PID(3);
%
[t1,x1]=ode45(@(t,x)smf(t,x,Kp,Ki,Kd,N),[0 tfin],xini,optionsODE);
%
figure(1)
subplot(2,1,1)
plot(t0,x0(:,3),t1,x1(:,3),'LineWidth',2.0)
legend('x_{i}','x_{f}')
xlabel('t, (s)'), ylabel('x, (m)')
grid
%
subplot(2,1,2)
plot(t0,x0(:,4),t1,x1(:,4),'LineWidth',2.0)
legend('y_{i}','y_{f}'), ylabel('y, (m)')
xlabel('t, (s)')
grid
%
figure(2)
plot(x0(:,3),x0(:,4),'LineWidth',2.0)
xlabel('x, (m)'), ylabel('y, (m)')
grid
hold on
for i=1:round(length(x0(:,3))/30):length(x0(:,3)),
    desenherobo(x0(i,3),x0(i,4),x0(i,5),0.03);
end
plot(x1(:,3),x1(:,4),'r-','LineWidth',2.0),grid
for i=1:round(length(x1(:,3))/30):length(x1(:,3)),
    desenherobo(x1(i,3),x1(i,4),x1(i,5),0.03);
end
grid
hold off
%
str = sprintf('Kp = %d, Ki = %d e Kd = %d', kp0,ki0,kd0);
disp(str);
str = sprintf('Kp = %d, Ki = %d e Kd = %d', Kp,Ki,Kd);
disp(str);
%
function [i] = fobj(k)
%
P=k(1);
I=k(2);
D=k(3);
%
xi=[0;0;-2;-2;-60*(pi/180)];
[t,x]=ode45(@(t,x)smf(t,x,P,I,D,N),[0 tfin],xi,optionsODE);
%
xg=2;
yg=2;
%
xp=x(:,3);
yp=x(:,4);
%
e=sqrt((xg-xp).^2+(yg-yp).^2);
%
% itse   
i=sum(t.*e.^2);
% ise   
%i=sum(e.^2);
%i=1/(1+itse);
end
%

% Função objetivo distância da reta
function [i] = fobjLineDist(k)
%
P=k(1);
I=k(2);
D=k(3);
%
%xi=[0;0;-2;-2;-60*(pi/180)];
%[t,x]=ode45(@(t,x)smf(t,x,P,I,D,N),[0 tfin],xi,optionsODE);
%
xg=2;
yg=2;
%
% xp=x(:,3);
% yp=x(:,4);
xp0 = -2;
yp0 = -2;

run('PioneerTest.m');

% Reta by + ax + c = 0
aR = -(yg-yp0)/(xg-xp0);
cR = -yp0 - aR*xp0;
bR = 1;
d=abs(aR*xp + bR*yp + cR)/sqrt(aR^2+bR^2);
plot(d)

% Minimiza a distância máxima
i=max(d);
end


function [c,ceq] = mycon(x)
kp=x(1);ki=x(2);kd=x(3);n=N;
%
c(1) = -(ki+kp*n)/(kp+kd*n);
c(2) = -(ki*n)/(kp+kd*n);
c(3) = -(ki*(n+1)+kp*n)/(kp+kd*n);
%
ceq = [];
end
%
function xdot = smf(t,x,Kp,Ki,Kd,N)
%
xc=[x(1,1);x(2,1)];
%
xp=x(3,1);
yp=x(4,1);
fp=x(5,1);
%
Ac=[0,1;0,-N];
Bc=[0;1];
Cc=[(Ki*N)/(Kp + Kd*N), (Ki + Kp*N)/(Kp + Kd*N) - N];
Dc=[1];
% Goal
xg=2;
yg=2;
fd=atan2(yg-yp,xg-xp);
%
e=fd-fp;
e=atan2(sin(e),cos(e));
%
xdotc=Ac*xc+Bc*e;
w=Cc*xc+Dc*e;
%
delta=norm([xg-xp yg-yp]);
if delta < 0.01,
    v=0.0;
else
    v=0.12;
end
%
xdotp=[v*cos(fp);v*sin(fp);w];
%
xdot=[xdotc;xdotp];
end
%
function []=desenherobo(x,y,q,s)
%
p=[ 1              1/7     
   -3/7            1       
   -5/7            6/7     
   -5/7            5/7     
   -3/7            2/7     
   -3/7            0       
   -3/7           -2/7     
   -5/7           -5/7     
   -5/7           -6/7     
   -3/7           -1       
    1             -1/7     
    1              1/7 ];
%
p=s*p;
p=[p,ones(length(p),1)];
r=[cos(q),sin(q);-sin(q),cos(q);x,y];
p=p*r;
%
X=p(:,1); 
Y=p(:,2); 
plot(X,Y,'k-','LineWidth',1.5)
end
%
end
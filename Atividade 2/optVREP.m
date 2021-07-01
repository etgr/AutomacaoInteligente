clc, clear, close all;

%% Loading V-REP remote interface - client side
vrep=remApi('remoteApi');

oldopts=optimset(@fmincon);
options1=optimset(oldopts,'display','iter');
oldopts=gaoptimset(@ga);
x0 = [1, 0.1, 0.01];
opts = optimoptions('ga','display','iter','MutationFcn',@mutationadaptfeasible);
opts.InitialPopulationMatrix = x0;
%options2=gaoptimset(oldopts,'display','iter','MutationFcn',@mutationadaptfeasible, 'PopulationSize',50,'InitialPopulationMatrix',x0);

kp0=1; ki0=0.1; kd0=0.01; N=50;
%

%fun=@fobj;
fun = @fobjLineDist;
%fun = @fobjMS;
A=[-1,0,0;0,-1,0;0,0,-1];b=[0;0;0];
Aeq=[];beq=[];
lb=[0,0,0];ub=[4,1,1];
nonlcon=@mycon;
nvars=3;
%
%PID=fmincon(fun,[kp0 ki0 kd0],A,b,Aeq,beq,lb,ub,nonlcon,options1);
%Kp=PID(1); Ki=PID(2); Kd=PID(3);
%save ControladorFminconRobust.mat Kp Ki Kd;
PID=ga(fun,nvars,A,b,Aeq,beq,lb,ub,nonlcon,opts)
Kp=PID(1); Ki=PID(2); Kd=PID(3);
save ControladorGARobust.mat Kp Ki Kd;

function [i] = fobj(k)
%
P=k(1)
I=k(2)
D=k(3)

%
xg=2;
yg=2;
%
[xp,yp,t] = PioneerTest(P,I,D,50);
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
P=k(1)
I=k(2)
D=k(3)

xg=2;
yg=2;
xp0 = -2;
yp0 = -2;

[xp,yp] = PioneerTest(P,I,D,50);

% Reta by + ax + c = 0
aR = -(yg-yp0)/(xg-xp0);
cR = -yp0 - aR*xp0;
bR = 1;
d=abs(aR*xp + bR*yp + cR)/sqrt(aR^2+bR^2);

% Minimiza a distância máxima
i=max(d);
end

function [i] = fobjMS(k)
%
P=k(1);
I=k(2);
D=k(3);

xg=2;
yg=2;
xp0 = -2;
yp0 = -2;
fo = atan2(yg-yp0, xg-xp0);

v0 = 0.12;
N = 50;
MsRef = 1.3;

% Modelo do controlador
Ac=[0,1;0,-N];
Bc=[0;1];
Cc=[(I*N)/(P + D*N), (I + P*N)/(P + D*N) - N];
Dc=[1];

% Modelo do sistema
Am = [0, 0, -v0*sin(fo); 0, 0, v0*cos(fo)];
A = [Ac, zeros(2,3); zeros(2,2), Am; Cc, zeros(1,3)];
B = [Bc; zeros(2,1);Dc];
C = [0,0,0,0,1];

[num, den] = ss2tf(A,B,C,0);
L = tf(num,den);
S = 1/(1+L);
[MagS,~] = bode(S);
Ms = max(MagS);

i = abs(Ms - MsRef);

end

function [c,ceq] = mycon(x)
kp=x(1);ki=x(2);kd=x(3);n=50;
%
c(1) = -(ki+kp*n)/(kp+kd*n);
c(2) = -(ki*n)/(kp+kd*n);
c(3) = -(ki*(n+1)+kp*n)/(kp+kd*n);
%
ceq = [];
end

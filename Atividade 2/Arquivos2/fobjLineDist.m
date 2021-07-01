function [i] = fobjLineDist(k)
%
P=k(1);
I=k(2);
D=k(3);
%
xi=[0;0;0;0;-60*(pi/180)];
[t,x]=ode45(@(t,x)smf(t,x,P,I,D,N),[0 tfin],xi,optionsODE);
%
xg=1;
yg=3;
%
xp=x(:,3);
yp=x(:,4);
%
e=sqrt((xg-xp).^2+(yg-yp).^2);
 
i=sum(e.^2);

end

end


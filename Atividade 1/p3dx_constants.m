%% Constants
%P3DX
D = 0.195;
R = D/2;
L = 0.381;
M = 9;

%cuboid aproximation of robot
h = 0.165;
w = 0.455;
d = L;  

%Moment of Inertia
J = (1/12)*M*(w^2 + d^2);

%Motors constats
Ra = 1;
La = 0.1;
Kt = 0.01;
Kb = 0.08;
N = 38.1;


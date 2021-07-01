function [dtet] = ContHibrido(e, dtetprev)
%CONTHIBRIDO Summary of this function goes here
%   Detailed explanation goes here
e1 = 0.05;
e2 = 0.1;

if(abs(e) < e1) %estado start
    dtet = 0;
elseif(e > e2) % estado right
    dtet = pi;
elseif(e < -e2)
    dtet = -pi; % estado left
else
    dtet = dtetprev;
end


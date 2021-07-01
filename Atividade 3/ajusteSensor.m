function [distAjuste] = ajusteSensor(dist, bitSens)
%AJUSTESENSOR Summary of this function goes here
%   Detailed explanation goes here

if(bitSens == 0)
    distAjuste = 1;
else
    distAjuste = dist;
end

end

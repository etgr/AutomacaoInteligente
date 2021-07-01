function [errorCurrent, fRef] = calculateRefError(x,y,xg,yg,f)
fRef = atan2(yg-y,xg-x);
errorCurrent = fRef - f;
errorCurrent = atan2(sin(errorCurrent), cos(errorCurrent));
end


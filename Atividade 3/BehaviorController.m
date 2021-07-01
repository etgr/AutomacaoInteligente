function [W, V] = BehaviorController(goalControl, obstacleControl, trackControl, Dr, Dl, Df)
%BEHAVIORCONTROLLER Summary of this function goes here
%   Detailed explanation goes here

distThreshold = 0.7;
if(((Dr < distThreshold) || (Dl < distThreshold)) && (Df < distThreshold))
    W = deg2rad(obstacleControl(1));
    V = obstacleControl(2);
    %fprintf('Modo = Obstacle Avoid\n');
elseif((Dl > distThreshold) && (Dr > distThreshold) && (Df < distThreshold))
    W = deg2rad(obstacleControl(1));
    V = obstacleControl(2);
    %fprintf('Modo = Obstacle Avoid\n');
elseif((Dl < distThreshold) && (Dr < distThreshold) && (Df > distThreshold))
     W = deg2rad(trackControl(1));
     V = trackControl(2);
     %fprintf('Modo = Tracking\n');
elseif(((Dl < distThreshold) || (Dr < distThreshold)) && (Df > distThreshold))
     W = deg2rad(trackControl(1));
     V = trackControl(2);
     %fprintf('Modo = Tracking\n');
else
    W = deg2rad(goalControl(1));
    V = goalControl(2); 
    %fprintf('Modo = Goal Seek\n');
end
end


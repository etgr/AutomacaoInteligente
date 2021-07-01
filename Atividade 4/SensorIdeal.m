function [Gamma, Theta] = SensorIdeal(xa, ya, fa, pathX, pathY)
%SENSORIDEAL Summary of this function goes here
% %   Detailed explanation goes here
% 
% % dX = diff(pathX);
% % dY = diff(pathY);
% % fPath = atan2(dY,dX);
% % theta = fPath - fa;
% % Theta = atan2(sin(theta),cos(theta));
% % [Theta, iValidos] = find(abs(Theta) < 1.4835); %[-85º, 85º]
% % 
% % pathXrobotFrame = (pathY(iValidos) - ya)*sin(fa) + (pathX(iValidos) - xa)*cos(fa);
% % pathYrobotFrame = (pathY(iValidos) - ya)*cos(fa) - (pathX(iValidos) - xa)*sin(fa);
% % 
% % dist = sqrt(((xa-pathX(iValidos)).^2)+((ya-pathY(iValidos)).^2));
% % [~, indMin] = min(dist);
% % 
% % iG = 0; 
% % if(pathXrobotFrame(indMin) == 0)
% %     iG = indMin;
% % else
% %    di = 1;
% %    while(iG == 0)
% %        iminus = indMin - di;
% %        if(iminus < 1)
% %            iminus = 1;
% %        end
% %        iplus = indMin + di;
% %        if(iplus > length(iValidos))
% %            iplus = length(iValidos);
% %        end
% %        if(pathXrobotFrame(indMin)*pathXrobotFrame(iminus) < 0) % Mudança de sinal no eixo X
% %            iG = iminus;
% %        elseif(pathXrobotFrame(indMin)*pathXrobotFrame(iplus) < 0)
% %            iG = iplus;
% %        end
% %        di = di+1;
% %    end
% % end
% % 
% % Gamma = pathYrobotFrame(iG);
% % Theta = Theta(iG);
% 
% N = length(pathX);
% 
% % dist = sqrt(((xa-pathX).^2)+((ya-pathY).^2));
% % [Gamma, indMin] = min(dist);
% % 
% % if (indMin >= N) % Previnir out of range
% %    indMin = N-1; 
% % elseif (indMin <= 0)
% %    indMin = 1;
% % end
% 
% pathXrobotFrame = (pathY - ya)*sin(fa) + (pathX - xa)*cos(fa);
% pathYrobotFrame = (pathY - ya)*cos(fa) - (pathX - xa)*sin(fa);
% % 
% contInd = 1;
% for i = 1:N-1
%    if(pathXrobotFrame(i)*pathXrobotFrame(i+1) <= 0)
%        ind(contInd) = i+1;
%        contInd = contInd+1;
%    end
% end
% if(contInd == 1)
%    [~,indMin] = min(abs(pathXrobotFrame));
% else
%    [~,indMin] = min(abs(pathYrobotFrame(ind)));
% end
% Gamma = pathYrobotFrame(indMin);
% if(indMin == N)
%     indMin = N-1;
% end
% 
% % di = 0;
% % if(abs(pathXrobotFrame(indMin)) < abs(pathXrobotFrame(indMin-1)))
% %     di = 1;
% % elseif(abs(pathXrobotFrame(indMin)) > abs(pathXrobotFrame(indMin-1)))
% %     di = -1;
% % end
% % 
% % while(di ~= 0)
% %     indMin = indMin + di;
% %     if (indMin <= 0 || indMin >= N || (pathXrobotFrame(indMin-di)*pathXrobotFrame(indMin)) < 0)% Ocorre cruzamento no eixo X
% %         di = 0;
% %     end
% % end
% % Gamma = pathYrobotFrame(indMin);
% 
% fPath = atan2(pathY(indMin+1) - pathY(indMin), pathX(indMin+1) - pathX(indMin));
% % fGamma = atan2(ya - pathY(indMin), xa - pathX(indMin));
% % df = fGamma - fPath;
% % df = atan2(sin(df),cos(df));
% % if(df > 0)
% %     Gamma = -Gamma;
% % end
% theta = fPath - fa;
% Theta = atan2(sin(theta),cos(theta));

%% Inicial
% N = length(pathX);
% dist = sqrt(((xa-pathX).^2)+((ya-pathY).^2));
% [Gamma, indMin] = min(dist);
% 
% if (indMin >= N) % Previnir out of range
%    indMin = N-1; 
% elseif (indMin <= 0)
%    indMin = 1;
% end
% fPath = atan2(pathY(indMin+1) - pathY(indMin), pathX(indMin+1) - pathX(indMin));
% fGamma = atan2(ya - pathY(indMin), xa - pathX(indMin));
% df = fGamma - fPath;
% df = atan2(sin(df),cos(df));
% if(df > 0)
%     Gamma = -Gamma;
% end
% theta = fPath - fa;
% Theta = atan2(sin(theta),cos(theta));

%% Teste 1
% N = length(pathX);
% pathXrobotFrame = (pathY - ya)*sin(fa) + (pathX - xa)*cos(fa);
% pathYrobotFrame = (pathY - ya)*cos(fa) - (pathX - xa)*sin(fa);
% 
% contInd = 1;
% for i = 1:N-1
%    if(pathXrobotFrame(i)*pathXrobotFrame(i+1) <= 0)
%        ind(contInd) = i+1;
%        contInd = contInd+1;
%    end
% end
% if(contInd == 1)
%    [~,indMin] = min(abs(pathXrobotFrame));
% else
%    [~,indMin] = min(abs(pathYrobotFrame(ind)));
% end
% Gamma = pathYrobotFrame(indMin);
% if(indMin == N)
%     indMin = N-1;
% end
% fPath = atan2(pathY(indMin+1) - pathY(indMin), pathX(indMin+1) - pathX(indMin));
% theta = fPath - fa;
% Theta = atan2(sin(theta),cos(theta));

%% Teste 2
N = length(pathX);
dist = sqrt(((xa-pathX).^2)+((ya-pathY).^2));
[~, indMin] = min(dist);
pathXrobotFrame = (pathY - ya)*sin(fa) + (pathX - xa)*cos(fa);
pathYrobotFrame = (pathY - ya)*cos(fa) - (pathX - xa)*sin(fa);
stop = 0;
di = 1;
while (stop == 0)
    indNext = min(indMin + di, N);
    indPrev = max(indMin - di, 1);
    if (pathXrobotFrame(indMin)*pathXrobotFrame(indNext) <= 0)
       stop = 1;
       indMin = indNext;
    elseif (pathXrobotFrame(indMin)*pathXrobotFrame(indPrev) <= 0)
        stop = 1;
        indMin = indPrev;
    elseif (indNext == N && indPrev == 1)
        stop = 1;
        [~,indMin] = min(abs(pathXrobotFrame));
    else
        di = di + 1;
    end
end
Gamma = pathYrobotFrame(indMin);
if(indMin == N)
    indMin = N-1;
end
fPath = atan2(pathY(indMin+1) - pathY(indMin), pathX(indMin+1) - pathX(indMin));
theta = fPath - fa;
Theta = atan2(sin(theta),cos(theta));

end


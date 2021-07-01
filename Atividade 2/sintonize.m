function sintonize
clear all
close all
warning off
options = optimset('display','iter','TolX',1e-6,'TolFun',1e-6);
%
h = tf(1,[1 4 6 4 1]);%[1 8 28 56 70 56 28 8 1]);
P = 0.1; I = 0.1; D = 0.1;
g = tf([(P+D*(D/10)) (I+D*(D/10)) (I*D/10)],[1 (D/10) 0]);
ftmf = g*h/(1+g*h);
str = sprintf('P = %d, I = %d e D = %d', P,I,D);
disp(str);
endt=100;
tt = [0:1e-3:endt];
uu = ones(size(tt));
[y0,t0] = lsim(ftmf,uu,tt);
%
%PID=fminsearch(@fobj,[P I D],options);
%PID=fmincon(@fobj,[P I D],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options)
 nvars=3;
 oldoptions = gaoptimset(@ga);
 options = gaoptimset(oldoptions,'display','iter','TolCon',1e-6,'MutationFcn',@mutationadaptfeasible);
 PID=ga(@fobj,nvars,[],[],[],[],[0,0,0],[],[],[],options);
%
P = PID(1); I = PID(2); D = PID(3);
g = tf([(P+D*(D/10)) (I+D*(D/10)) (I*D/10)],[1 (D/10) 0]);
ftmf = g*h/(1+g*h);
%
str = sprintf('P = %d, I = %d e D = %d', P,I,D);
disp(str);
[y1,t1] = lsim(ftmf,uu,tt);
%
figure(1)
plot(t0,y0,'k-',t1,y1,'r-')
axis([0 endt 0 2])
grid
%
    function [i] = fobj(x)
        %
        P = x(1);
        I = x(2);
        D = x(3);
        %
        g = tf([(P+D*(D/10)) (I+D*(D/10)) (I*D/10)],[1 (D/10) 0]);
        ftmf = g*h/(1+g*h);
        %
        [y,t] = lsim(ftmf,uu,tt);
        %        
        e = ones(size(t)) - y;
        % ISE  i = sum((e).^2);
        % IAE  
        i = sum(abs(e));
        % ITAE i = sum(t.*abs(e));
        % ITSE i = sum(t.*e.^2);
        %i = 1/(1+i);
    end
end
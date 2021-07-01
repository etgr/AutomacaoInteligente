clc, clear, close;

load DadosRede.mat;

i = 1;
N = length(input);
while i <= N
   if ((min(input(1:7,i)) > 0.1) || (min(input(8:14,i) > 0.1)))
       input(:,i) = [];
       output(:,i) = [];
       N = N - 1;
   else
       i = i+1;
   end
end

save DadosRede2.mat input output;
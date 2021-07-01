clc, clear, close all;

load DadosRede2.mat

% Rede 1

% a = randperm(size(input,2));
% input = input(:,a);
% output = output(:,a);

% net1 = feedforwardnet(5);
% net1 = train(net1,input,output);
% view(net1)
% y = net1(input);
% perf = perform(net1,y,output);

% Rede 2
numLayers = 5;

%net2 = cascadeforwardnet(numLayers,'traingdx');
net2 = fitnet(numLayers,'trainbr');
%net2.performFcn='msereg';
%net2.trainParam.goal=1e-6;
net2.divideParam.trainRatio = 70/100;
net2.divideParam.valRatio = 15/100;
net2.divideParam.testRatio = 15/100;
% net2.layers{1}.transferFcn='tansig';
% net2.layers{2}.transferFcn='tansig';
% net2.trainFcn='trainbfg';
%net2.performParam.regularization = 0.0001;
view(net2)
net2 = train(net2,input,output);
y = net2(input);
perf = perform(net2,y,output);

%net2 = cascadeforwardnet(numLayers,'traingdx');
net3 = fitnet(numLayers,'trainbr');
%net2.performFcn='msereg';
%net3.trainParam.goal=1e-6;
net3.divideParam.trainRatio = 70/100;
net3.divideParam.valRatio = 15/100;
net3.divideParam.testRatio = 15/100;
% net2.layers{1}.transferFcn='tansig';
% net2.layers{2}.transferFcn='tansig';
% net2.trainFcn='trainbfg';
%net2.performParam.regularization = 0.0001;
view(net3)
net3 = train(net3,input,output(1,:));
y = net3(input);
perf = perform(net3,y,output(1,:));

%net4 = cascadeforwardnet(numLayers,'trainbr');
net4 = fitnet(5,'trainbr');
%net4 = feedforwardnet(5,'trainbr');
net4.performFcn='msereg';
%net4.trainParam.goal=1e-6;
net4.divideParam.trainRatio = 70/100;
net4.divideParam.valRatio = 15/100;
net4.divideParam.testRatio = 15/100;
%net4.layers{1}.transferFcn='logsig';
%net4.layers{2}.transferFcn='logsig';
%net4.performParam.regularization = 0.0001;
view(net4)
net4 = train(net4,input,output(2,:));
y2 = net4(input);
perf = perform(net4,y2,output(2,:));


save RedeNeural8.mat net3 net4;

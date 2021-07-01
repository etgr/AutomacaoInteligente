clc, clear, close all;

np=400;
hd=50e-3;
tf=np*hd;

%% Modelo no Simulink

% Tensões de armadura dos motores DC
vaR = 12;
vaL = 5;

sim('DDMRsimulink.slx');
tSim = DadosSim.time;
xpSim = DadosSim.signals.values(:,1);
ypSim = DadosSim.signals.values(:,2);
fpSim = DadosSim.signals.values(:,3);

%% Modelo no V-REP

%Velocidades dos pneus
vR = DadosSim.signals.values(end,4);
vL = DadosSim.signals.values(end,5);

%Parametros de simulação
tc=0;
td=0;
id=1;
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);

sinalTeste1 = vL*ones(1,np);
sinalTeste2 = vR*ones(1,np);

run('PioneerTest.m');

%% Resultados

figure(1)
plot(t,xp,t,yp,t,fp),grid
legend('x_p(t)','y_p(t)','\phi_p(t)')
xlabel('t [s]')

figure(2)
plot(tSim,xpSim,tSim,ypSim,tSim,fpSim),grid
legend('x_p(t)','y_p(t)','\phi_p(t)')
xlabel('t [s]')

% Comparação entre V-REP e Simulink
figure(3)
plot(t,xp, tSim,xpSim, '--'),grid
legend('V-REP', 'Simulink')
ylabel('x_p(t)')
xlabel('t [s]')

figure(4)
plot(t,yp, tSim,ypSim, '--'),grid
legend('V-REP', 'Simulink')
ylabel('y_p(t)')
xlabel('t [s]')

figure(5)
plot(t,fp, tSim,fpSim, '--'),grid
legend('V-REP', 'Simulink')
ylabel('\phi_p(t)')
xlabel('t [s]')

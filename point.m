clc
clear

xg = [5 5]; %Posi��o Desejada
x0 = [0 0 pi/2]; %Posi��o Inicial
r = sim('sl_drivepoint'); %Simula��o do Movimento
q = r.find('yout'); %Resultados da Simula��o




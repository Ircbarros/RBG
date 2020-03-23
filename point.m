clc
clear

xg = [5 5]; %Posição Desejada
x0 = [0 0 pi/2]; %Posição Inicial
r = sim('sl_drivepoint'); %Simulação do Movimento
q = r.find('yout'); %Resultados da Simulação




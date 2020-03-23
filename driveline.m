clc
clear

sl_driveline
L = [1 -2 4]; %Posição Desejada
x0 = [8 5 pi/2]; %Posição Inicial
r = sim('sl_driveline'); %Simulação do Movimento
q = r.find('yout'); %Resultados da Simulação



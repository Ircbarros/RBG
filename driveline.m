clc
clear

sl_driveline
L = [1 -2 4]; %Posi��o Desejada
x0 = [8 5 pi/2]; %Posi��o Inicial
r = sim('sl_driveline'); %Simula��o do Movimento
q = r.find('yout'); %Resultados da Simula��o



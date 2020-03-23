clc
clear

%Definindo os Pontos

path=[0.05 0.05;
    0.05 1.6;
    0.15 1.6; 
    0.15 0.05; 
    0.25 0.05; 
    0.25 1.6; 
    0.35 1.6; 
    0.35 0.05; 
    0.45 0.05; 
    0.45 1.6;
    0.55 1.6;
    0.55 0.05;
    0.65 0.05;
    0.65 1.6;
    0.75 1.6;
    0.75 0.05;
    0.85 0.05;
    0.85 1.6;
    0.95 1.6;
    0.95 0.05];

robotCurrentLocation = path(1,:); %posi��o inicial
robotGoal = path(end,:); %caminho desejado

initialOrientation = 90; %�ngulo entre o rob� e o eixo-x
robotCurrentPose = [robotCurrentLocation initialOrientation]; %Posi��o do robo em termos de [x,y,theta]

%Iniciando o Simulador
robotRadius = 0.05;
robot = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);

%Visualizando o Caminho
 plot(path(:,1), path(:,2),'k--d')
 xlim([0 0.982]) %Largura de uma placa solar
 ylim([0 1.638]) %Comprimento de uma placa solar

%Chamada do Controlador

controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.1; %Velocidade m�xima em m/s do Zumo
controller.MaxAngularVelocity = 1; %Velocidade angular em rad/s, r=2cm
controller.LookaheadDistance = 0.05; %Resposta do Controlador

%Path Following com Controlador
goalRadius = 0.05;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

controlRate = robotics.Rate(490); %Controlador em 490Hz
while( distanceToGoal > goalRadius )

    [v, omega] = controller(robot.getRobotPose);
    
    % Simula��o do Rob� utilizando as Sa�das do Controlador
    drive(robot, v, omega);

    % Extrai a posi��o P:{X,Y} do rob�
    robotCurrentPose = robot.getRobotPose;

    % Recalcula a dist�ncia at� o objetivo
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

    waitfor(controlRate);

end

drive(robot, 0,0);



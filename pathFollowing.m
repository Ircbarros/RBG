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

robotCurrentLocation = path(1,:); %posição inicial
robotGoal = path(end,:); %caminho desejado

initialOrientation = 90; %ângulo entre o robô e o eixo-x
robotCurrentPose = [robotCurrentLocation initialOrientation]; %Posição do robo em termos de [x,y,theta]
    
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

controller = robotics.PurePursuit
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.09; %Velocidade em m/s
controller.MaxAngularVelocity = 2; %Velocidade angular em rad/s
controller.LookaheadDistance = 1; %Lookahead

%Path Following com Controlador
goalRadius = 0.025;
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(10); %Controlador em 10Hz
while( distanceToGoal > goalRadius )

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.getRobotPose);

    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);

    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.getRobotPose;

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

    waitfor(controlRate);

end
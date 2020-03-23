#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <Wire.h>
#include <LSM303.h> 

#define SPEED_CALIB           200  //Velocidade de rotação durante a calibração
#define SPEED                 100  //Aproximadamente 15% da velocidade máxima
#define TURN_BASE_SPEED       100 //Velocidade Base ao efetuar a Rotação


#define CALIBRATION_SAMPLES 70  // Número de Leituras do Compasso quando efetuando calibração
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M valor do magnetômetro +/-2.5 gauss em escala completa
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M 220 Hz update rate do manetômetro

//Desvio máximo permitido (em degraus) do ângulo desejado que deverá ser alcançado antes de se locomover em linha reta
#define DEVIATION_THRESHOLD 5

ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
LSM303 compass;

//Início da calibração do compasso definindo os valores máximos/mínimos do magnetômetro
void setup()
{
  LSM303::vector<int16_t> running_min = {-5367, -3111, -16931}, running_max = {-2934, -418, -15753};
  unsigned char index;

  Serial.begin(9600);

  // Início da Biblioteca Wire colocando o protocolo I2C bus
  Wire.begin();

  // Inicio o módulo LSM303
  compass.init();

  // Aciona o acelerômetro e o manetômetro
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  button.waitForButton();

  Serial.println("Iniciando a Calibração");

  motors.setLeftSpeed(SPEED_CALIB);
  motors.setRightSpeed(-SPEED_CALIB);

  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Coleta a leitura do vetor magnético e armazena em compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    Serial.println(index);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  // Adiciona os valores calibrados em compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  button.waitForButton();
   
}

void loop()
{
  float heading, relative_heading;
  int speed;
  static float target_heading = averageHeading();

  // A orientação é dado em degraus distante do vetor magnético, aumentando no sentido horário
  heading = averageHeading();

  // Esta fórmula permite a orientação de acordo com o ângulo desejado
  relative_heading = relativeHeading(heading, target_heading);

  Serial.print("Ângulo Desejado: ");
  Serial.print(target_heading);
  Serial.print("    Ângulo Atual: ");
  Serial.print(heading);
  Serial.print("    Diferença: ");
  Serial.print(relative_heading);

    // Se o Zumo encontrou a direção irá seguir em linha reta por 2s, efetuando outra rotação após o tempo
  if(abs(relative_heading) < DEVIATION_THRESHOLD)
  {
    motors.setSpeeds(SPEED, SPEED);

    Serial.print("   Linha Reta");

    delay(4000);

    // Desliga os Motores para Evitar Interferência por um curto Intervalo de Tempo
    motors.setSpeeds(0, 0);
    delay(100);
    // Turn 90 degrees relative to the direction we are pointing.
    // This will help account for variable magnetic field, as opposed
    // to using fixed increments of 90 degrees from the initial
    // heading (which might have been measured in a different magnetic
    // field than the one the Zumo is experiencing now).
    // Note: fmod() is floating point modulo
    target_heading = fmod(averageHeading() + 90, 360);
  }
  else
  {
    // Evita o overshoot a médida que o Zumo se aproxima do objetivo
    // diminuindo a velocidade de rotação com o tempo. A velocidade do motor
    // deve ser reduzida para uma base mínima, variando através da diferença
    // de orientação.
    speed = SPEED*relative_heading/180;

    if (speed < 0)
      speed -= TURN_BASE_SPEED;
    else
      speed += TURN_BASE_SPEED;

    motors.setSpeeds(speed, -speed) ;

    Serial.print("   Turn");
  }
  Serial.println();
}

// Converte as componentes x e y do vetor de orientação em degraus
// Esta função é usada em vez do LSM303::heading() para a aceleração não afetar a compensação em tilt
// Assume-se que o robô está sempre em level

template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Relaciona a diferença de ângulo em degraus entre duas orientações
float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // Limita em -180 a 180 degraus
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Medição de 10 vetores para suavizar a interferência do motor
float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg é a medição média do vetor magnéticoa.
  return heading(avg);
}

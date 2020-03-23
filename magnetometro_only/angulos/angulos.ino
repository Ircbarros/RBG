#include <Wire.h>
#include <LSM303.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>


#define CALIBRATION_SAMPLES 150       // Número de Leituras do Compasso quando efetuando calibração
#define CRA_REG_M_220HZ    0x1C      // CRA_REG_M 220 Hz update rate do magnetômetro
#define SPEED               100  // Aproximadamente 15% da velocidade máxima
#define TURN_BASE_SPEED     75 // Velocidade Base ao efetuar a Rotação


ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
LSM303 compass;

//Variáveis para um Filtro Passa-Baixa
const float alpha = 1.8; //Constante do Filtro
float fXa = 0;
float fYa = 0;
float fZa = 0;
float fXm = 0;
float fYm = 0;
float fZm = 0;

//Início da calibração do compasso definindo os valores máximos/mínimos do magnetômetro
void setup() {
  
  Serial.begin(9600);
  
  // Início da Biblioteca Wire colocando o protocolo I2C bus
  Wire.begin();
  
  // Inicio o módulo LSM303
  compass.init();
  
  // Aciona o acelerômetro e o manetômetro
  compass.enableDefault();
  
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);
  }
  

void loop()
{
// Coleta a leitura do vetor magnético
compass.read();

compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);

float pitch, pitch_print, roll, roll_print, Heading, Xa_off, Ya_off, Za_off, Xa_cal, Ya_cal, Za_cal, Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal, fXm_comp, fYm_comp, acceleration;
int speed;

// Calibração do Acelerômetro
Xa_off = compass.a.x/16.0 + 19.278068; //Eixo-X
Ya_off = compass.a.y/16.0 + 19.572994; //Eixo-Y
Za_off = compass.a.z/16.0 - 66.759342; //Eixo-Z
Xa_cal =  27.221081*Xa_off + 0.155270*Ya_off  + 0.115521*Za_off;  //Eixo-X com faltor de correção
Ya_cal =  0.155270*Xa_off  + 26.775628*Ya_off + 0.144242*Za_off;  //Eixo-Y com faltor de correção
Za_cal =  0.115521*Xa_off  + 0.144242*Ya_off  + 24.643995*Za_off; //Eixo-Z com faltor de correção


// Calibração do Magnetômetro
Xm_off = compass.m.x*(10000.0/670.0)+62357.951393; // Ganho X [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems - vide datasheet)
Ym_off = compass.m.y*(10000.0/670.0)+26041.789917; // Ganho Y [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems - vide datasheet)
Zm_off = compass.m.z*(10000.0/600.0 )+289715.567676;  // Ganho Z [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems - vide datasheet)
Xm_cal =  1.311241*Xm_off - 0.053088*Ym_off - 0.021878*Zm_off; //Eixo-X com faltor de correção
Ym_cal = -0.053088*Xm_off + 1.265448*Ym_off + 0.069805*Zm_off; //Eixo-Y com faltor de correção
Zm_cal = -0.021878*Xm_off + 0.069805*Ym_off + 0.996998*Zm_off; //Eixo-Z com faltor de correção

// Filtro Passa-Baixa para o Acelerômetro
fXa = Xa_cal * alpha + (fXa * (1.0 - alpha));
fYa = Ya_cal * alpha + (fYa * (1.0 - alpha));
fZa = Za_cal * alpha + (fZa * (1.0 - alpha));

// Filtro Passa-Baixa para o Magnetômetro
fXm = Xm_cal * alpha + (fXm * (1.0 - alpha));
fYm = Ym_cal * alpha + (fYm * (1.0 - alpha));
fZm = Zm_cal * alpha + (fZm * (1.0 - alpha));

// Pitch and roll
roll  = (atan2(fYa, sqrt(fXa*fXa + fZa*fZa)));
pitch = (atan2(fXa, sqrt(fYa*fYa + fZa*fZa)));
roll_print = (roll*180.0/M_PI);
pitch_print = (pitch*180.0/M_PI);

// Tilt Compensation
  fXm_comp = fXm*cos(pitch)+fZm*sin(pitch);
fYm_comp = fXm*sin(roll)*sin(pitch)+fYm*cos(roll)-fZm*sin(roll)*cos(pitch);

// Arcotang de y/x
Heading = (atan2(fYm_comp,fXm_comp)*180.0)/M_PI;
if (Heading < 0)
Heading += 360;

//Aceleração
 LSM303::vector<float> const aInG = {
    (float)compass.a.x / 4096,
    (float)compass.a.y / 4096,
    (float)compass.a.z / 4096}
  ;
  float mag = sqrt(LSM303::vector_dot(&aInG, &aInG));


Serial.print("Pitch (X): "); Serial.print(pitch_print); Serial.print("  ");
Serial.print("Roll (Y): "); Serial.print(roll_print); Serial.print("  ");
Serial.print("Heading: "); Serial.println(Heading);
Serial.print("Aceleração: "); Serial.println(mag);
delay(250);
//FIM DO LOOP
}


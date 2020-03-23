#include <Wire.h>
#include <LSM303.h>
#include <math.h> 
LSM303 compass;

#define CALIBRATION_SAMPLES 70  // Número de Leituras do Compasso quando efetuando calibração
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M valor do magnetômetro +/-2.5 gauss em escala completa
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M 220 Hz update rate do manetômetro


void setup() {

Serial.begin(9600);

// Início da Biblioteca Wire colocando o protocolo I2C bus
Wire.begin();

// Inicio o módulo LSM303
compass.init();

// Aciona o acelerômetro e o magnetômetro
compass.enableDefault();

compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate


Serial.println("Magnetômetro Descalibrado(Unidades em Nanotesla)"); 
}

void loop() {

compass.read();
float Xm_print, Ym_print, Zm_print;
float Pi = 3.14159;

Xm_print = compass.m.x*(100000.0/1100.0); // Ganho X [LSB/Gauss](1.3 neste caso)
Ym_print = compass.m.y*(100000.0/1100.0); // Ganho Y [LSB/Gauss](1.3 neste caso)
Zm_print = compass.m.z*(100000.0/980.0 );  // Ganho Z [LSB/Gauss](1.3 neste caso)

//Cálculo do ângulo do vetor x, y
float heading = (atan2(Ym_print,Xm_print)*180)/Pi;

//Normalizar entre 0 e 360
if(heading < 0){
  heading = 360 + heading;
  
  //Serial.print(Xm_print, 10); Serial.print(" "); Serial.print(Ym_print, 10); Serial.print(" "); Serial.println(Zm_print, 10);
  //delay(125);
  Serial.print("Leitura do ângulo: ");
  Serial.println(heading);
  delay(500);
}

}

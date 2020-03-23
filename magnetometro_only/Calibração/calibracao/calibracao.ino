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

compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // 
compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate


Serial.println("Magnetômetro Descalibrado(Unidades em Nanotesla)"); 
}

void loop() {

compass.read();
float Xm_print, Ym_print, Zm_print;
float Pi = 3.14159;

Xm_print = compass.m.x*(10000.0/670.0); // Ganho X [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems)
Ym_print = compass.m.y*(10000.0/670.0); // Ganho Y [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems)
Zm_print = compass.m.z*(10000.0/600.0 );  // Ganho Z [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems)


Serial.print(Xm_print, 10); Serial.print(" "); Serial.print(Ym_print, 10); Serial.print(" "); Serial.println(Zm_print, 10);
delay(125);
}


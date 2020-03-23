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
float Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal;

//Cálculo Ajustado via Magneto
//Valores Combinados
Xm_off = compass.m.x*(10000.0/670.0)+62357.951393; // Ganho X [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems)
Ym_off = compass.m.y*(10000.0/670.0)+26041.789917; // Ganho Y [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems)
Zm_off = compass.m.z*(10000.0/600.0 )+289715.567676;  // Ganho Z [LSB/Gauss](+/- 2.5 gauss sensitivity to hopefully avoid overflow problems)

//Fator de Correção
Xm_cal =  1.311241*Xm_off - 0.053088*Ym_off - 0.021878*Zm_off; //Eixo-X com faltor de correção
Ym_cal = -0.053088*Xm_off + 1.265448*Ym_off + 0.069805*Zm_off; //Eixo-Y com faltor de correção
Zm_cal = -0.021878*Xm_off + 0.069805*Ym_off + 0.996998*Zm_off; //Eixo-Z com faltor de correção


Serial.print(Xm_cal, 10); Serial.print(" "); Serial.print(Ym_cal, 10); Serial.print(" "); Serial.println(Zm_cal, 10);
delay(125);
}


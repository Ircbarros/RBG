
#include <Wire.h>
#include <LSM303.h>
LSM303 compass;

void setup()
{
Serial.begin(9600);
Wire.begin();
compass.init();
compass.enableDefault();
Serial.println("Accelerometer Calibrated (Units in mGal)");
}

void loop()
{
compass.read();
float Xa_off, Ya_off, Za_off, Xa_cal, Ya_cal, Za_cal;

Xa_off = compass.a.x/16.0 + 19.278068; //X-axis combined bias (Non calibrated data - bias)
Ya_off = compass.a.y/16.0 + 19.572994; //Y-axis combined bias (Default: substracting bias)
Za_off = compass.a.z/16.0 - 66.759342; //Z-axis combined bias

Xa_cal =  27.221081*Xa_off + 0.155270*Ya_off  + 0.115521*Za_off; //X-axis correction for combined scale factors (Default: positive factors)
Ya_cal =  0.155270*Xa_off  + 26.775628*Ya_off + 0.144242*Za_off; //Y-axis correction for combined scale factors
Za_cal =  0.115521*Xa_off  + 0.144242*Ya_off  + 24.643995*Za_off; //Z-axis correction for combined scale factors

Serial.print(Xa_cal); Serial.print(" "); Serial.print(Ya_cal); Serial.print(" "); Serial.println(Za_cal);
delay(125);
}

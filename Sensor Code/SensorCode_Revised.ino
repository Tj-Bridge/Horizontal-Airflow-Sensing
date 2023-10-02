#include "Wire.h" 
#include "utility/twi.h"
#define TCAADDR 0x70

const int tle_addr = 0x35; // obtained by i2c scanner. No need to be changed.
const int config_reg = 0x11;
const int lp_mode = 0x13;

byte bx_high;
byte by_high;
byte bz_high;
byte temp_low;
byte bxy_low;
byte bz_low;

unsigned int bx_value;
unsigned int by_value;
unsigned int bz_value;
//unsigned int temp_value = (temp_high >> 4) | temp_low;

int16_t Bx1;
int16_t By1;
int16_t Bz1;
int16_t Bx2;
int16_t By2;
int16_t Bz2;
int16_t avg_x;
int16_t avg_y;
int16_t avg_z;
int16_t delta_x;
int16_t delta_y;
int16_t delta_z;
int list1[6] = {0};
int list2[6] = {0};
int num_pin = 2;

int sensorCalibration(int list[], int Bx, int By, int Bz) {
  if (list[0] == 0) {
        list[0] = Bx;
        list[1] = By;
        list[2] = Bz;
      }
  else {
    list[3] = Bx;
    list[4] = By;
    list[5] = Bz;
    delta_x = list[3] - list[0];
    delta_y = list[4] - list[1];  // Values are zeroed from their starting position 
    delta_z = list[5] - list[2];
      }
  }

// Function for I2C multiplexer switching
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

 
void setup() {
  
  Wire.begin();   //join I2C bus
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */); //From Terri
  Serial.begin(9600); //start Serial
  while(!Serial); //wait for Serial to be available

  for (int i = 0; i < num_pin; i++){
  tcaselect(i);
  Wire.beginTransmission(tle_addr);
  Wire.write(config_reg); //set pointer/access configuration register  
  Wire.write(lp_mode); //
  Wire.endTransmission(); //end configurations
  }
  
  delay(100); //delay to allow time for sensor the update
  Serial.print("Setup Complete\n\n");
}

void loop(){
 // Serial.println(123456);  This is for matlab visualization
  
  for(int i = 0; i < num_pin; i++){
    tcaselect(i);
    Serial.print(i);
    Serial.print("|");
    // Request 7 bytes from the sensor and return, if it didn't send enough re-configure
    while(Wire.requestFrom(tle_addr, 6) < 6){
      //Serial.println("Read failed");
      Wire.begin();   //join I2C bus
      Wire.beginTransmission(tle_addr);
      Wire.write(config_reg);
      Wire.write(lp_mode); 
      Wire.endTransmission();
    }
  
    // Read all registers to variables
    bx_high = Wire.read();
    by_high = Wire.read();
    bz_high = Wire.read();
    temp_low = Wire.read();
    bxy_low = Wire.read();
    bz_low = Wire.read();

    // Split the variables to get B field values
    bx_value = (bx_high << 4) | ((bxy_low & 0xF0)>>4);
    by_value = (by_high << 4) | (bxy_low & 0x0F);
    bz_value = (bz_high << 4) | (bz_low & 0x0F);
    //unsigned int temp_value = (temp_high >> 4) | temp_low;
    
    if (i == 0) {
      Bx1 = (int16_t)(bx_value << 4) / 16 ;
      By1 = (int16_t)(by_value << 4) / 16 ;
      Bz1 = (int16_t)(bz_value << 4) / 16 ;
      sensorCalibration(list1, Bx1, By1, Bz1);
      Serial.print(delta_x);
      Serial.print(",");   
      Serial.print(delta_y);
      Serial.print(",");
      Serial.print(delta_z);
      Serial.print(",");
      delay(50);
    }
    else {
      Bx2 = (int16_t)(bx_value << 4) / 16 ;
      By2 = (int16_t)(by_value << 4) / 16 ;
      Bz2 = (int16_t)(bz_value << 4) / 16 ;
      /*avg_x = (int16_t)(Bx1 + Bx2) / 2;
      avg_y = (int16_t)(By1 + By2) / 2;     //Obtains the average of the two readings
      avg_z = (int16_t)(Bz1 + Bz2) / 2;*/ 
      sensorCalibration(list2, Bx2, By2, Bz2);
      Serial.print(delta_x);
      Serial.print(",");
      Serial.print(delta_y);
      Serial.print(",");
      Serial.print(delta_z); 
      Serial.println(",");
      delay(50);
      }
  }
}
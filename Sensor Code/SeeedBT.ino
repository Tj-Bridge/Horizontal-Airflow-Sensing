#include <Wire.h>       // default I²C library
#include <LSM6DS3.h>
//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
#define ADDRESS 0x35


#include <ArduinoBLE.h>           // Bluetooth Library

#define DELAY 10       // value in ms, change to modify update rate
//#define INT_PIN 9       // XMC pin connected to the sensor interrupt
int list1[6] = {0};
int16_t Bx1;
int16_t By1;
int16_t Bz1;
int16_t delta_x1;
int16_t delta_y1;
int16_t delta_z1;
int16_t X_drone;
int16_t Y_drone;
int16_t Angle;
float magnitude;

//Filter Variables
const int maxWindowSize = 100; // Maximum allowable window size
int X_values[maxWindowSize];
int Y_values[maxWindowSize];
int X_sum = 0;
int Y_sum = 0;
int X_index = 0;
int Y_index = 0;
int windowSize = 20; // Initial averaging window size



// Initalizing global variables for sensor data to pass onto BLE
String p, t, m;

// BLE Service Name
BLEService customService("180C");

// BLE Characteristics
// Syntax: BLE<DATATYPE>Characteristic <NAME>(<UUID>, <PROPERTIES>, <DATA LENGTH>)
BLEStringCharacteristic imu_acc("2A56", BLERead | BLENotify, 20);

// Function prototype
void readValues();

void setup()
{
    // Initalizing all the sensors
    //Serial.begin(9600);
    pinMode(LED_BLUE,OUTPUT);
    if (!BLE.begin())
    {
        //Serial.println("BLE failed to Initiate");
        delay(500);
        while (1);
    }

        //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        //Serial.println("Device error");
    } else {
        //Serial.println("Device OK!");
    }

    // Setting BLE Name
    BLE.setLocalName("Arduino Environment Sensor");
    
    // Setting BLE Service Advertisment
    BLE.setAdvertisedService(customService);
    
    // Adding characteristics to BLE Service Advertisment
    customService.addCharacteristic(imu_acc);

    // Adding the service to the BLE stack
    BLE.addService(customService);

    // Start advertising
    BLE.advertise();
    //Serial.println("Bluetooth device is now active, waiting for connections...");
      // Init I2C peripheral
  Wire.begin();
  delay(3);
  Wire.setClock(400000);

  // Reset sensor
  Wire.beginTransmission(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure sensor
  Wire.beginTransmission(ADDRESS); // Sensor address
  Wire.write(0x10);             // Register address
  Wire.write(0b00000000);       // Config register: measure all channels, full range, odd CP parity
  Wire.write(0b00011001);       // MOD1 register: master controlled mode, one byte read command, enabled interrupt, no collision avoidance, odd FP parity
  //Wire.write(0x11);
  Wire.endTransmission();

  // Configure interrupt pin
 // pinMode(INT_PIN, INPUT);
 // attachInterrupt(digitalPinToInterrupt(INT_PIN), irq_handler, FALLING);

  //Serial.println("Bx By Bz T");

  // Trigger first measurement (empty write command with set trigger bits)
  Wire.beginTransmission(ADDRESS); // Sensor address
  Wire.write(0b00100000);       // trigger after I2c write frame is finished
  //Wire.write(0x20); 
  Wire.endTransmission();

  // Initialize averaging parameters
  memset(X_values, 0, sizeof(X_values));
  memset(Y_values, 0, sizeof(Y_values));
}

void loop()
{
   // Variable to check if cetral device is connected
    BLEDevice central = BLE.central();
    if (central)
    {
        //Serial.print("Connected to central: ");
        //Serial.println(central.address());
        while (central.connected())
        {
            delay(20);
            // Read values from sensors
            readValues();

            imu_acc.writeValue(m);
            digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on
            delay(50);                        // Wait for 50 ms
            digitalWrite(LED_BUILTIN, LOW);   // Turn the LED off
            delay(50);   

            //Serial.println(m);
            //Serial.println("\n");
           
        }
    }
    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());
}

void readValues(){
   // Readout the first 7 data bytes
    uint8_t buf[7];
    Wire.requestFrom(ADDRESS, 7);
    for (uint8_t i = 0; i < 7; i++) {
      buf[i] = Wire.read();
    }
  
    // Built 12 bit data 
    int16_t X = (int16_t)((buf[0] << 8) |  (buf[4] & 0xF0)) >> 4;
    int16_t Y = (int16_t)((buf[1] << 8) | ((buf[4] & 0x0F) << 4)) >> 4;
    int16_t Z = (int16_t)((buf[2] << 8) | ((buf[5] & 0x0F) << 4)) >> 4;
    //uint16_t T = (buf[3] << 4) | (buf[5] >> 4);
 
      Bx1 = X;
      By1 = Y ;
      Bz1 = Z ;
//      if (list1[0] == 0) {
//        list1[0] = Bx1;
//        list1[1] = By1;
//        list1[2] = Bz1;
//      }
//      else {
//        list1[3] = Bx1;
//        list1[4] = By1;
//        list1[5] = Bz1;
//        delta_x1 = list1[3] - list1[0];
//        delta_y1 = list1[4] - list1[1];  // Values are zeroed from their starting position
//        delta_z1 = list1[5] - list1[2];
//      }

      X_drone = By1;
      Y_drone = -Bx1;
      
//      // AVERAGING FILTER
//      // Update the sum by subtracting the oldest value and adding the new value
//      X_sum = X_sum - X_values[X_index] + X_drone;
//      Y_sum = Y_sum - Y_values[Y_index] + Y_drone;
//    
//      // Update the arrays with the new values
//      X_values[X_index] = X_drone;
//      Y_values[Y_index] = Y_drone;
//    
//      // Move to the next index, wrapping around at the end of the array
//      X_index = (X_index + 1) % windowSize;
//      Y_index = (Y_index + 1) % windowSize;
//    
//      // Calculate the running average for X_drone and Y_drone
//      float avg_X_drone = X_sum / (float)windowSize;
//      float avg_Y_drone = Y_sum / (float)windowSize;

      Angle = round(atan2(Y_drone,X_drone)*(180/PI))+180;
      //magnitude = sqrt(avg_X_drone**2+avg_Y_drone**2);

      m = String(Angle) + " " + String(X_drone) + " " + String(Y_drone); //+ " " + String(myIMU.readFloatGyroX())+ " " + String(myIMU.readFloatGyroY())+ " " + String(myIMU.readFloatGyroZ());
      //Serial.print(Angle);
//      Serial.print(",");
//      Serial.print(int(round(avg_X_drone)));
//      Serial.print(",");   
//      Serial.print(int(round(avg_Y_drone)));
//      Serial.print(",");
//      Serial.println(delta_z1);
      //delay(10);
 
    
    delay(DELAY);

    // Trigger new measurement (empty write command with set trigger bits)
    Wire.beginTransmission(ADDRESS); // Sensor address
    Wire.write(0b00100000);       // trigger after I2c write frame is finished
    //Wire.write(0x20); 
    Wire.endTransmission();
}

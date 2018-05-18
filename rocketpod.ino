#include "Adafruit_Si7021.h"
#include "SparkFunLIS3DH.h"
#include "Adafruit_AMG88xx.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4; //chip specifier
const byte trigger; // = interface for accel interupt
const byte numBytes;
const byte accel;
bool takeoff = false;

Adafruit_Si7021 sensor = Adafruit_Si7021();
Adafruit_AMG88xx amg;
LIS3DH myIMU(I2C_MODE, 0x19); //Default constructor is I2C, addr 0x19.
File dataFile;

void setup() {
  Wire.begin(); // twi init
  Serial.begin(9600); // usb init
  
    // wait for serial port to open
  while (!Serial) {
    delay(10);
  }
  Serial.println("RocketPod.c connection established");
  Serial.println("Initializing...");
  // init temp sensor
  Serial.print("Temperature Sensor Status :");
  if (!sensor.begin()) {
    Serial.println("FAILED");
  }
  else {
    Serial.println("OK");
  }


 // init lis3dh chip
  Serial.print("Thermal Camera Status     :");
  if (!amg.begin()) {
    Serial.println("FAILED");
  }
  else {
    Serial.println("OK");
  }
  
  pinMode(trigger, INPUT);
  attachInterrupt(digitalPinToInterrupt(trigger), wakeUp, RISING);

// Accel reg settings
  myIMU.settings.accelSampleRate = 50;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  myIMU.settings.accelRange = 8;      //Max G force readable.  Can be: 2, 4, 8, 16

  myIMU.settings.adcEnabled = 0;
  myIMU.settings.tempEnabled = 0;
  myIMU.settings.xAccelEnabled = 1;
  myIMU.settings.yAccelEnabled = 1;
  myIMU.settings.zAccelEnabled = 1;
  
  //Call .begin() to configure the IMU
  Serial.print("Accelerometer Status     :");
  if(!myIMU.begin()){
    Serial.println("FAILED\n\r");
  }
  else {
    Serial.println("OK\n\r");
  }
  configIntterupts();

  // init SD card
  Serial.print("SD Card Status            :");
  if (!SD.begin(chipSelect)){
    Serial.println("FAILED"); 
  }
  else {
    Serial.println("OK");
  }
  
  Serial.println("Creating example.txt...");
  dataFile = SD.open("example.txt", FILE_WRITE); //open file
  dataFile.close();
  delay(1);
  if (SD.exists("example.txt")) {
    Serial.println("example.txt exists.");
  } else {
    Serial.println("example.txt doesn't exist.");
  }
  
  dataFile = SD.open("rocketpod.txt",FILE_WRITE);
  if (dataFile) { 
    dataFile.println("RocketPod.c data log"); 
    dataFile.close();
  }
}

void wakeUp (){
  takeoff = true;
}

float sensHmdData, sensTmpData;
void loop() {

  //while (!takeoff){
  //}
  float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  amg.readPixels(pixels);
  sensHmdData = sensor.readHumidity();
  sensTmpData = sensor.readTemperature();
  Serial.print("Humidity:    "); Serial.print(sensHmdData, 2);
  Serial.print("\tTemperature: "); Serial.println(sensTmpData, 2);
  
  
  // SD write
  dataFile = SD.open("example.txt", FILE_WRITE); //open file
  if (dataFile) { 
    //dataFile.print("Humidity:    "); dataFile.print(sensHmdData, 2);
    //dataFile.print("  Temperature: "); dataFile.println(sensTmpData, 2); 
    for (int i =0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      dataFile.print(pixels[i], 2); dataFile.print(" ");
      Serial.print(pixels[i], 2); Serial.print(" ");
    }
    dataFile.println("\n\r");
    Serial.println("\n\r***");
    dataFile.close();
  }
  else {
    Serial.println("could not open file");
  }
//  delay(500);
}

void configIntterupts() {
     uint8_t dataToWrite = 0;

  //LIS3DH_INT1_CFG   
  //dataToWrite |= 0x80;//AOI, 0 = OR 1 = AND
  dataToWrite |= 0x40;//6D, 0 = interrupt source, 1 = 6 direction source
  //Set these to enable individual axes of generation source (or direction)
  // -- high and low are used generically
  dataToWrite |= 0x20;//Z high
  //dataToWrite |= 0x10;//Z low
  //dataToWrite |= 0x08;//Y high
  //dataToWrite |= 0x04;//Y low
  //dataToWrite |= 0x02;//X high
  //dataToWrite |= 0x01;//X low
  myIMU.writeRegister(LIS3DH_INT1_CFG, dataToWrite);
  
  //LIS3DH_INT1_THS   
  dataToWrite = 0;
  //Provide 7 bit value, 0x7F always equals max range by accelRange setting
  dataToWrite |= 0x20; // ~2/8 range
  myIMU.writeRegister(LIS3DH_INT1_THS, dataToWrite);

    //LIS3DH_INT1_DURATION  
  dataToWrite = 0;
  //minimum duration of the interrupt
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x01; // 1 * 1/50 s = 20ms
  myIMU.writeRegister(LIS3DH_INT1_DURATION, dataToWrite);

  // LIS3DH_ACT_THS
  dataToWrite = 0;
  // sleep-wakeUp thresh
  dataToWrite |= 0x18; // ~1.5G 
  myIMU.writeRegister(0x3E, dataToWrite);

  //LIS3DH_ACT_DUR
  dataToWrite = 0;
  // sleep - wakeUp dur
  dataToWrite |= 0x0;
  myIMU.writeRegister(0x3F, dataToWrite);
}


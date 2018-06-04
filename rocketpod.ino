#include "Adafruit_Si7021.h"
#include "SparkFunLIS3DH.h"
#include "Adafruit_AMG88xx.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define F_CPU 16000000 // for delay.h

#define LED_PORT_DDR DDRF 
#define LED_PORT PORTF 
#define LED_PIN PF6
#define TRIG_INT 0 // = interface for accel interupt

// Timer 1 OCR1A compare values adjusted for CTC mode 1 sec = 62499 .5 sec = 31249 .25 sec = 15624 1/16 sec = 3906
#define Int_Timer_Ticks 3906

const int chipSelect = 4; //chip specifier

const byte numBytes;
const byte accel;
float sensHmdData, sensTmpData;
bool takeoff = false;

uint8_t timeCntrQSec    = 0;
uint8_t timeCntrSeconds = 0;
uint8_t timeCntrMinutes = 0;
uint8_t timeCntrHours   = 0;
uint8_t dataGrab = 0;

Adafruit_Si7021 sensor = Adafruit_Si7021();
Adafruit_AMG88xx amg;
LIS3DH myIMU(I2C_MODE, 0x19); //Default constructor is I2C, addr 0x19.
File dataFile;

void setup() {
  timer_init();
  sei();
  DDRF = (1 << PF6);
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
  
  //pinMode(TRIG_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(TRIG_INT), wakeUp, RISING);

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


    Serial.println("Creating pdata.txt...");
    dataFile = SD.open("pdata.txt", FILE_WRITE); //open file
    dataFile.close();
    delay(1);
    if (SD.exists("pdata.txt")) {
      Serial.println("pdata.txt exists.");
    } else {
      Serial.println("pdata.txt doesn't exist.");
    }
  
    dataFile = SD.open("pdata.txt",FILE_WRITE);
    if (dataFile) { 
      dataFile.println("RocketPod data log"); 
      dataFile.close();
    }
}

void wakeUp (){
  if (!takeoff){
    dataFile = SD.open("pdata.txt", FILE_WRITE); //open file
    if (dataFile) { 
      dataFile.print("***** Lift Off ***** Time[");dataFile.print(timeCntrHours);dataFile.print(":");dataFile.print(timeCntrMinutes);dataFile.print(":"); 
      dataFile.print(timeCntrSeconds);dataFile.print(":");dataFile.print((timeCntrQSec*3.75));dataFile.println("]");
    dataFile.close();
    }
  }
  takeoff = true;
}

void TakePic(){

  float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
  amg.readPixels(pixels);
  
  
  // SD write
  dataFile = SD.open("pdata.txt", FILE_WRITE); //open file
  if (dataFile) { 
    for (int i =0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      dataFile.print(pixels[i], 2); dataFile.print(" ");
      Serial.print(pixels[i], 2); Serial.print(" ");
    }
    dataFile.println("");
    dataFile.close();
  }
  else {
    Serial.println("could not open file");
  }
}

void loop() {

   if (dataGrab || takeoff){
   //if (dataGrab){ 
      sensHmdData = sensor.readHumidity();
      sensTmpData = sensor.readTemperature();
      Serial.print(timeCntrHours);Serial.print(":");Serial.print(timeCntrMinutes);Serial.print(":"); 
      Serial.print(timeCntrSeconds);Serial.print(":");Serial.print((timeCntrQSec*3.75));  
      Serial.print("  Humidity:    "); Serial.print(sensHmdData, 2);
      Serial.print("  Temperature: "); Serial.print(sensTmpData, 2); 
      Serial.print("  Accelerometer:  ");Serial.print(" X = ");Serial.print(myIMU.readFloatAccelX(), 4);
      Serial.print(" Y = ");Serial.print(myIMU.readFloatAccelY(), 4);Serial.print(" Z = ");Serial.println(myIMU.readFloatAccelZ(), 4);
      dataGrab = 0;
      dataFile = SD.open("pdata.txt",FILE_WRITE);
      if (dataFile) {
        dataFile.print(timeCntrHours);dataFile.print(":");dataFile.print(timeCntrMinutes);dataFile.print(":"); 
        dataFile.print(timeCntrSeconds);dataFile.print(":");dataFile.print((timeCntrQSec*3.75));  
        dataFile.print("  Humidity:    "); dataFile.print(sensHmdData, 2);
        dataFile.print("  Temperature: "); dataFile.print(sensTmpData, 2); 
        dataFile.print("  Accelerometer:  ");dataFile.print(" X = ");dataFile.print(myIMU.readFloatAccelX(), 4);
        dataFile.print(" Y = ");dataFile.print(myIMU.readFloatAccelY(), 4);dataFile.print(" Z = ");dataFile.println(myIMU.readFloatAccelZ(), 4);
        dataFile.close();
      }
      else {
        Serial.print("file not found");
      }
      TakePic();
      }
   else {
    Serial.print(dataGrab);Serial.print(takeoff);Serial.print('\r');
   }
}

/*void configIntterupts() {
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
  dataToWrite |= 0x01;
  myIMU.writeRegister(0x3F, dataToWrite);
  
}*/
void configIntterupts()
{
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
  dataToWrite |= 0x20; // 1/8 range
  myIMU.writeRegister(LIS3DH_INT1_THS, dataToWrite);
  
  //LIS3DH_INT1_DURATION  
  dataToWrite = 0;
  //minimum duration of the interrupt
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x01; // 1 * 1/50 s = 20ms
  myIMU.writeRegister(LIS3DH_INT1_DURATION, dataToWrite);
  
  //LIS3DH_CLICK_CFG   
  dataToWrite = 0;
  //Set these to enable individual axes of generation source (or direction)
  // -- set = 1 to enable
  //dataToWrite |= 0x20;//Z double-click
  dataToWrite |= 0x10;//Z click
  //dataToWrite |= 0x08;//Y double-click 
  dataToWrite |= 0x04;//Y click
  //dataToWrite |= 0x02;//X double-click
  dataToWrite |= 0x01;//X click
  myIMU.writeRegister(LIS3DH_CLICK_CFG, dataToWrite);
  
  //LIS3DH_CLICK_SRC
  dataToWrite = 0;
  //Set these to enable click behaviors (also read to check status)
  // -- set = 1 to enable
  //dataToWrite |= 0x20;//Enable double clicks
  dataToWrite |= 0x04;//Enable single clicks
  //dataToWrite |= 0x08;//sine (0 is positive, 1 is negative)
  dataToWrite |= 0x04;//Z click detect enabled
  dataToWrite |= 0x02;//Y click detect enabled
  dataToWrite |= 0x01;//X click detect enabled
  myIMU.writeRegister(LIS3DH_CLICK_SRC, dataToWrite);
  
  //LIS3DH_CLICK_THS   
  dataToWrite = 0;
  //This sets the threshold where the click detection process is activated.
  //Provide 7 bit value, 0x7F always equals max range by accelRange setting
  dataToWrite |= 0x0A; // ~1/16 range
  myIMU.writeRegister(LIS3DH_CLICK_THS, dataToWrite);
  
  //LIS3DH_TIME_LIMIT  
  dataToWrite = 0;
  //Time acceleration has to fall below threshold for a valid click.
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x08; // 8 * 1/50 s = 160ms
  myIMU.writeRegister(LIS3DH_TIME_LIMIT, dataToWrite);
  
  //LIS3DH_TIME_LATENCY
  dataToWrite = 0;
  //hold-off time before allowing detection after click event
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x08; // 4 * 1/50 s = 160ms
  myIMU.writeRegister(LIS3DH_TIME_LATENCY, dataToWrite);
  
  //LIS3DH_TIME_WINDOW 
  dataToWrite = 0;
  //hold-off time before allowing detection after click event
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x10; // 16 * 1/50 s = 320ms
  myIMU.writeRegister(LIS3DH_TIME_WINDOW, dataToWrite);

  //LIS3DH_CTRL_REG5
  //Int1 latch interrupt and 4D on  int1 (preserve fifo en)
  myIMU.readRegister(&dataToWrite, LIS3DH_CTRL_REG5);
  dataToWrite &= 0xF3; //Clear bits of interest
  dataToWrite |= 0x08; //Latch interrupt (Cleared by reading int1_src)
  //dataToWrite |= 0x04; //Pipe 4D detection from 6D recognition to int1?
  myIMU.writeRegister(LIS3DH_CTRL_REG5, dataToWrite);

  //LIS3DH_CTRL_REG3
  //Choose source for pin 1
  dataToWrite = 0;
  //dataToWrite |= 0x80; //Click detect on pin 1
  dataToWrite |= 0x40; //AOI1 event (Generator 1 interrupt on pin 1)
  dataToWrite |= 0x20; //AOI2 event ()
  //dataToWrite |= 0x10; //Data ready
  //dataToWrite |= 0x04; //FIFO watermark
  //dataToWrite |= 0x02; //FIFO overrun
  myIMU.writeRegister(LIS3DH_CTRL_REG3, dataToWrite);
 
  //LIS3DH_CTRL_REG6
  //Choose source for pin 2 and both pin output inversion state
  dataToWrite = 0;
  dataToWrite |= 0x80; //Click int on pin 2
  //dataToWrite |= 0x40; //Generator 1 interrupt on pin 2
  //dataToWrite |= 0x10; //boot status on pin 2
  //dataToWrite |= 0x02; //invert both outputs
  myIMU.writeRegister(LIS3DH_CTRL_REG6, dataToWrite);
  
}
void timer_init() {
  cli();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12); //  TCCR1B Prescaler = clk/256, CTC mode 
  TCCR1C = 0; //  TCCR1C not forcing output compare  
  TCNT1 = 0;  // TCNT1 set timer counter initial value (16 bit value) 

  // OCR1A Interrupts on OCR1A compare  (every 1 second) 
  OCR1A = Int_Timer_Ticks;

  // T1MSK1 Timer/Counter1 Output Compare A Match interrupt is enabled 
  TIMSK1 = (1 << OCIE1A);

}

ISR( TIMER1_COMPA_vect) // 16 bit timer 1 compare 1A match 
{

    // Toggle LED_PORT LED_PIN 
    LED_PORT ^= (1 << LED_PIN);
    //dataGrab = 1;
  // Increment time counter 
  timeCntrQSec++;
  if (timeCntrQSec > 16){
    timeCntrQSec = 0;
    timeCntrSeconds++;
  }
  if (timeCntrSeconds > 60){
    // inc minutes
    timeCntrSeconds = 0;
    timeCntrMinutes++;
    dataGrab = 1;
  }
  if (timeCntrMinutes > 60){
    // inc hours
    timeCntrMinutes = 0;
    timeCntrHours++;
   }


}


// Sensor-reading ethercat slave.

// Currently this slave simply reads an imu-6050 IMU at 1kHz.

// Ethercat buffer content:
// - 0-5: accel X Y Z, raw values.
// - 6-7: temperature
// - 8-13: gyro X Y Z, raw values.
#include "Adafruit_MPU6050.h"
#include "EasyCAT.h"

Adafruit_MPU6050 imu;
EasyCAT easycat;
  
void setup(void) 
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  bool isIMUInit = imu.begin();
  
  if (!isIMUInit) 
    Serial.println("Failed to find MPU6050 chip");

  bool isEthercatInit = easycat.Init();
  if (!isEthercatInit)
    Serial.println("Failed to find EtherCAT shield");
  

  if (isIMUInit && isEthercatInit)
  {
    // Configure IMU: range of measurement, setting highest possible filter value.
    imu.setAccelerometerRange(MPU6050_RANGE_2_G);
    imu.setGyroRange(MPU6050_RANGE_250_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  
    // Configure time-based interrupt: use TC 8, interrupt at 1kHz
    PMC->PMC_PCER1 |= PMC_PCER1_PID35;                      // TC8 power ON : Timer Counter 2 channel 2 IS TC8 - see page 38
  
    TC2->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3  // MCK/8 = 2.625 MHz, clk on rising edge
                                | TC_CMR_WAVE               // Waveform mode
                                | TC_CMR_WAVSEL_UP_RC;      // UP mode with automatic trigger on RC Compare
   
    TC2->TC_CHANNEL[2].TC_RC = 2625;  //<*********************  Frequency = (Mck/2)/TC_RC  Hz -> Here 1kHz.
  
    TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;                // Interrupt on RC compare match
    NVIC_EnableIRQ(TC8_IRQn);
    TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC2 counter and enable
    
    // Read and clear status register
    TC2->TC_CHANNEL[2].TC_SR;
    pinMode(LED_BUILTIN, OUTPUT);
  }
}


void TC8_Handler() 
{
  
  static uint32_t Count;
  // Read and clear status register
  TC2->TC_CHANNEL[2].TC_SR;

  // Read IMU data into ethercat buffer.
  imu.readRawValues(easycat.BufferIn.Byte);
  
  easycat.MainTask();
  
  if (Count++ == 100) 
  {
      Count = 0;
      PIOB->PIO_ODSR ^= PIO_ODSR_P27;                       // Toggle LED every 1 Hz
  }
  /*
  }
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");*/                        
}





void loop() 
{
  // Empty on purpose
}

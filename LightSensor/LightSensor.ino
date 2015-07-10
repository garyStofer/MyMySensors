#include <SPI.h>
#include <MySensor.h>

#define CHILD_ID_LIGHT 0

#define BATTERY_SENSOR_ANALOG_PIN 0
#define LIGHT_SENSOR_ANALOG_PIN 1

unsigned long SLEEP_TIME = 3000; // Sleep time between reads (in milliseconds)

MySensor gw;
MyMessage msg(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
int lastLightLevel;

void setup()
{
  gw.begin();
  pinMode(8, OUTPUT);  
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("MyOwn Sensor1", "1.0");

  // Register all sensors to gateway (they will be created as child devices)
  gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
//  mSetAck(msg,true);
  analogReference( INTERNAL);  // on mega328 this is 1.1V
}

void loop()
{ 
  unsigned int ADC_count;    // for battery reading
  unsigned int lightLevel;   // for other sensor input
  unsigned int batteryPcnt;  // calc the percanet charge of battery --

  // 1M, 470K divider across battery and using internal ADC ref of 1.1V
  // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
  // ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
  // 3.44/1023 = Volts per bit = 0.003363075
  // float batteryV  = ADC_count * 0.003363075;
   
  ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);
  delay(100);
ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);
  // TODO: This is not right -- a battery is empty long before 0V
  batteryPcnt = ADC_count / 10; // for now
  Serial.println(batteryPcnt);

  if (batteryPcnt > 100 )
    batteryPcnt = 100;

  lightLevel = (1023-analogRead(LIGHT_SENSOR_ANALOG_PIN))/10.23;

  Serial.println(lightLevel);
  msg.set(12);


 if (! gw.send(msg, false))    // this returns the state of the HW ack to the next node -- this only works for sinngle hop networks 
 {
   digitalWrite(8, HIGH);   // set the red LED on
 }
 
 gw.sendBatteryLevel(batteryPcnt);


  /*
  for ( int x= 0; x<10; x++)
  {
      gw.process();
      delay(1);
  }
*/

  
  //else
  

    if ( digitalRead(8) )   
    {
      delay(100);
    }


  digitalWrite(8, LOW);   // set the red LED off
  gw.sleep(SLEEP_TIME);
}






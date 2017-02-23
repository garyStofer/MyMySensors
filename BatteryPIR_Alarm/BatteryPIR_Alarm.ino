#include <SPI.h>
#include <EEPROM.h> 
#include <MySensor.h>

// Requieres MySensors v1.4 or newer

/* MySensor Node application for PIR motion sensor running on Mega328P powered by two AAA cells.

	Standby current consumption with PIR module attached is about 55uA.

	This Node will low power sleep until a pin change is detected on the PIR status pin. The Node then reports the number of
	times the PIR has been triggered since it has been powered on. It also reports it's battery status with each report.

-  Mega328P should be setup to run at 8Mhz internal clock for max relibility and minimum power consumption.
-  PIR input is connected to D3 aka PD3, aka INT1
-  Standard Arduino power LED and status LED on pin PB5 (aka SCK) needs to be removed for mimimum power consumption.
-  Input voltage regulator removed, feed B+ to VCC directly.
-  Standard Radio connection, VCC,D9,D10,Mosi,Miso,SCK,GND for nRF24L
-  Voltage divider 1M/470K 1% between B+ and BATTERY_V_DIV_GND_PIN for minimum power consumption.

*/


#define CHILD_1 1

#define BATTERY_SENSOR_ANALOG_PIN 7   // ADC7, Mega328P device pin 22, this is ANALOG pin 7
#define BATTERY_V_DIV_GND_PIN	  7   // PD7, Mega328P device pin 11, this is DIGITAL pin 7 	
#define SWITCH_PIN 7	              // This is the same as the BATTERY_V_DIF_GND_PIN.  battery measure voltage divider serves as the pull-up for the switch
#define LED_PIN	  8	   	      // PB0, Mega328P device pin 12 -- There are two LEDS connected in push pull configuration -- Tristate this pin to turn the LEDS off
#define PIR_SIGNAL_PIN 3               // PB3, device INT1 
#define PIR_INTERRUPT PIR_SIGNAL_PIN-2
#define ADC_BITVALUE (1.1f / 1024)    // ADC using 1.1V internal reference
#define ADC_BATTERY_DIV  (3.125f)     // external voltage divider ratio 1M:470K
#define BATTERY_EMPTY (2.6f)
#define BATTERY_FULL  (3.2f)


MySensor node;


unsigned char nTriggers = 0;

void
PIR_interrupt(void )
{
  // we use the interrupt to wake up from sleep mode only. Nothing needs to be done in here, since the PIR output is the only thing that can trigger this ISR
  nTriggers++;  // used for battery reporting every nth trigger
}

void setup()
{
  unsigned int node_nr;

  Serial.begin(115400);         // lower the BAUD rate if this runs on 8mhz
  pinMode(SWITCH_PIN, INPUT);
  if (digitalRead( SWITCH_PIN )==0 )
  {

  for (int i = 0; i < 512; i++)
      EEPROM.write(i, 0xff);

    Serial.println("Clearing of EEprom complete.");
  }

  node.begin();
  Serial.begin(115400);         // override the Baudrate from node.begin -- lower it if this runs on 8mhz

  node_nr = node.getNodeId();

            Serial.print("PIR Sensor Startup, My Node ID is ");
            Serial.print(node_nr);
            Serial.println(" .");
            pinMode(LED_PIN, OUTPUT);// blip the LED to announce our-selfs
            digitalWrite(LED_PIN, LOW);



            // Send the sketch version information to the gateway and Controller
            node.sendSketchInfo("Battery_PIR_Alarm_INT1", "1.0");

            // Register all sensors to gateway (they will be created as child devices)
            node.present(CHILD_1, S_MOTION);

            pinMode(PIR_SIGNAL_PIN, INPUT);

            // Setup and execute an initial battery voltage reading --
            pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);
            digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement
            analogReference( INTERNAL);  // on mega328 this is 1.1V
            analogRead(BATTERY_SENSOR_ANALOG_PIN);	// This needs to be done once upfront to setup the ADC before the first reading is taken
            pinMode(BATTERY_V_DIV_GND_PIN, INPUT);

            pinMode(LED_PIN, INPUT);    //LED off

            // setting up interrupt
            attachInterrupt(PIR_INTERRUPT, PIR_interrupt, RISING);

}


void loop()
{
  unsigned int ADC_count;    // for battery reading
  int BatteryPcnt;  // calc the percent charge of battery --
  float BatteryV;
  MyMessage msg(CHILD_1, V_TRIPPED);

  pinMode(LED_PIN, INPUT);    //LED off;   // set the red LED indicating failure to communicte with gateway or router

  // The NODE sleeps here until PIR pin goes high
  
  // NOTE: This node sleep relies on modified code in MyHWAtmega238.cpp so that it goes into standby when called with 0 -- only an interrupt brings it back
  node.sleep(0);
 
  // when here the node woke up


  Serial.print("Alarm Tripped ");
  Serial.print(nTriggers);
  Serial.print("-- Sending message\n");
  // Node only reports when Alarm triggers i.e. it does not send an alarm when the PIR signal resets. -- automation controller has to do the reset if needed
  msg.set(1);
  if (!node.send(msg, false))       // this returns the state of the HW ack to the next node -- this only works for single hop networks, second argument is protocol level ACK
  {
    Serial.println("Sending message failed -- no hardware ACK received");
    pinMode(LED_PIN, OUTPUT);    //LED on;   // set the LED indicating failure to communicte with gateway or router
    node.wait(500);              // for cap charge up
    return;
  }

  // setup the voltage divider for the battery measurement
  if (nTriggers % 10 == 0)
  {
    digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement
    pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);     // activate the voltage divider by pulling the low end to GND

    node.wait(500);  // for cap charge up

    ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);

    BatteryV = ADC_count *  ADC_BITVALUE * ADC_BATTERY_DIV;
    BatteryPcnt = 100 * (BatteryV - BATTERY_EMPTY) / (BATTERY_FULL - BATTERY_EMPTY);
    Serial.print( "Battery voltage: ");
    Serial.print(BatteryV);

    if (BatteryPcnt > 100 )
      BatteryPcnt = 100;

    if (BatteryPcnt < 0 )
      BatteryPcnt = 0;

    Serial.print(", Percent : ");

    Serial.println(BatteryPcnt);

    pinMode(BATTERY_V_DIV_GND_PIN, INPUT);      // deactivate the voltage divider by letting it float

    node.sendBatteryLevel(BatteryPcnt);			// send battery percent to controller

  }

}







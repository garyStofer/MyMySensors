#include <EEPROM.h>
#include <MySensor.h>
// NOTE: Uses PinChange interrupt feature on PortC so that sensor gets triggered on rising AND falling edge of PIR signal
// Rising edge sets the alarm state, falling edge clears.  PIR device has trimmer to set the active time
// Requires MySensor v1.5 due to PinChange interrupt flaw in 1.4 --

/* MySensor Node application for PIR motion sensor running on Mega328P powered by two AAA L92 Lithium cells.
	Standby current consumption with PIR module attached is about 55uA.

	This Node will sleep until a pin change is detected on the PIR status pin. The Node then reports true or false depending on the
	state of the PIR signal output, thus setting and clearing the alarm state.
	The node also reports battery status every BATTERY_REPORT_FREQ time.


  -  Mega328P should be setup to run at 8Mhz external XTAL clock for max reliability and minimum power consumption.
  -  PIR input(s) are using pin change interrupt on port PC. Therefore Analog input A0 (PC0,ADC0) is not available
  -  Standard Arduino power LED and status LED on pin PB5 (SCK) needs to be removed for minimum power consumption.
  -  Input voltage regulator removed, feed B+ to VCC directly. Radio
  -  Standard Radio connection, VCC,D9,D10,MOSI,MISO,SCK,GND for nRF24L
  -  Voltage divider 1M/470K 1% between B+ and BATTERY_V_DIV_GND_PIN for minimum power consumption.
  -  Red LED D8 to GND for loss of RF communication indication. -- Only failure to talk to the parent node is indicated (1st hop).
  -  White or Blue LED VCC to D8 for other indication.
  -  Battery life expectancy with two AAA L92 is about 1 year if the sensor doesn't get triggered often.

  -- MUST set Brownout fuse to 1.8V in order to run battery sufficiently down
*/


#define CHILD_1 1

#define BATTERY_SENSOR_ANALOG_PIN 7    // ADC7, Mega328P device pin 22, this is ANALOG pin 7
#define BATTERY_V_DIV_GND_PIN	  7	   // PD7, Mega328P device pin 11, this is DIGITAL pin 7  	

#define SWITCH_PIN BATTERY_V_DIV_GND_PIN // This is the same as the BATTERY_V_DIF_GND_PIN.  battery measure voltage divider serves as the pull-up for the switch
#define LED_PIN		 			    8		// PB0, Mega328P device pin 12
#define PIR_SIGNAL_PIN 		  14   		// PC0, aka ADC0, aka A0, Mega328P device pin 23  -- Do not move this pin -- Pin-Change code below sets up PC0 as input


#define ADC_BITVALUE (1.1f / 1024)    // ADC using 1.1V internal reference
#define ADC_BATTERY_DIV  (3.125f)     // external voltage divider ratio 1M:470K
#define BATTERY_EMPTY (2.6f)
#define BATTERY_FULL  (3.3f)
#define BATTERY_REPORT_FREQ 20        // 24  // report the battery status only every Nth time for lower power consumption



MySensor node;
MyMessage msg(CHILD_1, V_TRIPPED);

// Pin change interrupt to capture the event from the PIR sensor
static unsigned short AlarmInstances = 0;
ISR(PCINT1_vect)
{

  if (digitalRead(PIR_SIGNAL_PIN) )		// count alarm instances to send battery status only every Nth time
    AlarmInstances++;

}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);	// red LED
 //node.begin(NULL,2);       // fixed node ID -- will be stored in EEPROM if empty
  node.begin();               // this gets a node ID automatically if EEPROM is empty
  digitalWrite(LED_PIN, LOW);  // white LED
  Serial.begin(57600);        // lower the BAUD rate if this runs on 8mhz without an x-tal as per fuse settings


  pinMode(SWITCH_PIN, INPUT);
  // Erasing the EEprom of the chip -- Node will loose it's ID number, Gateway and Controller will furnish a new ID number
  if (digitalRead( SWITCH_PIN ) == 0 )
  {
    Serial.print("Erasing EEPROM\n");
    for (int i = 0; i < 512; i++)
      EEPROM.write(i, 0xff);

    Serial.println("Clearing of EEprom complete.");
  }

  Serial.print("PIR Sensor Startup -- Node ID:");
  Serial.println( node.getNodeId() );


  pinMode(BATTERY_V_DIV_GND_PIN, INPUT);
  digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement

  // Send the sketch version information to the Gateway and Controller
  Serial.print("Sending Sketch name\n");
  node.sendSketchInfo("Battery-PIR-MotionPCint", "1.1");

  // Register all sensors to gateway (they will be created as child devices)
  node.present(CHILD_1, S_MOTION);
  analogReference( INTERNAL);  // on mega328 this is 1.1V
  analogRead(BATTERY_SENSOR_ANALOG_PIN);	// This needs to be done once upfront to setup the ADC before the first reading is taken

  // Setting up the pin change interrupt of Port C
  pinMode(PIR_SIGNAL_PIN, INPUT);       // PC0, aka A0, aka digital pin14, aka device pin23
  // digitalWrite(PIR_SIGNAL_PIN, HIGH);   // pull up -- Not needed as PIR sensor already has pull-up
  // enable the pin change interrupt for PC0, aka, A0, aka pin14
  PCMSK1 |= 1 << PCINT8; // enable PCinterupt 8 , aka PC0, aka A0
  PCICR  |= 1 << PCIE1; // enable PCinterupt 1 vector

  pinMode(LED_PIN, INPUT); // leds off
}


void loop()
{
  unsigned int ADC_count;    // for battery reading
  int BatteryPcnt;  // calc the percent charge of battery --
  float BatteryV;

  pinMode(LED_PIN, INPUT);			// LEDS off


  // The NODE sleeps here until pin-change happens -- This requires a 328P as in a PicoPower device 
  node.sleep(0);  // This sleeps forever until a pin change interrupt wakes it up. Can not use timed sleep here.

  // when "HERE" the node woke up because of the pin change interrupt


  if (digitalRead(PIR_SIGNAL_PIN) )	//Rising edge of PIR status signal. i.e. tripped
  {
    Serial.print("PIR set\n");
    msg.set(1);    // The status of the PIR Sensor output
 

    if (!node.send(msg, false))    // this returns the state of the HW ACK to the next node -- this only works for single hop networks, second argument is protocol level ACK
    {
      Serial.print("failed to ack\n");
      digitalWrite(LED_PIN, HIGH);  // set the red LED indicating failure to communicate with parent (gateway or router)
      pinMode(LED_PIN, OUTPUT);		// LEDS on
      node.wait(300);
    }

  }
  else  // This is the end or the alarm pulse, the duration is adjustable by a trimmer on the PIR sensor module.
  {
    Serial.print("PIR cleared\n");
    msg.set(0);    // Set the alarm status back
    node.send(msg, false);

    // send Battery status every Nth time

    if (! ((AlarmInstances - 1) % BATTERY_REPORT_FREQ)) // -1 so that first alarm after reset sends battery status
    {
      Serial.print("Battery Voltage : ");
      // setup the voltage divider for the battery measurement
      digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement
      pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);     // activate the voltage divider by pulling the low end to GND

      pinMode(LED_PIN, OUTPUT);
      digitalWrite(LED_PIN, LOW);	// white LED on for battery load
      node.wait(500);  // for cap charge up

      ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);
      pinMode(LED_PIN, INPUT);		// LEDS off again

      BatteryV = ADC_count *  ADC_BITVALUE * ADC_BATTERY_DIV;
      BatteryPcnt = 100 * (BatteryV - BATTERY_EMPTY) / (BATTERY_FULL - BATTERY_EMPTY);

      Serial.println(BatteryV);

      if (BatteryPcnt > 100 )
        BatteryPcnt = 100;

      if (BatteryPcnt < 0 )
        BatteryPcnt = 0;

      pinMode(BATTERY_V_DIV_GND_PIN, INPUT);      // deactivate the voltage divider by letting it float

      node.sendBatteryLevel(BatteryPcnt);

    }

  }


}







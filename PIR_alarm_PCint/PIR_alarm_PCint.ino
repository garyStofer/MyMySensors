#include <EEPROM.h>
#define DEBUG
#include <MySensor.h>
 
// NOTE: Uses PinChange interrupt feature on PortC so that sensor gets triggered on rising AND falling edge of PIR signal
// Rising edge sets the alarm state, falling edge clears.  PIR device has trimmer to set the active time right side, left  side is sensitivity
// Requires MySensor v1.5 due to PinChange interrupt flaw in 1.4 --

/************************* IMPORTANT ************************************ */
//    Must set Brownout voltage to 1.8V and internal clock 8mhz with 4ms startup delay using Atmel Studio programmer with ICE-MKII after loading Optiboot 32Pin bootloader
//    If Brownout voltage is left at default 2.7V battery will not be fully used 
//    If 8Mhz startup delay is left at 65 ms the bootloader will not connect when device was in sleep mode and the device can only be programmed once 
//    A newer version of the Optiboot loader or modifying the defaults in baords.txt for the optiboot loader would also solve this
// FUSES : 0xFE,0XDE,0xD2
 

 
/* MySensor Node application for PIR motion sensor running on Mega328P powered by two AAA L92 Lithium cells.
	Standby current consumption with PIR module attached is about 55uA.

	This Node will sleep until a pin change is detected on the PIR status pin. The Node then reports true or false depending on the
	state of the PIR signal output, thus setting and clearing the alarm state.
	The node also reports battery status every BATTERY_REPORT_FREQ time.


  -  Mega328P should be setup to run at 8Mhz internal clock for max reliability and minimum power consumption. See above 
  -  PIR input(s) are using pin change interrupt on port PC. Therefore Analog input A0 (PC0,ADC0) is not available
  -  Standard Arduino power LED and status LED on pin PB5 (SCK) needs to be removed for minimum power consumption.
  -  Input voltage regulator removed, feed B+ to VCC directly. Radio
  -  Standard Radio connection, VCC,D9,D10,MOSI,MISO,SCK,GND for nRF24L
  -  Voltage divider 1M/470K 1% between B+ and BATTERY_V_DIV_GND_PIN for minimum power consumption.
  -  Red LED D8 to GND for loss of RF communication indication. -- Only failure to talk to the parent node is indicated (1st hop).
  -  White or Blue LED VCC to D8 for other indication.
  -  Battery life expectancy with two AAA L92 is about 1 year if the sensor doesn't get triggered constantly.

  -- MUST set Brownout fuse to 1.8V in order to run battery sufficiently down, see above.
*/

/*  
 * For library debug messages enable "#define DEBUG" in MyCinfig.h -- Will not work if defined in this source file
*/
#define CHILD_1 1


#define BATTERY_SENSOR_ANALOG_PIN 7  // ADC7, Mega328P device pin 22, this is ANALOG pin 7
#define BATTERY_V_DIV_GND_PIN	  7	   // PD7, Mega328P device pin 11, this is DIGITAL pin 7  	

#define SWITCH_PIN BATTERY_V_DIV_GND_PIN // This is the same as the BATTERY_V_DIF_GND_PIN.  battery measure voltage divider serves as the pull-up for the switch
#define LED_PIN		 			    8		// PB0, Mega328P device pin 12
#define PIR_SIGNAL_PIN 		  14   		// PC0, aka ADC0, aka A0, Mega328P device pin 23  -- Do not move this pin -- Pin-Change code below sets up PC0 as input


#define ADC_BITVALUE (1.1f / 1024)    // ADC using 1.1V internal reference
#define ADC_BATTERY_DIV  (3.125f)     // external voltage divider ratio 1M:470K
#define BATTERY_EMPTY (2.5f)
#define BATTERY_FULL  (3.0f)
#define BATTERY_REPORT_FREQ 24*2       // report the battery status only every 24th time for lower power consumption



MySensor node;
MyMessage msg(CHILD_1, V_TRIPPED);

// Pin change interrupt to capture the event from the PIR sensor
static  short AlarmInstances = -1;
ISR(PCINT1_vect)
{

		// count alarm instances to send battery status only every Nth time
    AlarmInstances++;

}

void setup()
{
  // The constructor of class MySensor sets the baudrate  as defined in MyConfig.h (115kbd by default) this however doesnt work for a node 
  // that runs on 3V and its internal 8MHZ oscillator
  // Modify MyConfig.h to use 57kbd to make it work at 3.3V / 8mhz
  
  Serial.begin(57600);       // slow down because 8Mhz --  
  Serial.println("Device Startup.");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);	// red LED
  pinMode(SWITCH_PIN, INPUT);
  
  // Erasing the EEprom of the chip -- Node will loose it's ID number, Gateway and Controller will furnish a new ID number
  if (digitalRead( SWITCH_PIN ) == 0 )
  {
    Serial.print("Erasing EEPROM\n");
    for (int i = 0; i < 512; i++)
      EEPROM.write(i, 0xff);

    Serial.println("Clearing of EEprom complete.");
  }

  Serial.println("Node Startup");
  // must have node.begin AFTER the EEProm erase, otherwise it will always use the old ID 
  //node.begin(NULL,3);         // fixed node ID -- will be stored in EEPROM if executed
  //node.begin(NULL,6,false,0); // No receive function, myNodeId =n, NoRepeater,myParentId =0  
  node.begin();               // this gets a node ID from the controller if EEPROM is empty or previous parent is dead    

  // Serial.begin(57600);       // slow down in case node begin reset the rate 
      
  digitalWrite(LED_PIN, LOW);  // white LED

  pinMode(BATTERY_V_DIV_GND_PIN, INPUT);
  digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement

  // Send the sketch version information to the Gateway and Controller
  char *sketch = "Battery-PIR-MotionPCint";
  Serial.print("Sending Sketch name ");
  Serial.println(sketch);
  
  node.sendSketchInfo(sketch, "1.4");
  // it seems that on the very first time after an EEProm reset the node number is not set yet on the first write call and so the setting of the script info gets lost
  // sending it a second time mitigaqtes that issue 
  node.sendSketchInfo(sketch, "1.4");
  Serial.print("\nPIR Sensor Startup -- Node ID:");
  Serial.println( node.getNodeId() );

  // Register all sensors to gateway (they will be created as child devices)
  node.present(CHILD_1, S_MOTION,false,"PIR_SENS");   // gives the sensor a meaningfull name so that domotics can display something upon discovery // 3rd arg==true is ackrequest
  analogReference( INTERNAL);  // on mega328 this is 1.1V
  analogRead(BATTERY_SENSOR_ANALOG_PIN);	// This needs to be done once upfront to setup the ADC before the first reading is taken

  // Setting up the pin change interrupt of Port C
  pinMode(PIR_SIGNAL_PIN, INPUT);       // PC0, aka A0, aka digital pin14, aka device pin23
  digitalWrite(PIR_SIGNAL_PIN, LOW);   // pull DOWN
  // enable the pin change interrupt for PC0, aka, A0, aka pin14
  PCMSK1 |= 1 << PCINT8; // enable PCinterupt 8 , aka PC0, aka A0
  PCICR  |= 1 << PCIE1; // enable PCinterupt 1 vector

  pinMode(LED_PIN, INPUT); // leds off
}


void loop()
{
  unsigned int ADC_count;    // for battery reading
  static int BatteryPcnt;  // calc the percent charge of battery --
  float BatteryV;

  pinMode(LED_PIN, INPUT);			// LEDS off


  // The NODE sleeps here until pin-change happens -- This requires a 328P as in a PicoPower device 
  node.sleep(0);  // This sleeps forever until a pin change interrupt wakes it up. Can not use timed sleep here.

  // when "HERE" the node woke up because of the pin change interrupt


  if (digitalRead(PIR_SIGNAL_PIN) )	//Rising edge of PIR status signal. i.e. tripped
  {
    Serial.print("PIR Set\n");
    msg.set(1);    // The status of the PIR Sensor output
  }
  else
  {
    Serial.print("PIR Clear\n");
    msg.set(0);    // The status of the PIR Sensor output
  }


  if (!node.send(msg, false))    // this returns the state of the HW ACK to the next node (1st hop) -- this only works for single hop networks,
                                 // second argument is protocol level ACK, but that requires that the message receive function is implemented and the recived message is checked.
  {
    Serial.print("failed to HW-ack\n");
    digitalWrite(LED_PIN, HIGH);  // set the red LED indicating failure to communicate with parent (gateway or router)
    pinMode(LED_PIN, OUTPUT);    // LEDS on
    node.wait(300);              // so I can see it
    return;
  }

 
  // send Battery status every Nth time
  if (AlarmInstances % BATTERY_REPORT_FREQ==0 ) // 
  {
    Serial.print("Battery Percent: ");
               
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

    if (BatteryPcnt > 100 )
      BatteryPcnt = 100;

    if (BatteryPcnt < 0 )
      BatteryPcnt = 0;
    Serial.println(BatteryPcnt);
    
    pinMode(BATTERY_V_DIV_GND_PIN, INPUT);      // deactivate the voltage divider by letting it float

    node.sendBatteryLevel(BatteryPcnt);
  }
}







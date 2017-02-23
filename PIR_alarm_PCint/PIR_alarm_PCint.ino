#include <SPI.h>
#include <MySensor.h>
// NOTE: Uses PinChange interrupt feature on PortC for potentially multiple PIR sensors.
// Requires MySensor v1.5 due to PinChange interrupt flaw in 1.4 --

/* MySensor Node application for PIR motion sensor running on Mega328P powered by two AAA L92 Lithium cells.
	Standby current consumption with PIR module attached is about 55uA.

	This Node will sleep until a pin change is detected on the PIR status pin. The Node then reports true or false depending on the
	state of the PIR signal output.
	The node also reports battery status every BATTERY_REPORT_FREQ time.


-  Mega328P should be setup to run at 8Mhz external xtal clock for max reliability and minimum power consumption.
-  PIR input(s) are using pin change interrupt on port PC. Therefore Analog input A0 (PC0,ADC0) is not available
-  Standard Arduino power LED and status LED on pin PB5 (SCK) needs to be removed for minimum power consumption.
-  Input voltage regulator removed, feed B+ to VCC directly. Radio
-  Standard Radio connection, VCC,D9,D10,Mosi,Miso,SCK,GND for nRF24L
-  Voltage divider 1M/470K 1% between B+ and BATTERY_V_DIV_GND_PIN for minimum power consumption.
-  Red LED added to Pin D8  for loss of rf communication indication. -- Only failure to talk to the parent node is indicated.
-  Optional LED for PIR trip indication.
-  Battery life expectancy with two AAA L92 is about 1 year if the sensor doesn't get triggered often.

-- MUST set Browout fuse to 1.8V in order to run battery sufficently down 
*/


#define CHILD_1 1

#define BATTERY_SENSOR_ANALOG_PIN 7    // ADC7, Mega328P device pin 22, this is ANALOG pin 7
#define BATTERY_V_DIV_GND_PIN	  7	   // PD7, Mega328P device pin 11, this is DIGITAL pin 7  	
#define LED_1_PIN		  8	   // PB0, Mega328P device pin 12
#define PIR_SIGNAL_PIN 		  14   // PC0, aka ADC0, aka A0, Mega328P device pin 23  -- Do not move this pin -- Pin-Change code below sets up PC0 as input

#define ADC_BITVALUE (1.1f / 1024)    // ADC using 1.1V internal reference
#define ADC_BATTERY_DIV  (3.125f)     // external voltage divider ratio 1M:470K
#define BATTERY_EMPTY (2.6f)
#define BATTERY_FULL  (3.3f)
#define BATTERY_REPORT_FREQ 2        // 24  // report the battery status only every Nth time for lower power consumption


MySensor node;
MyMessage msg(CHILD_1, V_TRIPPED);

// Pin change interrupt to capture the event from the PIR sensor
static unsigned short AlarmInstances =0;
ISR(PCINT1_vect)
{

    if (digitalRead(PIR_SIGNAL_PIN) )		// count alarm instances to send battery status only every Nth time
        AlarmInstances++;

}

void setup()
{
    pinMode(LED_1_PIN,OUTPUT);
    digitalWrite(LED_1_PIN, HIGH);
    //node.begin(NULL,1);             // fixed node ID -- will be stored in EEPROM if empty 
    node.begin();                     // this gets a node ID automatically if EEPROM is empty  
    Serial.begin(57600);         // lower the BAUD rate if this runs on 8mhz without an x-tal as per fuse settings 

    Serial.print("PIR Sensor Startup -- Node ID:");
    Serial.println( node.getNodeId() );
    digitalWrite(LED_1_PIN, LOW); 

    pinMode(BATTERY_V_DIV_GND_PIN,INPUT);
    digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement

    // Send the sketch version information to the Gateway and Controller
    node.sendSketchInfo("PIR-MotionSensor", "1.0");

    // Register all sensors to gateway (they will be created as child devices)
    node.present(CHILD_1, S_MOTION);
    analogReference( INTERNAL);  // on mega328 this is 1.1V
    analogRead(BATTERY_SENSOR_ANALOG_PIN);	// This needs to be done once upfront to setup the ADC before the first reading is taken

    // Setting up the pin change interrupt of Port C
    pinMode(PIR_SIGNAL_PIN, INPUT);       // PC0, aka A0, aka digital pin14, aka device pin23
    //digitalWrite(PIR_SIGNAL_PIN, HIGH);   // pull up -- Not needed as PIR sensor already has pull-up
    // enable the pin change interrupt for PC0, aka, A0, aka pin14
    PCMSK1 |= 1 << PCINT8; // enable PCinterupt 8 , aka PC0, aka A0
    PCICR  |= 1 << PCIE1; // enable PCinterupt 1 vector
}


void loop()
{
    unsigned int ADC_count;    // for battery reading
    int BatteryPcnt;  // calc the percent charge of battery --
    float BatteryV;

    digitalWrite(LED_1_PIN, LOW);   // set the red LED off


    // The NODE sleeps here until pin-change happens
    node.sleep(0);  // This sleeps forever until a pin change interrupt wakes it up. Can not use timed sleep here.

    // when "HERE" the node woke up because of the pin change interrupt

    if (digitalRead(PIR_SIGNAL_PIN) )	//Rising edge of PIR status signal. i.e. tripped
    {
        // digitalWrite(LED_2_PIN, HIGH);   indicate PIR detect -- need second LED for this
        msg.set(1);    // The status of the PIR Sensor output

        if (!node.send(msg, false))    // this returns the state of the HW ACK to the next node -- this only works for single hop networks, second argument is protocol level ACK
        {
            digitalWrite(LED_1_PIN, HIGH);   // set the red LED indicating failure to communicate with parent (gateway or router)
            node.wait(500);
        }

    }
    else  // This is the end or the alarm pulse, duration is adjustable by a trimmer on the PIR sensor module.
    {
        msg.set(0);    // Set the alarm status back
        node.send(msg, false);

        // send Battery status every Nth time

        if (! (AlarmInstances % BATTERY_REPORT_FREQ))
        {
            // setup the voltage divider for the battery measurement
            digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement
            pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);     // activate the voltage divider by pulling the low end to GND

            node.wait(500);  // for cap charge up

            ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);

            BatteryV = ADC_count *  ADC_BITVALUE* ADC_BATTERY_DIV;
            BatteryPcnt = 100*(BatteryV-BATTERY_EMPTY) / (BATTERY_FULL-BATTERY_EMPTY);
            Serial.print("Battery Voltage : ");
            Serial.println(BatteryV);

            if (BatteryPcnt > 100 )
                BatteryPcnt = 100;

            if (BatteryPcnt < 0 )
                BatteryPcnt =0;

            pinMode(BATTERY_V_DIV_GND_PIN, INPUT);      // deactivate the voltage divider by letting it float

            node.sendBatteryLevel(BatteryPcnt);

        }

    }


}







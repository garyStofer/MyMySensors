#include <SPI.h>
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
-  Red LED added to Pin D8  for loss of rf communication indication. -- Only failure to talk to the parent node is indicated.
-  Optional LED for PIR tripp indication.
*/


#define CHILD_1 1

#define BATTERY_SENSOR_ANALOG_PIN 0    // ADC0, device pin 23
#define BATTERY_V_DIV_GND_PIN	  7	   // PD7, device pin 11 	
#define LED_1_PIN		  8	   // PB0, device pin 12
#define PIR_SIGNAL_PIN 		  3	   // PB3, device INT1 

#define ADC_BITVALUE (1.1f / 1024)    // ADC using 1.1V internal reference
#define ADC_BATTERY_DIV  (3.125f)     // external voltage divider ratio 1M:470K
#define BATTERY_EMPTY (2.6f)
#define BATTERY_FULL  (3.2f)


MySensor node;
MyMessage msg(CHILD_1, V_TRIPPED);
unsigned short AlarmInstances =0;

void setup()
{
    node.begin();
   // Serial.begin(38400);         // lower the BAUD rate if this runs on 8mhz without an xtal per fuse settings
    Serial.println("PIR Sensor Startup");

    pinMode(LED_1_PIN,OUTPUT);
    //pinMode(LED_2_PIN,OUTPUT);
    pinMode(BATTERY_V_DIV_GND_PIN,INPUT);
    digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement

    // Send the sketch version information to the gateway and Controller
    node.sendSketchInfo("PIR-MotionSensor", "1.0");

    // Register all sensors to gateway (they will be created as child devices)
    node.present(CHILD_1, S_MOTION);
    analogReference( INTERNAL);  // on mega328 this is 1.1V
    analogRead(BATTERY_SENSOR_ANALOG_PIN);	// This needs to be done once upfront to setup the ADC before the first reading is taken

    pinMode(PIR_SIGNAL_PIN, INPUT);

}
static bool tr = false;
void loop()
{
    unsigned int ADC_count;    // for battery reading
    int BatteryPcnt;  // calc the percent charge of battery --
    float BatteryV;

    digitalWrite(LED_1_PIN, LOW);   // set the red LED off
    //digitalWrite(LED_2_PIN, LOW);  // set the grn LED off

    // The NODE sleeps here until pin-change happens
    node.sleep(1,RISING,0);
    // when here the node woke up because of the pin change interrupt

    if (digitalRead(PIR_SIGNAL_PIN) )	// test is somewhat redundant since the node.sleep above already is set for RISING edge.
    {
        // setup the voltage divider for the battery measurement
        pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);     // activate the voltage divider by pulling the low end to GND

        // digitalWrite(LED_2_PIN, HIGH);   indicate PIR detect -- need second LED for this
     //   msg.set(++AlarmInstances);    // The status of the PIR Sensor output
	 tr = !tr;
	 msg.set(tr);
	

        if (!node.send(msg, false))    // this returns the state of the HW ack to the next node -- this only works for single hop networks, second argument is protocol level ACK
        {
            digitalWrite(LED_1_PIN, HIGH);   // set the red LED indicating failure to communicte with gateway or router
        }

        ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);

        //Serial.println(ADC_count);

        BatteryV = ADC_count *  ADC_BITVALUE* ADC_BATTERY_DIV;
        BatteryPcnt = 100*(BatteryV-BATTERY_EMPTY) / (BATTERY_FULL-BATTERY_EMPTY);
        //Serial.println(BatteryV);

        if (BatteryPcnt > 100 )
            BatteryPcnt = 100;

        if (BatteryPcnt < 0 )
            BatteryPcnt =0;

        //Serial.println(BatteryPcnt);

        pinMode(BATTERY_V_DIV_GND_PIN, INPUT);      // deactivate the voltage divider by letting it float

       // node.sendBatteryLevel(BatteryPcnt);

    }


}







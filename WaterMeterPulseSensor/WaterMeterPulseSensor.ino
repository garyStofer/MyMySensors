/* MYsensor Application for a flow meter / totalizer Sensor node.
 *
 * This program uses an paddle wheel type of flow sensor that provides a number of pulses related to flow. The software counts and sums
 * the pulses to report the current meter stand (volume) and the current flow rate on a 15 second interval. The node sleeps during times of
 * no flow activity and is being woken up when the digital input from the flow sensors signal attached to one of the HW-Interrupt pins of
 * the Mega328 sees a raising edge.
 *
 * During start-up the node requests the last meter stand from the controller (domoticz) so it knows where to continue. This meter stand is
 * sent to the controller via a separate user var (V_VAR1) and is stored by the controller. This is somewhat of a kludge but necessary
 * because the domoticz controller doesn't have a way of reporting back the last meter stand via its volume entry for some reason.
 *
 * Upon transition to sleep mode the node therefore sends the current meter stand via V_VAR1 to the controller as well as the current
 * battery level.
 *
 * To make this a true battery operated sensor the flow meter attached must consume nearly no current while there is now flow. A standard
 * Hall effect sensor consumes between 2 and 10 ma and is not suitable for this purpose.  There are low duty cycle hall effect sensors available
 * that consume in the single digits of micro amps, but by them selfs they are also not suited because of the low duty-cycle and the resulting
 * maximum frequency they can resolve. Combining the two types of hall sensors will provide a solution by which the low current sensor provides
 * the start-up event and the regular sensor then is used to actually count the pulses wile it is kept un-powered during periods of no activity.
 *
 * An other viable solution is to use a passive reed-switch in the flow sensor and have a high impedance pull-up on the switch, so that should the
 * flow sensor come to rest while the reed is closed there are still only mere micro amps flowing.
 *
 * This code below is using a HW-interrupt with an interrupt service routine attached to the raising edge transition. This limits the sensor node
 * to a single flow-meter should the dual hall sensor implementation be chosen, or to two if a reed-switch is used in the sensors. The Mega328
 * only has 2 HW interrupts, however every digital pin can also trigger a (single) interrupt pin pin state change. It is possible to use this
 * Pin-change interrupt to facilitate more than two flow sensors on a node.

*/

#include <SPI.h>
#include <MySensor.h>
/* Stuff to configure the battery level measurements */
#define BATTERY_SENSOR_ANALOG_PIN 7    // ADC7, Mega328P device pin 22, this is ANALOG pin 7
#define BATTERY_V_DIV_GND_PIN	  7	   // PD7, Mega328P device pin 11, this is DIGITAL pin 7  

#define ADC_BITVALUE (1.1f / 1024)    // ADC using 1.1V internal reference
#define ADC_BATTERY_DIV  (3.125f)     // external voltage divider ratio 1M:470K
#define BATTERY_EMPTY (2.6f)
#define BATTERY_FULL  (3.3f)


#define SWITCH_PIN 7	// This is the same as the BATTERY_V_DIF_GND_PIN.  battery measure voltage divider serves as the pull-up for the switch
#define LED_PIN	  8	   	// PB0, Mega328P device pin 12 -- There are two LEDS connected in push pull configuration -- Tristate this pin to turn the LEDS off

#define WAKEUP_INPUT1_PIN 14   // PC0, aka ADC0, aka A0, Mega328P device pin 23  -- Do not move this pin -- Pin-Change code below sets up PC0 as input
// This pin is connected to the low duty cycle (standby) Hall sensor and is only needed to wake cpu on the onset of flow

#define WAKEUP_INPUT2_PIN 15   // PC1, aka, ADC1, aka A1, Mega 328p device pin 24 see above

#define HALL_SENSOR1_PIN 2                         // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define SENSOR1_INTERRUPT HALL_SENSOR1_PIN-2        // Usually the interrupt = pin -2 (on uno/nano anyway)
#define HALL_SENSOR2_PIN 3                         // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define SENSOR2_INTERRUPT HALL_SENSOR2_PIN-2
#define HALL_SENSOR1_PWR_PIN   4                        // This pin is providing power to the main Hall sensor -- It is switched off during sleep to conserve power
#define HALL_SENSOR2_PWR_PIN   5


#define PULSE_FACTOR 4740                 // Number of pulses  per 1000 liters  -- as measured 
#define PULSE_DIV   100                   // Using a divider to lower the PulseCount values reported to GW so that the count value doesn't overflow that quickly 

#define CHILD_1 1                         // Id of the sensor child
#define CHILD_2 2                         // Id of the sensor child
#define ONE_MINUTE 	60000Ul           // one minute in milli seconds
#define REPORT_PERIODE  10000Ul           // Minimum time between send (in milliseconds). We don't want to spam the gateway.

MySensor node;                            // the class that embodies the network node

MyMessage flowMsg1(CHILD_1,V_FLOW);
MyMessage volumeMsg1(CHILD_1,V_VOLUME);
MyMessage lastCounterMsg1(CHILD_1,V_VAR1);

MyMessage flowMsg2(CHILD_2,V_FLOW);
MyMessage volumeMsg2(CHILD_2,V_VOLUME);
MyMessage lastCounterMsg2(CHILD_2,V_VAR1);

double ppl = ((double)PULSE_FACTOR)/1000;     // Pulses per liter
volatile unsigned long PulseCount1 = 0;        // Current pulse count
unsigned long PPulseCount1 = 0;                // Prior Pulse Count (from prior reporting cycle)
volatile unsigned long PulseCount2 = 0;        // Current pulse count
unsigned long PPulseCount2 = 0;                // Prior Pulse Count (from prior reporting cycle)
unsigned char ReqPCforChild =0;
unsigned long LastSentTime =0;
unsigned long CurrentTime;

// Callback from node.process() when it has a data indication
void incomingMessage(const MyMessage &message)
{
    if (message.type==V_VAR1)
    {
        if (  ReqPCforChild == CHILD_1)
        {
            PPulseCount1 = PulseCount1 = message.getULong();
            Serial.print("Node received last pulsecount1 from GW:");
            Serial.println(PulseCount1);
        }
        else if (  ReqPCforChild == CHILD_2)
        {
            PPulseCount2= PulseCount2= message.getULong();
            Serial.print("Node received last pulsecount2 from GW:");
            Serial.println(PulseCount2);
        }

        ReqPCforChild = 0;

    }
}

// This is the ISR -- for a single sensor, simply count the pulses on rising edges
// It also divides the pulse count so that we record a lower number for the meter-stand in the GW -- Overflow of meter-stand in SQL
void
onPulse1()
{
    static unsigned char cnt_div = 0;

    if (cnt_div++ >= PULSE_DIV )
    {
        PulseCount1++;
        cnt_div =0;
    }
}

// This is the ISR -- for a single sensor, simply count the pulses on rising edges
// It also divides the pulse count so that we record a lower number for the meter-stand in the GW -- Overflow of meter-stand in SQL
void
onPulse2()
{
    static unsigned char cnt_div = 0;

    if (cnt_div++ >= PULSE_DIV )
    {
        PulseCount2++;
        cnt_div =0;
    }
}

// This is the ISR for the standby hall effect sensor that detects the onset of water flow and the  turns on the power for the main hall effect sensor.

ISR(PCINT1_vect)		// Pin change interrupt
{
    // turn power on to main hall effect sensor
    pinMode(HALL_SENSOR1_PWR_PIN, OUTPUT);
    digitalWrite(HALL_SENSOR1_PWR_PIN, HIGH);
    pinMode(HALL_SENSOR2_PWR_PIN, OUTPUT);
    digitalWrite(HALL_SENSOR2_PWR_PIN, HIGH);
}

void
setup()
{
    double volume ;

    // blip the LED to announce our-selfs
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    node.begin(incomingMessage);

    pinMode(HALL_SENSOR1_PIN, INPUT);
    pinMode(HALL_SENSOR2_PIN, INPUT);
    // digitalWrite(HALL_SENSOR2_PIN, HIGH);	// don't pull up --works against Hall effect sensor being turned off

    // setup the power feed pins for HALL 1 & 2
    pinMode(HALL_SENSOR1_PWR_PIN, OUTPUT);
    digitalWrite(HALL_SENSOR1_PWR_PIN, HIGH);
    pinMode(HALL_SENSOR2_PWR_PIN, OUTPUT);
    digitalWrite(HALL_SENSOR2_PWR_PIN, HIGH);

    // Send the sketch version information to the gateway and Controller
    node.sendSketchInfo("Water Meter", "1.0");
    Serial.print( "Water meter ");
    Serial.println( LIBRARY_VERSION);

    // Register this device as Water-flow sensor
    node.present(CHILD_1, S_WATER);
    node.present(CHILD_2, S_WATER);

	/* reset the meter stand when holding the switch during power on */
    if ( digitalRead( SWITCH_PIN ) == 0)
    {
        node.send(lastCounterMsg1.set(PulseCount1));  //
        node.send(lastCounterMsg2.set(PulseCount2));  //
    }

    // Setup and execute an initial battery voltage reading --
    pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);

    digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement

    analogReference( INTERNAL);  // on mega328 this is 1.1V

    analogRead(BATTERY_SENSOR_ANALOG_PIN);	// This needs to be done once upfront to setup the ADC before the first reading is taken

    pinMode(BATTERY_V_DIV_GND_PIN, INPUT);



    // Fetch last known pulse count value from gw
    Serial.println( "Node sent a REQ for last known pulse count ");

    // If the node can not communicate with the Gateway it can't proceed to start reporting flow since it didn't get the starting count.

    ReqPCforChild = CHILD_1;

    while (ReqPCforChild == CHILD_1)
    {
        //Last Pulsecount not yet received from controller, request it again
        node.request(ReqPCforChild, V_VAR1);
        node.wait(50);
        node.process();
        if ( !ReqPCforChild )
          break;
        node.wait(50);
        node.process();
        Serial.print("1");
    }

    ReqPCforChild = CHILD_2;
    node.wait(1000);
    while (ReqPCforChild == CHILD_2)
    {
        //Last Pulsecount not yet received from controller, request it again
        node.request(ReqPCforChild, V_VAR1);
        node.wait(50);
        node.process();
        if ( !ReqPCforChild )
          break;
        node.wait(50);
        node.process();
        Serial.print("2");
    }

    // send back current volume from received PulseCount count to have controller recognize it as a operational node
    volume = ((double)PulseCount1/((double)PULSE_FACTOR));

    node.send(volumeMsg1.set(volume, 3));               // Send volume value back to gw

    Serial.print("Node returns volume1 in m3 :");

    Serial.println(volume, 3);

    volume = ((double)PulseCount2/((double)PULSE_FACTOR));

    node.send(volumeMsg2.set(volume, 3));               // Send volume value back to gw

    Serial.print("Node returns volume2 in m3 :");

    Serial.println(volume, 3);
    pinMode(LED_PIN, INPUT);		// LED OFF
    LastSentTime = millis();

    // setting up interrupts
    attachInterrupt(SENSOR1_INTERRUPT, onPulse1, RISING);

    attachInterrupt(SENSOR2_INTERRUPT, onPulse2, RISING);

    // the pin change interrupt of Port C
    pinMode(WAKEUP_INPUT1_PIN, INPUT);       // PC0, aka A0, aka digital pin14, aka device pin23

    digitalWrite(WAKEUP_INPUT1_PIN, HIGH);   // weak pull up NOT needed with standby hall effect sensor(s)

    pinMode(WAKEUP_INPUT2_PIN, INPUT);       // PC0, aka A0, aka digital pin14, aka device pin23

    digitalWrite(WAKEUP_INPUT2_PIN, HIGH);

    // enable the pin change interrupt for PC0, aka, A0, aka pin14
    PCMSK1 |= 1 << PCINT8; // enable PCinterupt 8 , aka PC0, aka A0

    PCMSK1 |= 1 << PCINT9; // enable PCinterupt 8 , aka PC0, aka A0

    PCICR  |= 1 << PCIE1; // enable PCinterupt 1 vector
}


void
loop()
{
    /* Note that there is no further node.process() call in the loop.
    /* The GW never sends an other message after the initial request for pulse count.
    */
    CurrentTime = millis();

    // Only send values at a specified interval in order not to flood the network with too many messages

    if (CurrentTime - LastSentTime > REPORT_PERIODE)    // every 'n' seconds  --
    {

        if ( PulseCount1 == PPulseCount1 && PulseCount2 == PPulseCount2)		// nothing has been flowing since last time
        {
            if (CurrentTime - LastSentTime >= (ONE_MINUTE/2))    // no action for one minute -- go to sleep until interrupt wakes us up again
            {
                Serial.println( "About to go to sleep ");
                // Turn off the Main Hall sensor power
                digitalWrite(HALL_SENSOR1_PWR_PIN, LOW);
                digitalWrite(HALL_SENSOR2_PWR_PIN, LOW);

                /* before we go to sleep send the battery level and the last meter count */
                int BatteryPcnt;  // calc the percent charge of battery --
                float BatteryV;
                unsigned int ADC_count;

                // setup the voltage divider for the battery measurement
                digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement
                pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);     // activate the voltage divider by pulling the low end to GND

                pinMode(LED_PIN, OUTPUT);
                digitalWrite(LED_PIN, HIGH);

                node.wait(500);  // for cap charge up

                ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);

                BatteryV = ADC_count *  ADC_BITVALUE* ADC_BATTERY_DIV;
                BatteryPcnt = 100*(BatteryV-BATTERY_EMPTY) / (BATTERY_FULL-BATTERY_EMPTY);
                Serial.print( "Battery voltage: ");
                Serial.print(BatteryV);

                pinMode(LED_PIN, INPUT);

                if (BatteryPcnt > 100 )
                    BatteryPcnt = 100;

                if (BatteryPcnt < 0 )
                    BatteryPcnt =0;

                Serial.print(", Percent : ");

                Serial.println(BatteryPcnt);

                pinMode(BATTERY_V_DIV_GND_PIN, INPUT);      // deactivate the voltage divider by letting it float

                node.sendBatteryLevel(BatteryPcnt);			// send battery percent to controller

                node.send(flowMsg1.set(0.0, 2));             // send 0 flow-rate to controller

                node.send(flowMsg2.set(0.0, 2));             // send 0 flow-rate to controller

                Serial.print("Sending pulse count1:");

                Serial.println(PulseCount1);

                node.send(lastCounterMsg1.set(PulseCount1));  // Send raw pulsecount so that we can initialize the total count from the controller if we go off power

                Serial.print("Sending pulse count2:");

                Serial.println(PulseCount2);

                node.send(lastCounterMsg2.set(PulseCount2));  // ""
				Serial.println( "Sleeping now!");

                node.sleep(0);

                // when we are here we just woke up from eternal slumber -- time to start taking readings again
                LastSentTime = millis();          // so that we don't fall asleep again

                Serial.println( "Woken up from sleep");

                return;  // this will bounce us back to the top of loop immediately.
            }
        }
        else    // we are taking readings now -- flow is present
        {
            float flow_rate;
            double volume_m3;

            Serial.println("Updating Controller :");
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, LOW);

            // for sensor flow sensor 1
            flow_rate = (PulseCount1-PPulseCount1)/ ppl  * ONE_MINUTE/REPORT_PERIODE;

            Serial.print("Rate1 L/min:");
            Serial.println(flow_rate);
            node.send(flowMsg1.set(flow_rate, 2));                   // Send flow rate to gw

            volume_m3 = ((double)PulseCount1/((double)PULSE_FACTOR));

            Serial.print("Total volume1 in m3 :");
            Serial.println(volume_m3, 3);

            node.send(volumeMsg1.set(volume_m3, 3));               // Send volume value to gw

            PPulseCount1 = PulseCount1;                             // keep the just reported pulse count in "Prior" for next reporting cycle

            // for sensor flow sensor 2
            flow_rate = (PulseCount2-PPulseCount2)/ ppl  * ONE_MINUTE/REPORT_PERIODE;

            Serial.print("Rate2 L/min:");
            Serial.println(flow_rate);
            node.send(flowMsg2.set(flow_rate, 2));                   // Send flow rate to gw

            volume_m3 = ((double)PulseCount2/((double)PULSE_FACTOR));

            Serial.print("Total volume2 in m3 :");
            Serial.println(volume_m3, 3);

            node.send(volumeMsg2.set(volume_m3, 3));               // Send volume value to gw

            PPulseCount2 = PulseCount2;                             // keep the just reported pulse count in "Prior" for next reporting cycle


            pinMode(LED_PIN, INPUT);							// turn the LED off
            LastSentTime = CurrentTime;
        }
    }


}






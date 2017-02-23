/* 
 *   Sensor node that acts as a dual light or appliance switch 
 *   Push button on node controls relay #1 only. Push button control works locally since feedback via controller is not always reliable. 
 *     
 */

#include <EEPROM.h>
#include <MySensor.h>

#define RELAY_PIN1  16  // aka A2, PC2, ADC2 -- chip pin 25
#define RELAY_PIN2  17  // aka A3, PC3, ADC3 -- chip pin 26
#define BUTTON_PIN  7  // Arduino Digital I/O pin number for button 
#define CHILD_ID1 1   // Id of the first sensor child
#define CHILD_ID2 2  // Id of the second sensor child
#define LED_PIN   8     // There are 2 LEDS on this pin -- RED from D8 to GND, and green from VCC to D8 -- Tri-state for leds off


MySensor node;
MyMessage msg1(CHILD_ID1, V_LIGHT);   // also holds the local relay state for R1 acted upon by the button
MyMessage msg2(CHILD_ID2, V_LIGHT);   


void ChangeSwitch (const MyMessage& msg)
{
   // Change relay states
   bool s= msg.getBool();   
   Serial.print( "Switch");
  
  if (msg.sensor == CHILD_ID1)
  {
    msg1.set(s);    // record the state in local msg1 structure so that the button toggle can be done locally
    digitalWrite(RELAY_PIN1, s );
    digitalWrite(LED_PIN, s); 
    Serial.print( "1: ");
    Serial.println( s);
  }
  else if (msg.sensor == CHILD_ID2)
  {
    msg2.set(s);
    digitalWrite(RELAY_PIN2, s );
    Serial.print( "2: ");
    Serial.println( s);
  }

}



void    // This function is called from the radio stack when data is reveived 
dataIndication(const MyMessage &message)
{
  // We only expect one type of message from controller. But we better check anyway.
  // Serial.println("data indication\n");
 Serial.println("data indication"); 

  if (message.isAck())
  {
    Serial.println("This is an ack from gateway");
  }
  else if (message.type == V_LIGHT)
  {

    ChangeSwitch(message);
    
    // Store state in EE-prom not such a good idea -- wears out the EEProm eventually
    // node.saveState(CHILD_ID1, state);

   // Write some debug info
   // Serial.print("Incoming change for Relay:");
   // Serial.print(message.sensor);
   // Serial.print(", New status: ");
   // Serial.println(state);
  }
  else 
    Serial.println("Unknown message received");
  
}

void    // arduino IDE initial function
setup()
{
// to ertase EEPROM holding device ID in order to create a uncommissioned device 
//    for (int i = 0; i < 512; i++)
//      EEPROM.write(i, 0xff);

  node.begin(dataIndication, AUTO, true);   // this is a repeater node as well since it's on main power anyway
 //  node.begin(dataIndication,7);   // non repeater mode
 
  Serial.print( "Relay with Switch ");
  Serial.println( LIBRARY_VERSION);
  
  // Send the sketch version information to the gateway and Controller
  node.sendSketchInfo("Relay with Switch", "1.0");
  
  // Register sensors to node
  node.present(CHILD_ID1, S_LIGHT);
  node.present(CHILD_ID2, S_LIGHT);
  
  node.sendBatteryLevel(100);			// send battery percent even though this is not a battery device -- Otherwise controller has nothing to display

  // setup the various IO pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // weak pull up for switch


  // Make sure relays are off when starting up
  digitalWrite(RELAY_PIN1, 0);
  digitalWrite(RELAY_PIN2, 0);
  // Then set relay pins in output mode
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);

 
  // Set relay to last known state (using EE-Prom storage)
  // node.loadState(CHILD_ID); // not such a good idea as it wears out the EEPROM
  

  // start with relays off
  node.send(msg1.set(0), true); // Send initial state so the controller is in sync with the switch
  node.send(msg2.set(0), true); // Send initial state so the controller is in sync with the switch
 
  
  ChangeSwitch(msg1);  // affect switch
  ChangeSwitch(msg2);  // affect switch
 
}


void    // Arduino IDE repetitive function
loop()
{
   node.process();   // must be called often so that the repeater works

// NOTE: getting a reply back from the GW after sending a toggled state doesn't always work!
// Often the node doesn't get a reply from the gateway to turn the switch after we have send the topggled state. 
// It is not know if the controller is simply not sending out a message, or if the device is not fast enough getting back to reveiving mode 
// after transmitting and therefore misses the transmission. 
// If the node is setup WITHOUT hardware ack request then the device never gets the reply at all.
// To fix the above the code directly changes the local state variable and controls the switch immediatly instead of waiting for a
// reply back from the controller/GW. Should a reply be received then the switch simply gets set again to the same state as it's already at.

  if (digitalRead(BUTTON_PIN) == 0)  // only affects relay 1 
  {
 
    node.wait(500); // pause at least 500ms or until the button is released
    while( digitalRead(BUTTON_PIN) == 0)
    ; 
    bool state1 = (msg1.getBool() ? false : true);  // Toggle 
    node.send(msg1.set(state1), true); // Send new state (toggle) and request ack back


    ChangeSwitch(msg1);  // affect switch

  
  }
}





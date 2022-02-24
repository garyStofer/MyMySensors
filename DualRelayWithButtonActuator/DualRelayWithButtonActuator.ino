/* 
 *   Sensor node that acts as a single or dual light or appliance switch 
 *   Push button on node controls relay #1 only. Push button control works locally since feedback via controller is not always reliable or speedy. 
 *     
 */
#include <EEPROM.h>
#include <MySensor.h>

#define RELAY_PIN1  16  // aka A2, PC2, ADC2 -- chip pin 25
#define RELAY_PIN2  17  // aka A3, PC3, ADC3 -- chip pin 26
#define BUTTON_PIN  7  // Arduino Digital I/O pin number for button 
#define CHILD_ID1 1   // Id of the first sensor child
#define CHILD_ID2 2  // Id of the second sensor child
#define LED_PIN   8     // There are 2 LEDS on this pin -- RED from D8 to GND, and blue or white from VCC to D8 -- Tri-state for leds off

// #define DUAL                  // uncomment for dual realay node 
#define RepeaterNode false        // 


MySensor node;
MyMessage msg1(CHILD_ID1, V_LIGHT); 
#ifdef DUAL  
MyMessage msg2(CHILD_ID2, V_LIGHT);   
#endif


void ChangeRelays(const MyMessage& msg)
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
#ifdef DUAL
  else if (msg.sensor == CHILD_ID2)
  {
    msg2.set(s);
    digitalWrite(RELAY_PIN2, s );
    Serial.print( "2: ");
    Serial.println( s);
  }
#endif

}



void    // This function is called from the radio stack when data is reveived 
dataIndication(const MyMessage &message)
{
  // We only expect one type of message from controller. But better check anyway.
  Serial.print("dataIndication:"); 

  if (message.isAck())
  {
    Serial.println("This is an ack from gateway");
  }
  else if (message.type == V_LIGHT)
  {
   // Write some debug info
    Serial.print("Incoming msg for Relay:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
    ChangeRelays(message);
  }
  else 
  {
    Serial.print("Unknown message received");
    Serial.println(message.type);    
  }

}

void    // arduino IDE initial function
setup()
{
  Serial.begin(57600);  // This Baud rate needs to correspond to the Baudrate defined in Mysensor.h
  Serial.println( "\nNode Startup");
  // setup the various IO pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); // weak pull up for switch
  
  // Make sure relays are off when starting up
  digitalWrite(RELAY_PIN1, 0);
  pinMode(RELAY_PIN1, OUTPUT);

#ifdef DUAL    
  digitalWrite(RELAY_PIN2, 0);
  pinMode(RELAY_PIN2, OUTPUT);
#endif  
  
  // to erase EEPROM holding device ID in order to create a uncommissioned device 
  if (digitalRead( BUTTON_PIN )==0 )
  {

      for (int i = 0; i < 512; i++)
        EEPROM.write(i, 0xff);

      Serial.println("Clearing of EEprom complete.");
  }
  
  Serial.println( "\nNode.Begin");
  //node.begin(dataIndication, AUTO, RepeaterNode);   // 2nd arg is forcing a discrete node ID,3rd arg==true == a repeater node 
  node.begin(dataIndication, AUTO, RepeaterNode);   

 // dummy call to get the node id assigned on initial boot of the device
  Serial.print( "\nSending dummy Sketch info:\n");
  node.sendSketchInfo("dummy", "info",true);// 3rd arg==true is ackrequest for this transaction

  int nodenr = node.getNodeId();

  Serial.print( "Relay with Switch is node# 0x");
  Serial.print( nodenr, HEX);
  Serial.print( " version ");
  Serial.println( LIBRARY_VERSION);
  

  // again -- this time it will take
  Serial.print( "\nSending Sketch info:\n");
  node.sendSketchInfo("Relay with Switch", "1.10",true);// 3rd arg==true is ackrequest for this transaction
  
  // Register sensors to node
  Serial.println( "Present Sensor 1 with ack req");
  node.present(CHILD_ID1, S_LIGHT, true,"AC-SW-1"); // 3rd arg==true is ackrequest
  
#ifdef DUAL
  Serial.println( "Present Sensor 2 with ack req");
  node.present(CHILD_ID2, S_LIGHT, true,"AC-SW-2");
#endif  
  
  
//  Serial.println( "Send Battery level without ack req");
 // 2nd arg==true is ackrequesr
//  node.sendBatteryLevel(59, false);			// send battery percent even though this is not a battery device -- Otherwise controller has nothing to display
  
 // get the state of the lights from the controller 
  Serial.println( "Req initial state from controller");
  node.request(CHILD_ID1, V_LIGHT); 
  
#ifdef DUAL
  node.request(CHILD_ID2, V_LIGHT); 
#endif
 
 }


void    // Arduino IDE repetitive function
loop()
{
   node.process();   // must be called often so that the repeater works

// NOTE: getting a reply back from the GW after sending a toggled state doesn't always work because of potential range issues.

// To fix this the code directly changes the local state variable and controls the relais immediatly instead of waiting for a
// reply back from the controller/GW. Should a reply be received then the switch simply gets set again to the same state as it's already at.
// If the outgoing "set" message to the controller did not make it but the "request" got through then the relays flicks momentairly. This is an indication
// that the connection to the controller is weak, however if the request message is lost then no indication is present. 

  if (digitalRead(BUTTON_PIN) == 0)  // affects relay 1 
  {
    bool state1 = (msg1.getBool() ? false : true);  // Toggle 
    msg1.set(state1);
    ChangeRelays(msg1);  // affect switch immediatly -- so that the switch still works when the gateway or controller is missing 
    
    node.wait(500); // pause for debounce, then wait until the button is released
    while( digitalRead(BUTTON_PIN) == 0)
    ; 
   
    Serial.println( "Send toggled status for child 1 with endtoend ackreq");
    node.send( msg1, true); // Send new state (toggle) and request ack back


    // follow up with a read request which then will be received by the dataIndication function. 
    // if the connection to the gateway is flakey then the light might flicker on/off or off/on if the controller is out of sync 
    // with the node. 
   
    node.request(CHILD_ID1, V_LIGHT);

  }
}





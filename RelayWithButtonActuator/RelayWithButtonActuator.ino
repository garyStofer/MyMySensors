#include <MySensor.h>
#include <SPI.h>
#include <Bounce2.h>

#define RELAY_PIN  16  // aka A2, PC2, ADC2 -- chip pin 25
#define BUTTON_PIN  7  // Arduino Digital I/O pin number for button 
#define BUTTON_GND_PIN 5
#define CHILD_ID 1   // Id of the sensor child
#define RELAY_ON 0
#define RELAY_OFF 1
#define LED_PIN 8

Bounce debouncer = Bounce();
int oldValue = 0;
bool state;
MySensor node;
MyMessage msg(CHILD_ID, V_LIGHT);

void dataIndication(const MyMessage &message)
{
  // We only expect one type of message from controller. But we better check anyway.
  if (message.isAck())
  {
    Serial.println("This is an ack from gateway");
  }

  if (message.type == V_LIGHT)
  {
    // Change relay state
    state = message.getBool();
    digitalWrite(RELAY_PIN, state ? RELAY_OFF : RELAY_ON);
    digitalWrite(LED_PIN, state ? RELAY_ON : RELAY_OFF);

    // Store state in EE-prom
    // node.saveState(CHILD_ID, state);

    // Write some debug info
    Serial.print("Incoming change for Relay:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(state);
  }
}

void setup()
{
  Serial.begin(115400);
  Serial.print( "Relay with Switch ");
  Serial.println( LIBRARY_VERSION);
  Serial.flush();
  
  
  node.begin(dataIndication, AUTO, true);
  // Send the sketch version information to the gateway and Controller
  node.sendSketchInfo("Relay with Switch", "1.0");
  // Register sensors to node
  node.present(CHILD_ID, S_LIGHT);
  node.sendBatteryLevel(100);			// send battery percent even though this is not a battery device -- Otherwise controller has nothing to display

 

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Setup the button
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUTTON_GND_PIN, OUTPUT);
  digitalWrite(BUTTON_GND_PIN, LOW);
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN, HIGH);

  // After setting up the button, setup debouncer
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5);

  // Make sure relays are off when starting up
  digitalWrite(RELAY_PIN, RELAY_OFF);
  // Then set relay pins in output mode
  pinMode(RELAY_PIN, OUTPUT);

  // Set relay to last known state (using EE-Prom storage)
  state = 0; // node.loadState(CHILD_ID);
  digitalWrite(RELAY_PIN, state ? RELAY_OFF : RELAY_ON);
  digitalWrite(LED_PIN, state ? RELAY_ON : RELAY_OFF);

}


void loop()
{
  node.process();
  debouncer.update();
  // Get the update value
  int value = debouncer.read();

  if (value != oldValue && value == 0) // edge to 0
  {
    Serial.println("Sending the toggled state to controller");

    node.send(msg.set(state ? false : true), true); // Send new state (toggle) and request ack back
  }

  oldValue = value;
}





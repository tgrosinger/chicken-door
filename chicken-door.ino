#include <Stepper.h>

// LED Values
const int ledPin = 13;

// Stepper and Solenoid mosfet
const int mosPin = 4;

// Stepper Values
const int stepsPerRevolution = 513;
const int coilA1Pin = 11;
const int coilA2Pin = 9;
const int coilB1Pin = 7;
const int coilB2Pin = 10;
Stepper doorStepper = Stepper(stepsPerRevolution, coilA1Pin, coilA2Pin, coilB1Pin, coilB2Pin);

// Solenoid Values
const int solPin = 1;

// Light Meter Values
const int lightPin = A3;
const int doorOpenThreshold = 350;
const int doorCloseThreshold = 325;
const long doorCloseDelay = 10 * 1000L; // 60 seconds // TODO
int lightValue = 0;

// Switches
const int doorOpenSwitch = A2;
const int doorCloseSwitch = A1;

// Door States
const int doorOpen = 1;
const int doorClosed = 2;
const int doorPreClose = 3;
const int doorUnknown = 4;
const int doorJammed = 5; // took too long to gravity close
const int doorJamCleared = 6;
const int expectedDoorCloseDurationMilli = 10 * 1000L; // 10 seconds
//const unsigned long minDoorOpenDuration = 8 * 60 * 60 * 1000L; // 8 hours
const unsigned long minDoorOpenDuration = 60 * 1000L; // TODO

const bool debug = true;
const int sec = 1000;

class Door {
  int doorState;
  unsigned long doorPreCloseMillis;
  unsigned long doorLastOpenMillis;

  public:
  Door() {
    doorState = doorUnknown;
    doorPreCloseMillis = 0;
    doorLastOpenMillis = 0;
  }

  void Update() {
    int lightValue = analogRead(lightPin);
    if (debug) {
      Serial.print("Light level: ");
      Serial.println(lightValue);
    }

    if (doorState == doorJammed || doorState == doorUnknown) {
      Open();
    }

    if (lightValue > doorOpenThreshold) {
      Open();
    } else if (lightValue < doorCloseThreshold) {
      Close();
    }
  }

  void Open() {
    if (doorState == doorOpen && digitalRead(doorOpenSwitch) == LOW) {
      return;
    }
 
    Serial.println("Going to open the door");
    
    digitalWrite(mosPin, HIGH); // Enable the motors and solenoid
    digitalWrite(solPin, HIGH); // Retract the lock

    while(digitalRead(doorOpenSwitch) == HIGH) {
      doorStepper.step(stepsPerRevolution / 8); // Open the door, 1/8 rotation
    }
 
    digitalWrite(solPin, LOW); // Extend the lock
    delay(sec);
    digitalWrite(mosPin, LOW); // Disable the motors and solenoid

    Serial.println("Door is open");

    if (doorState == doorJammed) {
      doorState = doorJamCleared;
    } else {
      doorState = doorOpen;
      doorLastOpenMillis = millis(); 
    }
  }

  void Close() {
    if (doorState == doorClosed && digitalRead(doorCloseSwitch) == LOW) {
      return;
    }

    if ((unsigned long)(millis() - doorLastOpenMillis) < minDoorOpenDuration) {
      Serial.println("Not closing, opened too recently");
      delay(sec);
      return;
    }

    if (doorState == doorOpen || doorState == doorJamCleared) {
      Serial.println("Entering door pre-close");
      doorState = doorPreClose;
      doorPreCloseMillis = millis();
    }

    if (doorState == doorPreClose) {
      long elapsed = ((unsigned long)(millis() - doorPreCloseMillis));
      if (elapsed < doorCloseDelay) {
        delay(sec);
        return;
      }

      // Finished pre-close delay
    }

    Serial.print("Going to close the door\n");

    // TODO: Add a "last-call" where door is opened briefly after 
    //       5 minutes, then closed again

    digitalWrite(mosPin, HIGH); // Enable the motors and solenoid
    digitalWrite(solPin, HIGH); // Retract the lock

    unsigned long doorCloseStart = millis();
    unsigned long interval = expectedDoorCloseDurationMilli * 1.5;
    while(digitalRead(doorCloseSwitch) == HIGH &&
            (unsigned long)(millis() - doorCloseStart) < interval) {
      doorStepper.step(stepsPerRevolution / 8 * -1); // Close the door, 1/8 rotation
    }

    Serial.print("Door close took ");
    Serial.print(millis() - doorCloseStart);
    Serial.println(" millis");

    if (digitalRead(doorCloseSwitch) == HIGH) {
      Serial.println("Door did not close, likely jammed");
      doorState = doorJammed;
      // Do not diable the solenoid or motor jump to update loop to bounce open and closed
      return;
    }
 
    digitalWrite(solPin, LOW); // Extend the lock
    delay(sec);
    digitalWrite(mosPin, LOW); // Disable the motors and solenoid

    Serial.println("Door is closed");
    doorState = doorClosed;
  }
};

class Flasher {
  int ledPin;      // the number of the LED pin
  long OnTime;     // milliseconds of on-time
  long OffTime;    // milliseconds of off-time

  int ledState;                   // ledState used to set the LED
  unsigned long previousMillis;   // will store last time LED was updated

  public:
  Flasher(int pin, long on, long off) {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);     
 
    OnTime = on;
    OffTime = off;

    ledState = LOW; 
    previousMillis = 0;
  }
 
  void Update() {
    // check to see if it's time to change the state of the LED
    unsigned long currentMillis = millis();
 
    if((ledState == HIGH) && ((unsigned long)(currentMillis - previousMillis) >= OnTime)) {
      ledState = LOW;  // Turn it off
      previousMillis = currentMillis;  // Remember the time
      digitalWrite(ledPin, ledState);  // Update the actual LED
    } else if ((ledState == LOW) && ((unsigned long)(currentMillis - previousMillis) >= OffTime)) {
      ledState = HIGH;  // turn it on
      previousMillis = currentMillis;   // Remember the time
      digitalWrite(ledPin, ledState);   // Update the actual LED
    }
  }
};

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(ledPin, OUTPUT);

  // initialize stepper pins
  pinMode(coilA1Pin, OUTPUT);
  pinMode(coilA2Pin, OUTPUT);
  pinMode(coilB1Pin, OUTPUT);
  pinMode(coilB2Pin, OUTPUT);
  doorStepper.setSpeed(20);

  // initialize pins
  pinMode(solPin, OUTPUT);
  pinMode(mosPin, OUTPUT);
  pinMode(doorOpenSwitch,  INPUT_PULLUP);
  pinMode(doorCloseSwitch, INPUT_PULLUP);

  //Initiate Serial communication
  if (debug) {
    Serial.begin(9600);
  }
}

Flasher led = Flasher(ledPin, 500, 500);
Door door = Door();

// loop runs forever
void loop() {
  led.Update();
  door.Update();

  delay(500);
}

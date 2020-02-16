#include <Stepper.h>

// LED Values
const int ledPin = 13;

// Stepper Values
const int stepsPerRevolution = 200;
const int coilA1Pin = 12;
const int coilA2Pin = 11;
const int coilB1Pin = 10;
const int coilB2Pin = 9;
const int driverEnPin = 0;
Stepper doorStepper = Stepper(stepsPerRevolution, coilA1Pin, coilA2Pin, coilB1Pin, coilB2Pin);

// Solenoid Values
const int solPin = 1;
const int solCoilA1Pin = 7;
const int solCoilA2Pin = 5;

// Light Meter Values
const int lightPin = A5;
const int doorOpenThreshold = 220;
const int doorCloseThreshold = 200;
const long doorCloseDelay = 20 * 1000L; // 20 seconds

int lightValue = 0;

// Switches
const int doorCloseSwitch = A1;
const int doorOpenSwitch = A2;
const int manualSwitch = A3;
const int openCloseSwitch = A4;

// Door States
const int doorOpen = 1;
const int doorClosed = 2;
const int doorPreClose = 3;
const int doorUnknown = 4;
const int doorJammed = 5; // took too long to gravity close
const int doorJamCleared = 6;
const int expectedDoorCloseDurationMilli = 16 * 1000L; // 16 seconds
const unsigned long minDoorOpenDuration = 8 * 60 * 60 * 1000L; // 8 hours

const bool debug = true;
const int sec = 1000;

void engageSolenoid() {
  digitalWrite(solPin, LOW);
  digitalWrite(solCoilA1Pin, LOW);
  digitalWrite(solCoilA2Pin, LOW);
  delay(sec/2);
}

void disengageSolenoid() {
  digitalWrite(solPin, HIGH);
  digitalWrite(solCoilA1Pin, HIGH);
  digitalWrite(solCoilA2Pin, LOW);
  delay(sec/2);
}

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

    
    if (digitalRead(manualSwitch) == LOW) {
      if (digitalRead(openCloseSwitch) == LOW) {
        Serial.println("door manually set to open");
        Open();
      } else {
        Serial.println("door manually set to close");
        Close();
      }

      return;
    }

    if (lightValue > doorOpenThreshold) {
      Open();
    } else if (lightValue < doorCloseThreshold) {
      CloseWithPreCloseChecks();
    }
  }

  void Open() {
    if (doorState == doorOpen && digitalRead(doorOpenSwitch) == LOW) {
      return;
    }
 
    Serial.println("Going to open the door");
    
    digitalWrite(driverEnPin, HIGH); // Enable the motor
    disengageSolenoid(); // Retract the lock

    while(digitalRead(doorOpenSwitch) == HIGH) {
      doorStepper.step(stepsPerRevolution / 8); // Open the door, 1/8 rotation
    }
 
    engageSolenoid(); // Extend the lock
    doorStepper.step(-1 * stepsPerRevolution / 4); // Lower door onto solenoid
    digitalWrite(driverEnPin, LOW); // Disable the motor

    Serial.println("Door is open");

    if (doorState == doorJammed) {
      doorState = doorJamCleared;
    } else {
      doorState = doorOpen;
      doorLastOpenMillis = millis(); 
    }
  }

  void CloseWithPreCloseChecks() {
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

    Close();
  }

  void Close() {
    if (doorState == doorClosed && digitalRead(doorCloseSwitch) == LOW) {
      return;
    }

    Serial.println("Going to close the door");

    digitalWrite(driverEnPin, HIGH); // Enable the motor
    doorStepper.step(stepsPerRevolution / 3); // Lift door off solenoid
    disengageSolenoid(); // Retract the lock

    unsigned long doorCloseStart = millis();
    unsigned long interval = expectedDoorCloseDurationMilli * 1.5;
    while(digitalRead(doorCloseSwitch) == HIGH &&
            (unsigned long)(millis() - doorCloseStart) < interval) {
      doorStepper.step(stepsPerRevolution / 8 * -1); // Close the door, 1/8 rotation
    }
    doorStepper.step(stepsPerRevolution / 8 * -1);

    if (digitalRead(doorCloseSwitch) == HIGH) {
      Serial.println("Door did not close, likely jammed");
      doorState = doorJammed;
      // Do not diable the solenoid or motor jump to update loop to bounce open and closed
      return;
    }

    Serial.print("Door close took ");
    Serial.print(millis() - doorCloseStart);
    Serial.println(" millis");
 
    engageSolenoid(); // Extend the lock
    digitalWrite(driverEnPin, LOW); // Disable the motor

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
  doorStepper.setSpeed(40);

  // initialize pins
  pinMode(solPin, OUTPUT);
  pinMode(solCoilA1Pin, OUTPUT);
  pinMode(solCoilA2Pin, OUTPUT);
  pinMode(driverEnPin, OUTPUT);
  pinMode(doorOpenSwitch,  INPUT_PULLUP);
  pinMode(doorCloseSwitch, INPUT_PULLUP);
  pinMode(manualSwitch, INPUT_PULLUP);
  pinMode(openCloseSwitch, INPUT_PULLUP);

  //Initiate Serial communication
  if (debug) {
    Serial.begin(9600);
  }

  delay(2000);
}

Flasher led = Flasher(ledPin, 500, 500);
Door door = Door();

// loop runs forever
void loop() {
  led.Update();
  door.Update();
  
  delay(500);
}

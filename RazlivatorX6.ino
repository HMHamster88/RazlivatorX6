#include <AccelStepper.h>
#include <avr/sleep.h>

const int minDose = 10;
const int maxDose = 40;
const int cupCount = 6;
const int servoPerMlTime = 40;

const int stepperFullRotateSteps = 4076 / 2;
const int stepperAcceleration = 200;
const int stepperMaxSpeed = 200;
const int startAngle = 10;
const int cupPositions[] = {
  60,
  415,
  790,
  1120,
  1460,
  1810
  };
/*                                +-----+
                     +------------| USB |------------+
                     |            +-----+            |
                B5   | [ ]D13/SCK        MISO/D12[ ] |   B4
                     | [ ]3.3V           MOSI/D11[ ]~|   B3
                     | [ ]V.ref     ___    SS/D10[ ]~|   B2
          CUP_0 C0   | [ ]A0       / N \       D9[ ]~|   B1   stepper
          CUP_1 C1   | [ ]A1      /  A  \      D8[ ] |   B0   stepper
          CUP_2 C2   | [ ]A2      \  N  /      D7[ ] |   D7   stepper
          CUP_3 C3   | [ ]A3       \_0_/       D6[ ]~|   D6   stepper
          CUP_4 C4   | [ ]A4/SDA               D5[ ]~|   D5   parkButtonPin
          CUP_5 C5   | [ ]A5/SCL               D4[ ] |   D4   pumpPin
           dosePin   | [ ]A6              INT1/D3[ ]~|   D3   doseVccPin
                     | [ ]A7              INT0/D2[ ] |   D2   startButtonPin
                     | [ ]5V                  GND[ ] |     
                C6   | [ ]RST                 RST[ ] |   C6
                     | [ ]GND   5V MOSI GND   TX1[ ] |   D0
                     | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |   D1
                     |          [ ] [ ] [ ]          |
                     |          MISO SCK RST         |
                     | NANO-V3                       |
                     +-------------------------------+
*/         

class Cup {
  const int _pin;
  public:
    Cup(int pin): _pin(pin) {
    }

    void setup() const {
      pinMode(_pin, INPUT_PULLUP);
    }

    bool pressed() const {
      return !digitalRead(_pin);
    }
};

const Cup cups[] = {
  Cup(A0),
  Cup(A1),
  Cup(A2),
  Cup(A3),
  Cup(A4),
  Cup(A5)
};

const int dosePin = A6;
const int doseVccPin = 3;
const int startButtonPin = 2;
const int pumpPin = 4;
const int parkButtonPin = 5;
AccelStepper stepper(AccelStepper::FULL4WIRE, 6, 8, 7, 9);

void wakeUpNow() {
}

void setup() {
  Serial.begin(9600);
  for(int cupButton = 0; cupButton < cupCount; cupButton++) {
    cups[cupButton].setup();
  }
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(doseVccPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);

  pinMode(parkButtonPin, INPUT_PULLUP);
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
}

void rotateArm(int cup) {
  stepper.enableOutputs();
  stepper.moveTo(cupPositions[cup]);
  while(stepper.isRunning()) {
    stepper.run();
  }
  stepper.disableOutputs();
}

bool parkButtonPressed() {
  return !digitalRead(parkButtonPin);
}

bool startButtonPressed() {
  return !digitalRead(startButtonPin);
}

bool park() {
  stepper.enableOutputs();
  if(parkButtonPressed()) {
    return true;
  }
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
  stepper.moveTo(-stepperFullRotateSteps);
  while(!parkButtonPressed() && stepper.isRunning() ) {
    stepper.run();
  }
  delay(100);
  if (!parkButtonPressed()) {
    return false;
  }
  stepper.setCurrentPosition(0);
  stepper.disableOutputs();
  return true;
}

int getDose() {
  digitalWrite(doseVccPin, HIGH);
  int adc = analogRead(dosePin);
  digitalWrite(doseVccPin, LOW);
  return minDose + ((maxDose - minDose) * adc) / 1023;
}

void pump(int dose) {
  digitalWrite(pumpPin, HIGH);
  delay(servoPerMlTime * dose);
  digitalWrite(pumpPin, LOW);
}

void sleepNow()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();         
  attachInterrupt(digitalPinToInterrupt(startButtonPin), wakeUpNow, FALLING);
  sleep_mode();
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(startButtonPin));
}

//#define CALLIBRATION

bool anyCupPressed() {
  for(int cupButton = 0; cupButton < cupCount; cupButton++) {
      if(cups[cupButton].pressed()) {
        return true;
      }
  }
  return false;
}

void loop() {
  #ifdef CALLIBRATION
  park();
  int num = Serial.parseInt();
  stepper.moveTo(num);
  while(stepper.isRunning()) {
    stepper.run();
  }
  delay(1000);
  #else
  sleepNow();
  delay(100);
  if (!startButtonPressed()) {
    return;
  }
  if(anyCupPressed()) {
    park();
    int dose = getDose();
    for(int cupButton = 0; cupButton < cupCount; cupButton++) {
      if (cups[cupButton].pressed()) {
        rotateArm(cupButton);
        pump(dose);
        delay(500);
      }
    }
    rotateArm(0);
  } else {
    if(startButtonPressed()) {
      delay(2000);
      if (!startButtonPressed()) {
        return;
      }
      park();
      rotateArm(0);
      digitalWrite(pumpPin, HIGH);
      while(startButtonPressed()) {
      }
      digitalWrite(pumpPin, LOW);
    }
  }  
  #endif
}

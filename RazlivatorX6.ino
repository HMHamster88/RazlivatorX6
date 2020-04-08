#include <AccelStepper.h>
#include <avr/sleep.h>

const int minDose = 10;
const int maxDose = 40;
const int cupCount = 6;
const int servoPerAngleTime = 1000 / 180;
const int servoMinImpulse = 544;
const int servoMaxImpulse = 2250;
const int servoPerMlTime = 40;

const int stepperFullRotateSteps = 4076 / 2;
const int stepperAcceleration = 200;
const int stepperParkStopAcceleration = 2000;
const int stepperMaxSpeed = 200;

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
           dosePin   | [ ]A6              INT1/D3[ ]~|   D3
        doseVccPin   | [ ]A7              INT0/D2[ ] |   D2   startButtonPin
                     | [ ]5V                  GND[ ] |     
                C6   | [ ]RST                 RST[ ] |   C6
                     | [ ]GND   5V MOSI GND   TX1[ ] |   D0
                     | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |   D1
                     |          [ ] [ ] [ ]          |
                     |          MISO SCK RST         |
                     | NANO-V3                       |
                     +-------------------------------+
*/         

const int cupButtonPins[] = {
  A0,
  A1,
  A2,
  A3,
  A4,
  A5
};

const int dosePin = A6;
const int doseVccPin = 7;
const int startButtonPin = 2;
const int pumpPin = 4;
const int parkButtonPin = 5;
AccelStepper stepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);


void wakeUpNow() {
}

void setup() {
  Serial.begin(9600);
  for(int cupButton = 0; cupButton < cupCount; cupButton++) {
    pinMode(cupButtonPins[cupButton], INPUT_PULLUP);
  }
  analogReference(EXTERNAL);
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(doseVccPin, OUTPUT);

  pinMode(parkButtonPin, INPUT_PULLUP);
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
}

void rotateArm(int cup) {
  int angle = 180 - cup * (180 / (cupCount - 1));
  Serial.println("Rotate arm to cup " + String(cup) + " at angle " + String(angle));

  Serial.println("Rotate arm finished ");
}

bool parkButtonPressed() {
  return !digitalRead(parkButtonPin);
}

bool park() {
  Serial.println("Parking arm...");
  if(parkButtonPressed()) {
    Serial.println("Already parked");
    return true;
  }
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
  stepper.moveTo(stepperFullRotateSteps);
  while(!parkButtonPressed() && stepper.isRunning() ) {
    stepper.run();
  }
  if (!parkButtonPressed()) {
    Serial.println("Park failed.");
    return false;
  }
  Serial.println("Park button pressed, stopping...");
  stepper.setAcceleration(stepperParkStopAcceleration);
  stepper.stop();
  while(stepper.isRunning()) {
    stepper.run();
  }
  Serial.println("Park finished");
  stepper.setAcceleration(stepperAcceleration);
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

void loop() {
  sleepNow();
  delay(100);
  if (digitalRead(startButtonPin)) {
    return;
  }
  park();
  bool cups[cupCount];
  bool anyCupPpressed = false;
  for(int cupButton = 0; cupButton < cupCount; cupButton++) {
    bool cupPressed = !digitalRead(cupButtonPins[cupButton]);
    anyCupPpressed = anyCupPpressed || cupPressed;
    cups[cupButton] = cupPressed;
  }
  if(!anyCupPpressed) {
    return;
  }

  int dose = getDose();
  for(int cupButton = 0; cupButton < cupCount; cupButton++) {
    if (cups[cupButton]) {
      rotateArm(cupButton);
      pump(dose);
      delay(1500);
    }
  }
  delay(100);
}

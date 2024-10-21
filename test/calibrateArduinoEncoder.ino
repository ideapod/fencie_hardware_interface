/* this calibration programme is intended to work out how many counts 
 * per revolution of the wheel so we can accurately determine position. 
 * and thus, velocity. 
 * as per articulated robotics directions, put tape on the wheel and count how many revolutions it makes
 * in the 'forward motion' part of the execution (from Going Forward to Stopping)
 * divide this my the number of revolutions to get a count per revolution. 

/* pin mappings */

#define frEn 2 // front right enable
#define rFwd 13 // right side forward (one pin forward both rr and fr )
#define rRvr 12 // right side reverse
#define frPhA 26
#define frPhB 29
#define rrEn 15 // rear  right enable
#define rrPhA 34
#define rrPhB 35

#define flEn 4
#define lFwd 17 
#define lRvr 16
#define flPhA 27
#define flPhB 14
#define rlEn 0
#define rlPhA 32
#define rlPhB 33

#define MAX_ENCODERS 8
#define MAX_NAME_LENGTH 15

typedef enum direction_set {
  directionForward,
  directionReverse,
  directionStopped
} DirectionSet;


struct wheel_sensor {
  char name[MAX_NAME_LENGTH];
  int pinA;
  int pinB;
  bool pinStateA;
  bool pinStateB;
  bool oldPinStateA;
  bool oldPinStateB;
  DirectionSet direction;
  int pulseCount;
} encoders[MAX_ENCODERS];
int numEncoders = 0;
int loopCount = 0;
DirectionSet directionCommand = directionStopped;

void setForward() {
 
  digitalWrite(rFwd, HIGH);
  digitalWrite(rRvr, LOW);
  digitalWrite(lFwd, HIGH);
  digitalWrite(lRvr, LOW);
}

void setReverse() {
  digitalWrite(rFwd, LOW);
  digitalWrite(rRvr, HIGH);
  digitalWrite(lFwd, LOW);
  digitalWrite(lRvr, HIGH);
}

void setStop() {
  digitalWrite(rFwd, LOW);
  digitalWrite(rRvr, LOW);
  digitalWrite(lFwd, LOW);
  digitalWrite(lRvr, LOW);
}

void savePinStates() {
  for (int i=0;i<numEncoders; i++) {
    savePinState(i);
  }
}

void savePinState(int i) {
    encoders[i].oldPinStateA= encoders[i].pinStateA;
    encoders[i].oldPinStateB= encoders[i].pinStateB;
}

void getPinStates() {
  for (int i=0;i<numEncoders; i++) {
    getPinState(i);
  }
}

void getPinState(int i) {
    encoders[i].pinStateA = (digitalRead(encoders[i].pinA) == HIGH);
    encoders[i].pinStateB = (digitalRead(encoders[i].pinB) == HIGH);
}

void pulseEncoder(int i) {
  getPinState(i);

  if (!encoders[i].oldPinStateA && encoders[i].pinStateA) {
      // low to high on pinA 

      if (!encoders[i].pinStateB && (encoders[i].direction == directionForward)) {
        // direction changed
        encoders[i].direction = directionReverse;
      } else if (encoders[i].pinStateB && (encoders[i].direction == directionReverse)) {
        // direction changed
        encoders[i].direction = directionForward;
      }
    }

    if (encoders[i].direction == directionForward) {
      encoders[i].pulseCount++;
    } else {
      encoders[i].pulseCount--;
    }

  savePinState(i);
}

void wheelSpeedISR0()
{
  pulseEncoder(0);
}

void wheelSpeedISR1() 
{
  pulseEncoder(1);
}

void wheelSpeedISR2() 
{
  pulseEncoder(2);
}

void wheelSpeedISR3() 
{
  pulseEncoder(3);
}



void setupMotors() {
  pinMode(frEn, OUTPUT);
  pinMode(rrEn, OUTPUT);
  pinMode(rFwd, OUTPUT);
  pinMode(rRvr, OUTPUT);
  pinMode(flEn, OUTPUT);
  pinMode(rlEn, OUTPUT);
  pinMode(lFwd, OUTPUT);
  pinMode(lRvr, OUTPUT);

     // Set initial rotation direction
  setForward();
}

void initEncoderPins() {
  numEncoders=4;

  encoders[0].pinA = frPhA;
  encoders[0].pinB = frPhB;
  encoders[0].direction = directionForward;
  strcpy(encoders[0].name, "Front Right");

  encoders[1].pinA = rrPhA;
  encoders[1].pinB = rrPhB;
  encoders[1].direction = directionForward;
  strcpy(encoders[1].name, "Rear Right");

  encoders[2].pinA = flPhA;
  encoders[2].pinB = flPhB;
  encoders[2].direction = directionForward;
  strcpy(encoders[2].name, "Front Left");

  encoders[3].pinA = rlPhA;
  encoders[3].pinB = rlPhB;
  encoders[3].direction = directionForward;
  strcpy(encoders[3].name, "Rear Left");




  for (int i=0; i < numEncoders; i++) {
    pinMode(encoders[i].pinA, INPUT);  
    pinMode(encoders[i].pinB, INPUT);  
    encoders[i].pulseCount = 0;
    // attachInterrupt(digitalPinToInterrupt(pin), ISR, mode); (recommended)
  }

  attachInterrupt(encoders[0].pinA, wheelSpeedISR0, CHANGE);
  attachInterrupt(encoders[1].pinA, wheelSpeedISR1, CHANGE);
  attachInterrupt(encoders[2].pinA, wheelSpeedISR2, CHANGE);
  attachInterrupt(encoders[3].pinA, wheelSpeedISR3, CHANGE);
}



void setup(){
	
	Serial.begin(115200);
	// Enable the weak pull down resistors

  initEncoderPins();
  // setupEncoders();
  setupMotors();
	Serial.println("Setup Complete");
}

void printState() {
  // Loop and read the count
  Serial.println("Encoders:");
  for(int i=0;i<numEncoders;i++) {
	  Serial.println(String(encoders[i].name) + " count = " + String(encoders[i].pulseCount));
  }
  Serial.println("loopCount: " + String(loopCount));

  
}

void processMotors() {
  int pwmOutput = 35;
  analogWrite(frEn, pwmOutput); // Send PWM signal to L298N Enable pin
  analogWrite(rrEn, pwmOutput);

  analogWrite(flEn, pwmOutput); // Send PWM signal to L298N Enable pin
  analogWrite(rlEn, pwmOutput);
}

void forwardStopReverse() {
  if (directionCommand == directionReverse) {
    setStop();
    directionCommand = directionStopped;
    Serial.println("Stopping");
  } else if (directionCommand == directionStopped) {
    setForward();
    Serial.println("Going Forward");
    directionCommand = directionForward;
  } else if  (directionCommand == directionForward) {
    setReverse();
    Serial.println("Going Reverse");
    directionCommand = directionReverse;
  }

  printState();
}

void forwardStop() {
  if (directionCommand == directionForward)
  {
    setStop();
    directionCommand = directionStopped;
    Serial.println("Stopping");

  } else if (directionCommand == directionStopped) {
    setForward();
    Serial.println("Going Forward");
    directionCommand = directionForward;

  } 
  printState();
}

void loop(){
  if ((loopCount % 100) == 0) {
    forwardStop();
    processMotors();
  }

  if ((loopCount % 20) == 0) {
    printState();
  }
  delay(100);
  loopCount++;
}








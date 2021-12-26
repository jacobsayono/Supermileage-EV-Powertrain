// RPM Detection from Hall Effect Sensor (code that will be added into main BLDC motor code OR add PID controller code into this one)
// We want to know how frequent the hall sensor identifies motor state changes
// Key problem: we have 2 motors to control but want to just run 1 code with 1 controller, so avoid using 'while loop' in loop() function
// We cannot allow the code to become stuck within 'while loop' for the first motor (if conditions aren't met) and must continue for the second motor

// *** Need to adjust saveTimeData() function. Right now array would only call same values every time ***

#define HALL_A1 A1
#define HALL_A2 A2
#define HALL_B1 A3
#define HALL_B2 A4
#define HALL_C1 A5
#define HALL_C2 A6

#define HALL_REPETITVE_COUNT 5 // choose odd number to set hall sensor based on majority of simulataneous readings

#define NUM_AVERAGES 10

// global variables for dual motors: motor1 and motor2
double data1[NUM_AVERAGES];
double data2[NUM_AVERAGES];
double rpmOut1; // RPM global variable used for PID controller
double rpmOut2;

// functions
void saveTimeData(unsigned long timePerState1, unsigned long timePerState2);
double averageTime(double data[]);
double findRPM1(double average1); // separate return functions for each of the 2 motors
double findRPM2(double average2);

void setup() {
  Serial.begin(115200);
  pinMode(HALL_A1, INPUT_PULLUP);
  pinMode(HALL_A2, INPUT_PULLUP);
  pinMode(HALL_B1, INPUT_PULLUP);
  pinMode(HALL_B2, INPUT_PULLUP);
  pinMode(HALL_C1, INPUT_PULLUP);
  pinMode(HALL_C2, INPUT_PULLUP);
}

void loop() {
  static byte prevState1 = readState1(); // initialize states
  static byte prevState2 = readState2();
  
  byte currentState1 = readState1(); // currentState will always renew to readState() for each iteration
  byte currentState2 = readState2();

  static unsigned long timeEnter1 = 0, timeLeave1 = 0; // initialize times
  static unsigned long timeEnter2 = 0, timeLeave2 = 0;
  
  // *** Needs to be adjusted to become an array for saveTimeData() function ***
  static unsigned long timePerState1;
  static unsigned long timePerState2;

  // motor 1
  if (currentState1 != prevState1) { // when state has changed
    prevState1 = readState1();
    timeLeave1 = millis(); // stop timer 
    timePerState1 = timeLeave1 - timeEnter1; // ignore first iteration
    timeEnter1  = millis(); // start timer
    double freq1 = 1/timePerState1; // frequency value for main BLDC motor code for debugging
    Serial.print("Motor 1 Frequency: ");
    Serial.println(freq1);

    saveTimeData(timePerState1, timePerState2);
    double avg1 = calcAverage(data1);
    rpmOut1 = findRPM(avg1);
    Serial.print("Motor 1 RPM: ");
    Serial.println(rpmOut1);
  }
  
  // motor 2
  if (currentState2 != prevState2) { // when state has changed
    prevState2 = readState2();
    timeLeave2 = millis(); // stop timer 
    timePerState2 = timeLeave2 - timeEnter2; // ignore first iteration
    timeEnter2  = millis(); // start timer
    double freq2 = 1/timePerState2; // frequency value for main BLDC motor code for debugging
    Serial.print("Motor 1 Frequency: ");
    Serial.println(freq2);

    saveTimeData(timePerState1, timePerState2);
    double avg2 = calcAverage(data2);
    rpmOut2 = findRPM(avg2);
    Serial.print("Motor 2 RPM: ");
    Serial.println(rpmOut2);
  }
}

// motor 1
byte readState1() { // function to read current state of BLDC motor from hall effect sensor
  byte tallyA1 = 0, tallyB1 = 0, tallyC1 = 0;
  bool readA1 = false, readB1 = false, readC1 = false;

  for (byte i = 0; i < HALL_REPETITVE_COUNT; i++) { // addresses hall sensor noise
    if (digitalRead(HALL_A1))
      tallyA1++;
    if (digitalRead(HALL_B1))
      tallyB1++;
    if (digitalRead(HALL_C1))
      tallyC1++;
  }
  if (tallyA1 > ceil(HALL_REPETITVE_COUNT / 2.))
    readA1 = true;
  if (tallyB1 > ceil(HALL_REPETITVE_COUNT / 2.))
    readB1 = true;
  if (tallyC1 > ceil(HALL_REPETITVE_COUNT / 2.))
    readC1 = true;

  if (readA1 && !readB1 && !readC1)
    return 6;
  else if (readA1 && readB1 && !readC1)
    return 5;
  else if (!readA1 && readB1 && !readC1)
    return 4;
  else if (!readA1 && readB1 && readC1)
    return 3;
  else if (!readA1 && !readB1 && readC1)
    return 2;
  else if (readA1 && !readB1 && readC1)
    return 1;
  return 0; // illegal state error
}

// motor 2
byte readState2() {
  byte tallyA2 = 0, tallyB2 = 0, tallyC2 = 0;
  bool readA2 = false, readB2 = false, readC2 = false;

  for (byte i = 0; i < HALL_REPETITVE_COUNT; i++) { // addresses hall sensor noise
    if (digitalRead(HALL_A2))
      tallyA2++;
    if (digitalRead(HALL_B2))
      tallyB2++;
    if (digitalRead(HALL_C2))
      tallyC2++;
  }
  if (tallyA2 > ceil(HALL_REPETITVE_COUNT / 2.))
    readA2 = true;
  if (tallyB2 > ceil(HALL_REPETITVE_COUNT / 2.))
    readB2 = true;
  if (tallyC2 > ceil(HALL_REPETITVE_COUNT / 2.))
    readC2 = true;

  if (readA2 && !readB2 && !readC2)
    return 6;
  else if (readA2 && readB2 && !readC2)
    return 5;
  else if (!readA2 && readB2 && !readC2)
    return 4;
  else if (!readA2 && readB2 && readC2)
    return 3;
  else if (!readA2 && !readB2 && readC2)
    return 2;
  else if (readA2 && !readB2 && readC2)
    return 1;
  return 0; // illegal state error
}

// *** Need to adjust saveTimeData() function, so it saves past data. Involves making timePerState in loop() function an array too - TO DO ***
void saveTimeData(unsigned long timePerState1, unsigned long timePerState2) { // function to store timePerState values into an array of NUM_AVERAGES values
  double data1[NUM_AVERAGES];
  double data2[NUM_AVERAGES];
  for (int i = 0; i < NUM_AVERAGES; i++) {
    data1[i] = timePerState1;
    data2[i] = timePerState2;
  }
}

double calcAverage(double data[]) { // function to average 10 values
  double total;
  for (int i = 0; i < NUM_AVERAGES; i++) {
    total += data[i]; // sum array
  }
  return total / NUM_AVERAGES;
}

double findRPM(double average) { // function to find RPM (from 12 states in full motor revolution)
  double averageTimePerRev = average*12; // units ms/rev
  double averageRPM = (1/averageTimePerRev)*((double)1000*60); // units rev/min
  return averageRPM;
}

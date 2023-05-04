#include <EEPROM.h>
#include <lowStepper.h>
#include <avr/sleep.h>
#include <DS3231.h>

/********************
    Dog Teeth Cleaning Toy
    ver 1.1
********************/

/* pin def */
#define RST0 1  // 3
#define INT0 2  // 4
#define PWM_B 3 // 5
#define PWM_G 5 // 11
#define PWM_R 6 // 12
#define PWM_MOTOR 9   // 15
#define PWM_BUZZER 10 // 16
#define DIG_STEP_A 0  // 2
#define DIG_STEP_B 7  // 13
#define DIG_STEP_C 4  // 6
#define DIG_STEP_D 8  // 14
#define PRESSURE_3 0  // 23
#define PRESSURE_2 1  // 24
#define PRESSURE_1 2  // 25
#define SDA 4 // 27
#define SCL 5 // 28

/* constant define */
#define MAX_BYTE 0xff
#define PRES_THRESHOLD 0x1e
#define REVOLSTEP 200
#define TIME_ADDR 0
#define BUZZ_VOL 127
#define BUZZ_DUR 3
#define INT_FREQ 60UL

byte speed;
float subTime;
float remTime;
const float sessionTime = 10.0;
bool buz_treat_flag = false;
volatile int f_wdt = 0;
lowStepper myStepper(REVOLSTEP, DIG_STEP_A, DIG_STEP_C, DIG_STEP_B, DIG_STEP_D);

DS3231 myRTC;
volatile byte tick = 0;
byte alarmDay;
byte alarmHour;
byte alarmMinute;
byte alarmSecond;
byte alarmBits;
bool alarmDayIsDay;
bool alarmH12;
bool alarmPM;

/* func prot */
void presRead(byte* p1, byte* p2, byte* p3);
void ledSet(byte r, byte g, byte b);
void ledSet_Pmos(byte r, byte g, byte b);
byte threeMax(byte* p1, byte* p2, byte* p3);
void vibSet(byte dutyCycle);
byte memSet(byte val, int addr, byte mode);
void buzzSet(byte volume, byte duration);
void stepSet(byte step);
byte speedControl(byte pressure);
void extSetup(byte stepperSpeed);
void timerSetup();
void alarmSet();
void wdSet();
void sleepSet();
void wakeUp();


/************************* main func *************************/

void setup() {
  pinMode(PRESSURE_1, INPUT_PULLUP);
  pinMode(PRESSURE_2, INPUT_PULLUP);
  pinMode(PRESSURE_3, INPUT_PULLUP);
  pinMode(INT0, INPUT_PULLUP);
  pinMode(RST0, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_G, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(PWM_MOTOR, OUTPUT);
  pinMode(PWM_BUZZER, OUTPUT);
  pinMode(DIG_STEP_A, OUTPUT);
  pinMode(DIG_STEP_B, OUTPUT);
  pinMode(DIG_STEP_C, OUTPUT);
  pinMode(DIG_STEP_D, OUTPUT);

  // make sure to set all PWM to high as default
  extSetup(20);
  wdSet();
  // timerSetup();
  remTime = memSet(sessionTime, TIME_ADDR, 0);
  // attachInterrupt(digitalPinToInterrupt(INT0), wakeUp, FALLING);
}

void loop() {
  ledSet_Pmos(MAX_BYTE, 0, 0); // red for IDLE
  byte p3, p2, p1;
  presRead(&p1, &p2, &p3);
  byte maxPressure = threeMax(&p1, &p2, &p3);
  while(maxPressure < PRES_THRESHOLD) {
    if (tick) {
      tick = 0;
      WDTCSR &= ~(_BV(WDIE));
      remTime = memSet(sessionTime, TIME_ADDR, 0);
      WDTCSR |= _BV(WDIE);
      // alarmSet();
      ledSet_Pmos(0, 0, 0);
      delay(500);
      ledSet_Pmos(MAX_BYTE, 0, 0);
    }
    // delay(1000);
    presRead(&p1, &p2, &p3);
    maxPressure= threeMax(&p1, &p2, &p3);
  }

  ledSet_Pmos(0, 0, MAX_BYTE); // blue for VIB
  do {
    speed = speedControl(maxPressure);
    vibSet(speed);
    subTime = float(speed) / 127.0;
    remTime = memSet(subTime, TIME_ADDR, 1);
    delay(1000); // changing status in a secondly basis
    presRead(&p1, &p2, &p3);
    maxPressure= threeMax(&p1, &p2, &p3);
  } while (maxPressure > PRES_THRESHOLD && remTime > 0);
  vibSet(0);

  if (remTime <= 0 && !buz_treat_flag) {
    buz_treat_flag = true;
    ledSet(HIGH, LOW, HIGH); // green for COMPLETE
    buzzSet(BUZZ_VOL, BUZZ_DUR);
    stepSet(50);
    sleepSet();
  }
}

/************************* helper func *************************/

/* read pressure from three sensor*/
void presRead(byte* p1, byte* p2, byte* p3) {
  *p3 = analogRead(PRESSURE_3) >> 2;
  *p2 = analogRead(PRESSURE_2) >> 2;
  *p1 = analogRead(PRESSURE_1) >> 2;
}

/* set RGB LED */
void ledSet(byte r, byte g, byte b) {
  digitalWrite(PWM_R, r);
  digitalWrite(PWM_G, g);
  digitalWrite(PWM_B, b);
}
void ledSet_Pmos(byte r, byte g, byte b) { 
  analogWrite(PWM_R, MAX_BYTE-r);
  analogWrite(PWM_G, MAX_BYTE-g);
  analogWrite(PWM_B, MAX_BYTE-b);
}

/* set vibration motor */
byte speedControl(byte pressure) {
  // 85% inverted minimum
  // 24% inverted max
  byte speed = float(pressure) / 256.0 * 480;
  if(speed > 153)
    speed = 153;
  else if(speed < 51)
    speed = 51;
  // byte speed = 100;
  return speed;
}
void vibSet(byte dutyCycle) {
  // should change to pmost on pcb
  analogWrite(PWM_MOTOR, MAX_BYTE-dutyCycle);
}

/* set buzzer */
void buzzSet(byte volume, byte duration) {
  for(int i = 0; i < duration; i++) {
    analogWrite(PWM_BUZZER, MAX_BYTE-volume);
    delay(500);
    analogWrite(PWM_BUZZER, MAX_BYTE);
    delay(500);
  }
}

/* access EEPROM */
byte memSet(float val, int addr, byte mode) {
  float time = EEPROM.read(addr);
  switch (mode) {
    case 0: time = val; break; // reset
    case 1: time = (time>=val) ? time-val : 0; break; // sub
    case 2: time += val; break; // add
    default: time = 0;
  }
  EEPROM.update(addr, time);
  return time;
}

/* set stepper motor */
void stepSet(byte step) {
  // if(step > 200)
  //   step = 200;
  myStepper.step(step);
  digitalWrite(DIG_STEP_A, LOW);
  digitalWrite(DIG_STEP_B, LOW);
  digitalWrite(DIG_STEP_C, LOW);
  digitalWrite(DIG_STEP_D, LOW);
}

/* keep max of three var */
byte threeMax(byte* p1, byte* p2, byte* p3) {
  if(*p1 > *p2) {
    *p2 = 0;
    if(*p1 > *p3) {
      *p3 = 0;
    } else {
      *p1 = 0;
    }
  } else {
    *p1 = 0;
    if(*p2 > *p3) {
      *p3 = 0;
    } else {
      *p2 = 0;
    }
  }

  byte maxPressure = *p1 | *p2 | *p3;
  return maxPressure;
}

/* external hardware preset */
void extSetup(byte stepperSpeed) {
  // stepper
  myStepper.setSpeed(stepperSpeed);
  digitalWrite(DIG_STEP_A, LOW);
  digitalWrite(DIG_STEP_B, LOW);
  digitalWrite(DIG_STEP_C, LOW);
  digitalWrite(DIG_STEP_D, LOW);
  // pwm
  analogWrite(PWM_BUZZER, MAX_BYTE);
  vibSet(0);
  ledSet_Pmos(0, 0, 0);
  // controll
}

/* timer preset */
void timerSetup() {
  Wire.begin();
  myRTC.setClockMode(false);
  myRTC.setEpoch(1640995200);
  alarmDay = myRTC.getDate();
  alarmHour = myRTC.getHour(alarmH12, alarmPM);
  alarmMinute = myRTC.getMinute();
  alarmSecond = INT_FREQ; // initialize to the interval length
  alarmBits = 0b00001110; // Alarm 1 when seconds match
  alarmDayIsDay = false; // using date of month
  myRTC.turnOffAlarm(1);
  myRTC.setA1Time(
    alarmDay, alarmHour, alarmMinute, alarmSecond,
    alarmBits, alarmDayIsDay, alarmH12, alarmPM);
  myRTC.turnOnAlarm(1);
  myRTC.checkIfAlarm(1);
  alarmMinute = 0xFF;
  alarmBits = 0b01100000;
  myRTC.setA2Time(
    alarmDay, alarmHour, alarmMinute,
    alarmBits, alarmDayIsDay, alarmH12, alarmPM);
  myRTC.turnOffAlarm(2);
  myRTC.checkIfAlarm(2);
}

/* update alarm */
void alarmSet() {
  DateTime alarmDT = RTClib::now();
  // disable Alarm 1 interrupt
  myRTC.turnOffAlarm(1);
  // Clear Alarm 1 flag
  myRTC.checkIfAlarm(1);
  // extract the DateTime values as a timestamp 
  uint32_t nextAlarm = alarmDT.unixtime();
  // add the INT_FREQ number of seconds
  nextAlarm += INT_FREQ;
  // update the DateTime with the new timestamp
  alarmDT = DateTime(nextAlarm);

  // upload the new time to Alarm 1
  myRTC.setA1Time(
    alarmDT.day(), alarmDT.hour(), alarmDT.minute(), alarmDT.second(),
    alarmBits, alarmDayIsDay, alarmH12, alarmPM);
  
  // enable Alarm 1 interrupts
  myRTC.turnOnAlarm(1);
  // clear Alarm 1 flag again after enabling interrupts
  myRTC.checkIfAlarm(1);
  // attachInterrupt(digitalPinToInterrupt(INT0), wakeUp, FALLING);
  buz_treat_flag = false;
}

/* goes to sleep */
void sleepSet(){
  sleep_enable();
  // attachInterrupt(digitalPinToInterrupt(INT0), wakeUp, FALLING);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ledSet(HIGH, HIGH, HIGH);
  delay(500);
  do{
    sleep_cpu();
  } while (tick != 1);
  sleep_disable();
  // SLEEP POINT
  WDTCSR &= ~(_BV(WDIE));
  remTime = memSet(sessionTime, TIME_ADDR, 0);
  WDTCSR |= _BV(WDIE);
  // alarmSet();
  tick = 0;
  buz_treat_flag = false;
}

/* IRQ handler */
void wakeUp() {
  if(buz_treat_flag)
    sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INT0));
  tick = 1;
}

void wdSet() {
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  WDTCSR |= _BV(WDIE);
}

ISR(WDT_vect)
{
  if(f_wdt < 7) {
    f_wdt ++;
  } else {
    tick = 1;
    f_wdt = 0;
  }
}

#include <EEPROM.h>
#include <Stepper.h>
#include <avr/sleep.h>
#include <DS3231.h>

/********************
    arduino testing
    ver 0.5
********************/

/* pin def */
#define RST_N 1
#define INT0 3
#define RST0 4
// builtin crystal for arduino
// #define CRYS_1 9
// #define CRYS_2 10
#define PWM_R 5
#define PWM_G 11
#define PWM_B 12
#define PWM_MOTOR 15
#define PWM_BUZZER 16
#define DIG_STEP_A 2
#define DIG_STEP_B 13
#define DIG_STEP_C 6
#define DIG_STEP_D 14
#define PRESSURE_3 23
#define PRESSURE_2 24
#define PRESSURE_1 25
#define SDA 27
#define SCL 28
// #define MOSI 17
// #define MISO 18
// #define SCK 19

/* func prot */
void ledSet(byte r, byte g, byte b);
void vibSet(byte dutyCycle);
byte memSet(byte val, int addr, byte mode);
void alarmSet();
void buzzSet(byte volume, byte duration);
void stepSet();
void sleepSet(int mode);
byte speedControl(int pressure);
byte timeCalculation(byte speed);

/* var def */
#define PRES_THRESHOLD 1000
#define TIME_ADDR 0
#define TIME_BRUSH_MAX 120
#define BUZZ_VOL 255
#define BUZZ_DUR 3
boolean buz_treat_flag = 0;
byte speed = 0;
byte subTime = 0;
byte remTime = 120;
const int stepsPerRevolution = 200; // subject to change
Stepper myStepper(stepsPerRevolution, DIG_STEP_A, DIG_STEP_B, DIG_STEP_C, DIG_STEP_D);
DS3231 rtc(SDA, SCL);

void setup() {
    // Serial.begin(9600);
    pinMode(RST_N, INPUT_PULLUP);
    pinMode(INT0, INPUT_PULLUP);
    pinMode(RST0, INPUT_PULLUP);
    pinMode(PRESSURE_1, INPUT_PULLUP);
    pinMode(PRESSURE_2, INPUT_PULLUP);
    pinMode(PRESSURE_3, INPUT_PULLUP);
    pinMode(PWM_R, OUTPUT);
    pinMode(PWM_G, OUTPUT);
    pinMode(PWM_B, OUTPUT);
    pinMode(PWM_MOTOR, OUTPUT);
    pinMode(PWM_BUZZER, OUTPUT);
    pinMode(DIG_STEP_A, OUTPUT);
    pinMode(DIG_STEP_B, OUTPUT);
    pinMode(DIG_STEP_C, OUTPUT);
    pinMode(DIG_STEP_D, OUTPUT);

    rtc.begin();
    myStepper.setSpeed(20); // subject to change
    attachInterrupt(digitalPinToInterrupt(INT0), alarmSet, HIGH); // subject to change
    alarmSet(); // initialize here
}

void loop() { 
    /* initialization, start of IDLE state */
    // if (digitalRead(INT0) == LOW)
    //     alarmSet();
    // ledSet(255, 0, 0); // RED for IDLE

    /* fetch pressure, if detect biting, go to next state */ 
    int pressure = max(max(analogRead(PRESSURE_1),analogRead(PRESSURE_2)), analogRead(PRESSURE_3));
    while (pressure < PRES_THRESHOLD)
        pressure = max(max(analogRead(PRESSURE_1),analogRead(PRESSURE_2)), analogRead(PRESSURE_3));

    /* VIBRATION state */
    ledSet(0, 0, 255); // blue for VIBRATION
    do {
        speed = speedControl(pressure);
        vibSet(speed);
        subTime = timeCalculation(speed);
        remTime = memSet(subTime, TIME_ADDR, 1);
        delay(1000); // changing status in a secondly basis
        pressure = max(max(analogRead(PRESSURE_1),analogRead(PRESSURE_2)), analogRead(PRESSURE_3));
    } while (pressure > PRES_THRESHOLD && remTime > 0);
    vibSet(255); // pmos 255->disconnect

    /* COMPLETE state */
    if (!remTime && !buz_treat_flag) {
        ledSet(0, 255, 0); // green for COMPLETE
        buzzSet(BUZZ_VOL, BUZZ_DUR);
        stepSet();
        buz_treat_flag = 1;
        sleepSet(SLEEP_MODE_PWR_DOWN);
    }
} 

/* func def */
void ledSet(byte r, byte g, byte b) { 
    analogWrite(PWM_R, 255-r);
    analogWrite(PWM_G, 255-g);
    analogWrite(PWM_B, 255-b);
}

void vibSet(byte dutyCycle) {
    analogWrite(PWM_MOTOR, dutyCycle);
}

byte memSet(byte val, int addr, byte mode) {
    byte time = EEPROM.read(addr);
    switch (mode) {
        case 0: time = val; break; // reset
        case 1: time = (time>=val) ? time-val : 0; break; // sub
        case 2: time += val; break; // add
        default: time = 0;
    }
    EEPROM.update(addr, time);
    return time;
}

// pcb error
// void alarmSet() {
//     sleep_disable();
//     ledSet(255, 0, 0); // RED for IDLE
//     memSet(TIME_BRUSH_MAX, TIME_ADDR, 0);
//     rtc.setAlarm(0b1100);
//     sleepSet(SLEEP_MODE_ADC);
// }

void buzzSet(byte volume, byte duration) {
    for (int i = 0; i < duration; i ++) {
        analogWrite(PWM_BUZZER, volume);
        delay(500);  
        analogWrite(PWM_BUZZER, 0);
        delay(50);  
    }
}

void stepSet() {
    myStepper.step(stepsPerRevolution);
}

void sleepSet(int mode) {
    sleep_enable();
    set_sleep_mode(mode);
    sleep_cpu();
}

byte speedControl(int pressure) {
    byte speed = 127;
    // ...
    // pressure - speed calculation
    // ...
    return speed;
}

byte timeCalculation(byte speed) {
    byte time = speed / 127;
    // ...
    // speed - time calculation
    // ...
    return time;
}
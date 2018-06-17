#include <Arduino.h>

//set up a peristaltic pump stepper motor
// uses a A4988 driver board
#include <BasicStepperDriver.h>
#define STEPPER_DIR         4
#define STEPPER_STEP        3
#define STEPPER_ENA         2
#define STEPPER_STEP_TURN   200
#define STEPPER_MICROSTEP   2
BasicStepperDriver stepper(STEPPER_STEP_TURN, STEPPER_DIR, STEPPER_STEP, STEPPER_ENA);

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// If using software SPI (the default case):
#define OLED_MOSI   6
#define OLED_CLK    7
#define OLED_DC     8
#define OLED_CS     9
#define OLED_RESET  -1
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#include <ClickEncoder.h>
#include <TimerOne.h>

int16_t oldEncPos, encPos;
uint8_t buttonState;

#define ENCODER_A       12
#define ENCODER_B       11
#define ENCODER_BUTTON  10
#define ENCODER_STEPS   1

ClickEncoder encoder(ENCODER_A, ENCODER_B, ENCODER_BUTTON, ENCODER_STEPS);

#define THERM_RAD_PIN A0
#define THERM_FLUID_PIN A1
#define PELTIER_PIN A2
#define FAN_PIN A3

#define PUMP_RPM 200
#define PUMP_ANGLE_PER_ML -270 //deg turn of a pomp motor doses one milliliter of fluid

bool pumping = false;

int fluidTempAvg = 0; //for averaging out
int fluidTempChange = 0; // for hysteresis

#define FLUID_TEMP_RANGE 2 //dongrees science

int fluidTempWarm = 0;
int fluidTempCold = -10;

int radTempAvg = 0; //for averaging out
int fanOnRadTemp = 50;

void pump(int ml) {
    stepper.enable();
    pumping = true;
    stepper.startRotate(ml * PUMP_ANGLE_PER_ML);
}

void stopPump() {
    stepper.startBrake();
}

bool isPumpingNow() {
    return pumping;
}

void pumpLoop() {
    int wait_time = stepper.nextAction();
    if (wait_time == 0) {
        stepper.disable();
        pumping = false;
    }
}

void timerIsr() {
  encoder.service();
}

int thermToTemp(int therm) {
    //TODO
    return 0;
}

void fanLoop() {
    radTempAvg = (4 * radTempAvg + analogRead(THERM_RAD_PIN)) / 5;
    if (thermToTemp(radTempAvg) > fanOnRadTemp) {
        digitalWrite(FAN_PIN, HIGH);
    } else {
        digitalWrite(FAN_PIN, LOW);
    }
}

void setup() {
    Serial.begin(9600);

    //PUMP
    //stepper/pump initialization
    stepper.begin(PUMP_RPM, STEPPER_MICROSTEP);
    stepper.setSpeedProfile(stepper.LINEAR_SPEED);
    stepper.disable();

    //DISPLAY
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC);
    // Clear the buffer.
    // display.clearDisplay();
    display.display();

    //INPUT
    Timer1.initialize(1000);
    Timer1.attachInterrupt(timerIsr);

    encoder.setAccelerationEnabled(true);

    //set initial averages for temperature reading
    for (int i = 0; i < 4; i++) {
        fluidTempAvg += analogRead(THERM_FLUID_PIN);
        delay(10);
        radTempAvg += analogRead(THERM_RAD_PIN);
        delay(10);
    }

    fluidTempAvg /= 4;
    radTempAvg /= 4;
}

void loop() {
    // check user input
    ClickEncoder::Button b = encoder.getButton();
    if (b == ClickEncoder::Clicked) {
        if (isPumpingNow()) {
            stopPump();
        } else {
            pump(25);
        }
    }

    //stepper service loop
    pumpLoop();

    //turns on a fan, when a radiator temperature goes too high
    fanLoop();

    //turns peltiers on and off in order to keep a stable fluid temperature
    // coolerLoop();
}
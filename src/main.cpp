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


#define PUMP_RPM 60
#define PUMP_ANGLE_PER_ML 480 //deg turn of a pomp motor doses one milliliter of fluid (found experimentaly)

bool pumping = false;

int fluidTempAvg = 0; //for averaging out
int fluidTempChange = 0; // for hysteresis

#define FLUID_TEMP_RANGE 2 //dongrees science

int fluidTempWarm = 0;
int fluidTempCold = -10;

int radTempAvg = 0; //for averaging out
int fanOnRadTemp = 50;

int frame = 0; //loop cycle counter (mod 50)

void pump(int ml) {
    stepper.enable();
    pumping = true;
    stepper.startRotate(ml * PUMP_ANGLE_PER_ML);
    digitalWrite(LED_BUILTIN, HIGH);
}

void stopPump() {
    stepper.startBrake();
    digitalWrite(LED_BUILTIN, LOW);
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

// resistance value at 25 degrees C
#define THERM_NOMINAL 10000.0
// series resistence
#define THERM_SERIES 10000.0
// temp. for nominal resistance (almost always 25 C)
#define THERM_NOMNAL_TEMP 25.0
// The beta coefficient of the thermistor (usually 3000-4000)
#define THERM_BCOEFFICIENT 3950.0

int thermToTemp(int therm) {
    double average = 1023.0 / therm - 1;
    average = THERM_SERIES / average;
    double steinhart;
    steinhart = average / THERM_NOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= THERM_BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (THERM_NOMNAL_TEMP + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    return (int) steinhart;
}

void fanLoop() {
    if (thermToTemp(radTempAvg) > fanOnRadTemp) {
        digitalWrite(FAN_PIN, HIGH);
    } else {
        digitalWrite(FAN_PIN, LOW);
    }
}

void timerIsr() {
  encoder.service();
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
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.display();

    //INPUT
    Timer1.initialize(1000);
    Timer1.attachInterrupt(timerIsr);

    encoder.setAccelerationEnabled(true);

    //other pins - therm/fan/peltier
// #define THERM_RAD_PIN A0
// #define THERM_FLUID_PIN A1
// #define PELTIER_PIN A2
// #define FAN_PIN A3

    pinMode(THERM_RAD_PIN, INPUT);
    pinMode(THERM_FLUID_PIN, INPUT);
    pinMode(PELTIER_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);

    //debugging
    pinMode(LED_BUILTIN, OUTPUT);

    //set initial averages for temperature reading
    for (int i = 0; i < 4; i++) {
        fluidTempAvg += analogRead(THERM_FLUID_PIN);
        delay(10);
        radTempAvg += analogRead(THERM_RAD_PIN);
        delay(10);
    }

    fluidTempAvg /= 4;
    radTempAvg /= 4;

    digitalWrite(FAN_PIN, HIGH); //fan always on
}

void screenLoop() {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("radiator");
    display.print(radTempAvg);
    display.print(" -> ");
    display.println(thermToTemp(radTempAvg));
    display.println("fluid");
    display.print(fluidTempAvg);
    display.print(" -> ");
    display.println(thermToTemp(fluidTempAvg));
    display.display();
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

    pumpLoop();

    switch(frame) {
        case 0: //read radiator temperature
            radTempAvg = (4 * radTempAvg + analogRead(THERM_RAD_PIN)) / 5;
            break;
        case 10: //read fluid temperature
            fluidTempAvg = (4 * fluidTempAvg + analogRead(THERM_FLUID_PIN)) / 5;
            break;
        // case 20: //turns on a fan, when a radiator temperature goes too high
        //     // fanLoop();
            
        //     break;
        case 30: //turns peltiers on and off in order to keep a stable fluid temperature
            // coolerLoop();
            if (isPumpingNow()) {
                digitalWrite(PELTIER_PIN, HIGH);
            } else if (radTempAvg > 160) { //overtemp protection
                digitalWrite(PELTIER_PIN, HIGH);
            } else {
                digitalWrite(PELTIER_PIN, LOW);
            }
            break;
        case 99: //update screen
            screenLoop();
            frame = -1;
            break;
    }
    frame++;
}
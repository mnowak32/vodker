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

#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#define LED_PIN 1
#define LED_COUNT 12

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> ring(LED_COUNT, LED_PIN);
RgbColor fullPix(10, 100, 10), blackPix(0, 0, 0);
NeoPixelAnimator anim(LED_COUNT, NEO_CENTISECONDS);

#define THERM_RAD_PIN A0
#define THERM_FLUID_PIN A1
#define PELTIER_PIN A2
#define FAN_PIN A3


#define PUMP_RPM 60
#define PUMP_ANGLE_PER_ML 480 //deg turn of a pomp motor doses one milliliter of fluid (found experimentaly)


bool pumping = false;
int blinking = 0;
int pulseCounter = 0;

int fluidTempAvg = 0; //for averaging out
int fluidTempChange = 0; // for hysteresis

#define FLUID_TEMP_RANGE 2 //dongrees science

int fluidTempWarm = 0;
int fluidTempCold = -10;

int radTempAvg = 0; //for averaging out
int fanOnRadTemp = 50;

int frame = 0; //loop cycle counter (mod 50)

long stepperTotalSteps = 1, stepperCompleteSteps = 0;
int pumpPerc = 0;

void pump(int ml) {
    stepper.enable();
    stepper.startRotate(ml * PUMP_ANGLE_PER_ML);
    stepperTotalSteps = stepper.getStepsRemaining();
    pumping = true;
}

void stopPump() {
    stepper.startBrake();
}

bool isPumpingNow() {
    return pumping;
}

void pumpLoop() {
    int wait = stepper.nextAction();
    stepperCompleteSteps = stepper.getStepsCompleted();
    pumpPerc = stepperCompleteSteps * 100 / stepperTotalSteps;
    if (wait == 0) {
        stepper.disable();
        pumping = false;
        pumpPerc = 0;
        blinking = 127;
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

void SetRandomSeed() {
    uint32_t seed;

    // random works best with a seed that can use 31 bits
    // analogRead on a unconnected pin tends toward less than four bits
    seed = analogRead(8);
    delay(1);

    for (int shifts = 3; shifts < 31; shifts += 3)
    {
        seed ^= analogRead(8) << shifts;
        delay(1);
    }

    // Serial.println(seed);
    randomSeed(seed);
}

void setup() {
    //PUMP
    //stepper/pump initialization
    stepper.begin(PUMP_RPM, STEPPER_MICROSTEP);
    stepper.setEnableActiveState(LOW);
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

    ring.Begin();

    ring.SetPixelColor(0, RgbColor(100, 20, 30));
    ring.SetPixelColor(3, RgbColor(30, 20, 100));
    ring.SetPixelColor(7, RgbColor(30, 100, 20));
    ring.SetPixelColor(9, RgbColor(60, 20, 60));
    ring.Show();

    delay(1000);
    ring.ClearTo(blackPix);
    ring.Show();

    SetRandomSeed();
}

void screenLoop() {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("pump ");
    display.print(pumpPerc);
    display.println(" %");
    display.println("fluid");
    display.print(fluidTempAvg);
    display.print(" -> ");
    display.println(thermToTemp(fluidTempAvg));
    display.display();
}

void showProgress(int percent) {
    int fullLight = (percent * LED_COUNT) / 100;
    int rising = (percent * LED_COUNT) % 100;
    for(int i = 0; i < LED_COUNT; i++) {
        if (i < fullLight) {
            ring.SetPixelColor(i, fullPix);
        } else if (i == fullLight) {
            ring.SetPixelColor(i, RgbColor(rising / 10, rising, rising / 10));
        } else {
            ring.SetPixelColor(i, blackPix);
        }
    }
    ring.Show();
}


struct VodkaAnimationState
{
    RgbColor startingColor;  // the color the animation starts at
    RgbColor endingColor; // the color the animation will end at
};

VodkaAnimationState animationState[LED_COUNT];
// one entry per pixel to match the animation timing manager

void animUpdate(const AnimationParam& param) {
    // first apply an easing (curve) to the animation
    // this simulates acceleration to the effect
    float progress = NeoEase::QuadraticInOut(param.progress);

    // this gets called for each animation on every time step
    // progress will start at 0.0 and end at 1.0
    // we use the blend function on the RgbColor to mix
    // color based on the progress given to us in the animation
    RgbColor updatedColor = RgbColor::LinearBlend(
        animationState[param.index].startingColor,
        animationState[param.index].endingColor,
        progress);
    // apply the color to the strip
    ring.SetPixelColor(param.index, updatedColor);
}

RgbColor randomColor() {
    return RgbColor(random(80), random(80), random(80));
}

void newAnimState() {
    for(int i = 0; i <LED_COUNT; i++) {
        animationState[i].startingColor = animationState[i].endingColor;
        animationState[i].endingColor = randomColor();
        anim.StartAnimation(i, 200, animUpdate);
    }
}

void pulseColors() {
    if (anim.IsAnimating()) {
        // the normal loop just needs these two to run the active animations
        anim.UpdateAnimations();
        ring.Show();
    } else {
        newAnimState();
    }
}

void ledLoop() {
    digitalWrite(LED_BUILTIN, pumping ? HIGH : LOW);
    if (pumping) { //show progress
        showProgress(pumpPerc);
    // } else if (blinking > 0) {
    //     showProgress((blinking & 8) * 12);
    //     blinking--;
    } else { //slowly pulse colors
        pulseColors();
    }
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
        case 25:
            ledLoop();
            break;
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
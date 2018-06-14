#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//set up a peristaltic pump stepper motor
// uses a A4988 driver board
#include <BasicStepperDriver.h>
#define STEPPER_DIR 4
#define STEPPER_STEP 3
#define STEPPER_ENA 2
#define STEPPER_STEPS_PER_TURN 200
BasicStepperDriver stepper(STEPPER_STEPS_PER_TURN, STEPPER_DIR, STEPPER_STEP, STEPPER_ENA);

// If using software SPI (the default case):
#define OLED_MOSI   5
#define OLED_CLK    6
#define OLED_DC     7
#define OLED_CS     8
#define OLED_RESET  9
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define ANGLE_PER_ML 90 //deg turn of a pomp motor doses one milliliter of fluid

void pump(int ml) {
    stepper.startRotate(ml * ANGLE_PER_ML);
}

void stopPump() {
    stepper.startBrake();
}

void setup() {
  Serial.begin(9600);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done
  // Clear the buffer.
  display.clearDisplay();
  display.display();

}

void loop() {
    // put your main code here, to run repeatedly:
}
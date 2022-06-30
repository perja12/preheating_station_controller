/*
  Make sure all the display driver and pin connections are correct by
  editing the User_Setup.h file in the TFT_eSPI library folder.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################
*/

#include <TFT_eSPI.h> // Graphics and font library for ILI9341 driver chip
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "reflow_state_machine.h"


// The chip select pin to use for MAX31855
#define MAXCS   5

#define TFT_GREY 0x5AEB

#define SSR_PIN 15

Adafruit_MAX31855 thermocouple(MAXCS);

TFT_eSPI tft = TFT_eSPI();  // Invoke library

unsigned long nextUpdate = 375;
unsigned long nextTimeUpdate = 1000;
unsigned long plotUpdate = 1000;
unsigned long tempUpdate = 0;

unsigned long windowStartTime;
unsigned long now;

int windowSize = 2000;

void setup(void) {
    Serial.begin(115200);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    pinMode(SSR_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW);

    Serial.print("Initializing sensor...");
    if (!thermocouple.begin()) {
        Serial.println("ERROR.");
        while (1) delay(10);
    }
    Serial.println("DONE.");

    struct reflowProfile_t profile;

    profile.preheatTemperature = 150;

    profile.soakTemperatureMin = 150;
    profile.soakTemperatureMax = 160;
    profile.soakTemperatureDuration = 120*1000;

    profile.reflowTemperatureMin = 210;
    profile.reflowTemperatureMax = 220;
    profile.reflowTemperatureDuration = 80*1000;

    profile.coolTemperatureMin = 20;

    // Initialize PID control window starting time
    windowStartTime = millis();
    initReflow(windowSize);
    setReflowProfile(profile);
    startReflow();
}

double tempC;

void loop() {
    // Set "cursor" at top left corner of display (0,0) and select font 2
    // (cursor will move to next line automatically during printing with 'tft.println'
    //  or stay on the line is there is room for the text with tft.print)
    tft.setCursor(0, 0, 2);
    // Set the font colour to be white with a black background, set text size multiplier to 1
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setTextSize(2);

    //Serial.print("Internal Temp = ");
    //Serial.println(thermocouple.readInternal());

    if (millis() > tempUpdate) {
        tempUpdate = millis() + 2000;
        tempC = thermocouple.readCelsius();
        if (isnan(tempC)) {
            Serial.println("Something wrong with thermocouple!");
        }
    }

    if (true) {
        if (millis() > plotUpdate) {
            plotUpdate = millis() + 1000;
            Serial.println(tempC);
        }
        if (millis() > nextUpdate) {
            nextUpdate = millis() + 400;
            plot_temp(tempC);
            tft.setCursor(10, 220, 2);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setTextSize(1);
            tft.println(tempC);

            // PID computation and SSR control
            if (getReflowStatus() == REFLOW_STATUS_ON) {
                now = millis();
                double value = processInput(tempC);
                //Serial.print("PID value: ");
                //Serial.println(value);


                if ((now - windowStartTime) > windowSize) {
                    // Time to shift the Relay Window
                    windowStartTime += windowSize;
                }

                tft.setTextColor(TFT_PINK, TFT_BLACK);
                tft.setTextSize(1);
                tft.setTextPadding(100);

                if (value > (now - windowStartTime)) {
                    digitalWrite(SSR_PIN, HIGH);
                    tft.drawCentreString("ON", 160, 50, 2);
                } else {
                    digitalWrite(SSR_PIN, LOW);
                    tft.drawCentreString("OFF", 160, 50, 2);
                }
            } else {
                // Reflow oven process is off, ensure oven is off
                digitalWrite(SSR_PIN, LOW);
            }
        }
    } else {
        Serial.println("Error: Could not read temperature data");
    }

    if (millis() > nextTimeUpdate) {
        nextTimeUpdate = millis() + 500;
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(280, 220);
        int secs = millis() / 1000;
        tft.println(secs);
        draw_stage();
    }
}

void draw_stage() {
    String stage = "Init";
    switch(getReflowState()) {
    case REFLOW_STATE_IDLE:
        stage = "init";
        break;
    case REFLOW_STATE_PREHEAT:
        stage = "preheat";
        break;
    case REFLOW_STATE_SOAK:
        stage = "soak";
        break;
    case REFLOW_STATE_REFLOW_RAMP:
        stage = "reflow ramp";
        break;
    case REFLOW_STATE_REFLOW:
        stage = "reflow";
        break;
    case REFLOW_STATE_COOL:
        stage = "cool";
        break;
    case REFLOW_STATE_COMPLETE:
        stage = "complete";
        break;
    case REFLOW_STATE_TOO_HOT:
        stage = "too hot";
        break;
    case REFLOW_STATE_ERROR:
        stage = "error";
        break;
    default:
        stage = "init";
    }

    tft.setTextColor(TFT_PINK, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextPadding(100);
    tft.drawCentreString(stage, 160, 220, 2);
}

int last_x = 0;
int last_y = -1;

void plot_temp(float temp) {
    // plot the temperature over time
    // 2s -> 120/320 = 0.375 s/px (1 pixel is 2.7 seconds)
    // x = (elapsed_time / 2.7)
    int x = millis() / 0.375 / 1000;
    // 270/27 = 10
    //int x = 320 / (120 * 1000 /  millis());
    // 320 / (120000 / 60000) = 160
    // 320 / (120000 / 10000) =  26,7
    int y = 240 - ((240 / 250.0) * temp);
    if (last_y == -1) {
        last_y = y;
    }
    tft.drawLine(last_x, last_y, x, y, TFT_RED);
    last_x = x;
    last_y = y;
}

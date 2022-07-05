#include <Arduino.h>
#include <Servo.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>

/***
 * 
 * Pin attachement definitions
 * 
 ***/

#define LED_R                 (4)         // The pin to which the red LED component is attached
#define LED_G                 (5)         // The pin to which the green LED component is attached
#define LED_B                 (6)         // The pin to which the blue LED component is attached

#define LCC                   (A0)        // The pin to which the lower counterclockwise light sensor is attached
#define UCC                   (A1)        // The pin to which the upper counterclockwise light sensor is attached
#define UCW                   (A3)        // The pin to which the upper clockwise light sensor is attached
#define LCW                   (A2)        // The pin to which the lower clockwise light sensor is attached

#define SERVO_ALT             (2)         // The pin to which the altitude servo is attached
#define SERVO_AZ              (3)         // Tho pin to which the azimuth servo is attached

#define RE_CLK                (7)         // The pin to which the rotary encoder's CLK pin is attached
#define RE_DT                 (8)         // The pin to which the rotary encoder's DT pin is attached
#define RE_SW                 (9)         // The pin to which the rotary encoder's SW pi is attached

/***
 * 
 * Misc compile-time definitions
 * 
 ***/

#define DEBUG                             // Uncomment to enable debug printing
#define BANNER                F("\nHeliostat v0.1.0")
#define ALT_START             (90)        // Starting angle for alt servo
#define ALT_MIN               (0)         // Min legal angle for alt servo
#define ALT_MAX               (180)       // Max legal angle for alt servo
#define AZ_START              (90)        // Starting angle for az servo
#define AZ_MIN                (0)         // Min legal angle for az servo
#define AZ_MAX                (180)       // Max legal angle for az servo

#define SENSOR_AVG_MS         (50)        // millis() between light sensor readings
#define SENSOR_AVG_N          (50)        // Number of light sensor readings to average
#define SENSOR_MAX            (1023)      // The maximum value a sensor can have
#define LIGHT_ON_MS           (100)       // Number of millis() for the LED to be on in tracking mode
#define N_CORR                (10)        // Number of correction constants for each sensor
#define COUNT_INTERVAL        (SENSOR_MAX / (N_CORR - 1))     // Sensor counts per calibration bucket
#define offBucket(sData)      ((sData) / COUNT_INTERVAL)      // Which bucket a sensor count falls into
#define OUR_SIG               (0x4F2A)    // Some rando thing. Used to identify the EEPROM data is ours

const uint8_t sensorPin[4] = {LCC, UCC, UCW, LCW};
enum sensorid_t : uint8_t {lcc, ucc, ucw, lcw, SENSOR_COUNT};
enum calstate_t : uint8_t {tracking, adjAlt, adjAz, decide};
struct eepromdata_t {
  uint16_t signature;                       // Random value to serve as our "signature" to know the eeprom data is ours
  uint8_t offset[N_CORR][SENSOR_COUNT];     // The offests. I.e., corrReading = f(rawReading) = rawReading + interpolated offset
};
#define nvMemOffs(c, s)       (2 + 4 * (c) + (s)) // Offset in EEPROM of eepromdata_t x.offset[c][s]

/***
 * 
 * Globals
 * 
 ***/
Servo alt;
Servo az;
RotaryEncoder re(RE_CLK, RE_DT, RE_SW);
int altAngle = ALT_START;
int azAngle = AZ_START;
calstate_t calState = tracking;
uint16_t sensorVal[SENSOR_COUNT];
eepromdata_t nvData;                  // The data that lives in EEPROM
eepromdata_t newNvData;               // Where we accumulate data during calibration

/***
 * 
 * Print the calibration offset table
 * 
 ***/
void printCalibrationTable() {
  Serial.println(F("     lcc ucc ucw lcw"));
  String datum;
  for (uint8_t bucketIx = 0; bucketIx < N_CORR; bucketIx++) {
    datum = "  ";
    datum += bucketIx;
    Serial.print(datum.substring(datum.length() - 2));
    Serial.print(F(" | "));
    for (uint8_t sensorIx = 0; sensorIx < SENSOR_COUNT; sensorIx++) {
      datum = "   ";
      datum += nvData.offset[bucketIx][sensorIx];
      Serial.print(datum.substring(datum.length() - 3));
      Serial.print(sensorIx < (SENSOR_COUNT - 1) ? F(" ") : F("\n"));
    }
  }
}
 
/***
 * 
 * Set the RGB LED to the specified color
 * 
 ***/
//                           000   001    010   011  100      101     110    111
enum ledcolor_t : uint8_t {black, blue, green, cyan, red, magenta, yellow, white};
void setLed(ledcolor_t ledVal) {
  digitalWrite(LED_R, (ledVal & 0b100) != 0 ? HIGH : LOW);
  digitalWrite(LED_G, (ledVal & 0b010) != 0 ? HIGH : LOW);
  digitalWrite(LED_B, (ledVal & 0b001) != 0 ? HIGH : LOW);
}

/***
 * 
 * Handle rotary encoder switch clicks
 * 
 ***/
void onReSwitchClick(re_click_t sw) {
  if (sw == re_click_t::cLong) {        // Long click
    switch (calState) {
      case tracking:
        calState = adjAlt;
        newNvData = nvData;             // Start with existing cal data
        Serial.println(F("Calibraton mode. Point the sun sensor directly at the sun. Turn the knob to change the altitude.\n"
                         "Short click the knob to switch to azimuth correction. Long click to exit calibration."));
        break;
      case adjAlt:
      case adjAz:
        calState = decide;
        Serial.println(F("Exiting calibration. Long click to make calibration changes permanent; short click to discard them."));
        break;
      case decide:
        nvData = newNvData;
        #ifdef DEBUG
        Serial.println(F("Storing:"));
        printCalibrationTable();
        EEPROM.put(0, nvData);
        #else
        EEPROM.put(0, nvData);
        #endif
        calState = tracking;
        Serial.println(F("Tracking mode. Long click to enter calibration mode."));
    }
  } else {                              // Short click
    switch (calState) {
      case tracking:
        return;
      case adjAlt:
        calState = adjAz;
        Serial.println(F("Calibraton mode. Turn knob to correct azimuth. Short click to move to altitude correction. Long click to exit calibration."));
        break;
      case adjAz:
        calState = adjAlt;
        Serial.println(F("Calibraton mode. Turn knob to correct altitude. Short click to move to azimuth correction. Long click to exit calibration."));
        break;
      case decide:
        calState = tracking;
        Serial.println(F("Discarding calibration changes and switching to tracking mode."));
    }
  }
  setLed(tracking ? black : calState == adjAlt ? blue : calState == adjAz ? red : green);
}

/***
 * 
 * Handle rotary encoder steps
 * 
 ***/
void onReRotated(re_dir_t dir) {
  if (calState != adjAlt && calState != adjAz) {
    return;
  }

  int16_t step = dir == re_dir_t::cw ? +1 : -1;
  if (calState == adjAlt) {
    altAngle += step;
    altAngle = altAngle > ALT_MAX ? ALT_MAX : altAngle < ALT_MIN ? ALT_MIN : altAngle;
    alt.write(altAngle);
  } else {
    azAngle += step;
    azAngle = azAngle > AZ_MAX ? AZ_MAX : azAngle < AZ_MIN ? AZ_MIN : azAngle;
    az.write(azAngle);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(BANNER);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLed(blue);

  analogReference(EXTERNAL);    // Use the external voltage divider reference as the top of scale.

  EEPROM.get(0, nvData);
  if (nvData.signature != OUR_SIG) {
    Serial.println(F("Invalid signature. Initializing EEPROM data."));
    nvData.signature = OUR_SIG;
    for (uint8_t offsetIx = 0; offsetIx < N_CORR; offsetIx++) {
      for (uint8_t sensorIx = 0; sensorIx < SENSOR_COUNT; sensorIx++) {
        nvData.offset[offsetIx][sensorIx] = 0;
      }
    }
    EEPROM.put(0, nvData);
  }
  Serial.println(F("Calibraton: "));
  printCalibrationTable();
  
  alt.attach(SERVO_ALT);
  az.attach(SERVO_AZ);
  alt.write(altAngle);
  az.write(azAngle);

  re.begin();
  re.attachOnButton(onReSwitchClick);
  re.attachOnRotation(onReRotated);

  Serial.print(F("Mode: "));
  Serial.println(calState == tracking ? F("tracking.") : F("calibration."));
}

void loop() {
  static unsigned long lastSensorMillis = millis();
  static unsigned long lastLightOnMillis = millis();
  static uint16_t sensorSum[4] = {0, 0, 0, 0};
  static uint8_t sampleCount = 0;
  static bool blinkUpDn = true;
  re.run();


  unsigned long curMillis = millis();
  // Accumulate another sensor reading if at least one sensor reading interval has passed
  if (curMillis - lastSensorMillis >= SENSOR_AVG_MS) {
    for (uint8_t ix = 0; ix < SENSOR_COUNT; ix++) {
      sensorSum[ix] += analogRead(sensorPin[ix]);
    }
    sampleCount++;
    lastSensorMillis = curMillis;
  }
  // If we're in tracking mode and enough time has passed, turn the LED off
  if (calState == tracking && curMillis - lastLightOnMillis >= LIGHT_ON_MS) {
    setLed(black);
    lastLightOnMillis = curMillis;
  }

  // If we've accumulated enough sensor readings, deal with what we've got
  if (sampleCount >= SENSOR_AVG_N) {
    #ifdef DEBUG
    Serial.print(F("Sensors (lcc, ucc, ucw, lcw): "));
    #endif
    // Calculate the average reading value for each sensor
    for (uint8_t ix = 0; ix < SENSOR_COUNT; ix++) {
      sensorVal[ix] = sensorSum[ix] / SENSOR_AVG_N;
      sensorSum[ix] = 0;
      #ifdef DEBUG
      Serial.print(sensorVal[ix]);
      Serial.print(ix == SENSOR_COUNT - 1 ? F("\n") : F(", "));
      #endif
    }
    sampleCount = 0;

    // If we're doing calibration, figure out what the new offset values should be,
    // save them in the right slots in newNvData.offset and apply them to the sensorVal data
    if (calState == adjAlt || calState == adjAz) {
      uint8_t ixLeastVal = 0;
      for (uint8_t sensorIx = 1; sensorIx < SENSOR_COUNT; sensorIx++) {
        if (sensorVal[sensorIx] < sensorVal[ixLeastVal]) {
          ixLeastVal = sensorIx;
        }
      }
      for (uint8_t sensorIx = 0; sensorIx < SENSOR_COUNT; sensorIx++) {
        uint8_t newOff = sensorVal[sensorIx] - sensorVal[ixLeastVal];
        newNvData.offset[offBucket(sensorVal[sensorIx])][sensorIx] = newOff;
        sensorVal[sensorIx] -= newOff;
      }
    } else {  // Otherwise we're tracking. Apply the nvData.offset offsets to the sensorVal data
      for (uint8_t sensorIx = 0; sensorIx < SENSOR_COUNT; sensorIx++) {
        sensorVal[sensorIx] -= nvData.offset[offBucket(sensorVal[sensorIx])][sensorIx];
      }
    }

    // Decide on whether we would need to step the altitude and azimuth to get closer to pointing at 
    // the sun and, if so, in which direction.
    int16_t upDn = (((sensorVal[ucc] + sensorVal[ucw]) /2) - ((sensorVal[lcc] + sensorVal[lcw]) / 2));
    upDn = upDn < 0 ? +1 : upDn > 0 ? -1 : 0;
    int16_t cwCc = ((sensorVal[ucw] + sensorVal[lcw]) / 2) - ((sensorVal[ucc] + sensorVal[lcc]) / 2);
    cwCc = cwCc < 0 ? +1 : cwCc > 0 ? -1 : 0;

    // If we're trying to track the sun
    if (calState == tracking) {
      // Actually make the move that gets us closer
      altAngle += upDn;
      altAngle = altAngle > ALT_MAX ? ALT_MAX : altAngle < ALT_MIN ? ALT_MIN : altAngle;
      alt.write(altAngle);
      azAngle -= cwCc;
      azAngle = azAngle > AZ_MAX ? AZ_MAX : azAngle < AZ_MIN ? AZ_MIN : azAngle;
      az.write(azAngle);

      // Blink the LED to alternate between showing blue for up, yellow down and green for clockwise, red counterclockwise
      ledcolor_t theColor;
      if (blinkUpDn) {
        theColor = upDn > 0 ? yellow : upDn < 0 ? blue : white;
      } else {
        theColor = cwCc > 0 ? green : cwCc < 0 ? red : white;
      }
      blinkUpDn = !blinkUpDn;
      setLed(theColor);
    }
    #ifdef DEBUG
    // Show our work
    Serial.print(F("Alt: "));
    Serial.print(upDn == +1 ? F("Dn") : upDn == -1 ? F("Up") : F("Nc"));
    Serial.print(F(", Az: "));
    Serial.println(cwCc == +1 ? F("Cc") : cwCc == -1 ? F("Cw") : F("Nc"));
    #endif
  }
}
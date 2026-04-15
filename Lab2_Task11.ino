#include <Arduino_HS300x.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

// Raw sensor values
float rh = 0.0;
float tempC = 0.0;
float magMetric = 0.0;

int rVal = 0;
int gVal = 0;
int bVal = 0;
int clearVal = 0;

float baseRH = 0.0;
float baseTemp = 0.0;
float baseMag = 0.0;

int baseR = 0;
int baseG = 0;
int baseB = 0;
int baseClear = 0;


// Thresholds 
const float RH_JUMP_THRESHOLD = 3.0;// %RH rise for breath/humid air
const float TEMP_RISE_THRESHOLD = 1.0;// deg C rise for warm air
const float MAG_SHIFT_THRESHOLD = 3;// magnetic change threshold
const int CLEAR_CHANGE_THRESHOLD = 80;// light change threshold
const int COLOR_CHANGE_THRESHOLD = 60;// summed RGB change threshold

const unsigned long UPDATE_MS = 250;
const unsigned long COOLDOWN_MS = 2000;
const unsigned long BASELINE_REFRESH_MS = 5000;

unsigned long lastUpdate = 0;
unsigned long lastEventTime = 0;
unsigned long lastBaselineRefresh = 0;


void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize HS3003.");
    while (1);
  }

  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960.");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize BMI270/BMM150.");
    while (1);
  }

  delay(1000); // let sensors settle a bit
  captureBaseline();

  Serial.println("Task 11 event detector started.");
}


void captureBaseline() {
  // Humidity / temperature
  
    baseRH = HS300x.readHumidity();
    baseTemp = HS300x.readTemperature();
  

  // Magnetometer
  float mx, my, mz;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    baseMag = sqrt(mx * mx + my * my + mz * mz);
  }

  // Color / clear
  if (APDS.colorAvailable()) {
    APDS.readColor(baseR, baseG, baseB, baseClear);
  }

  lastBaselineRefresh = millis();
}

void readSensors() {
  
    rh = HS300x.readHumidity();
    tempC = HS300x.readTemperature();
  

  float mx, my, mz;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    magMetric = sqrt(mx * mx + my * my + mz * mz);
  }

  if (APDS.colorAvailable()) {
    APDS.readColor(rVal, gVal, bVal, clearVal);
  }
}

// Event classification

String classifyEvent(bool humidJump, bool tempRise, bool magShift, bool lightOrColorChange) {
  if (magShift) {
    return "MAGNETIC_DISTURBANCE_EVENT";
  }

  if (humidJump || tempRise) {
    return "BREATH_OR_WARM_AIR_EVENT";
  }

  if (lightOrColorChange) {
    return "LIGHT_OR_COLOR_CHANGE_EVENT";
  }

  return "BASELINE_NORMAL";
}

// Main loop
void loop() {
  if (millis() - lastUpdate < UPDATE_MS) return;
  lastUpdate = millis();

  readSensors();


  bool humidJump = (rh - baseRH) > RH_JUMP_THRESHOLD;
  bool tempRise = (tempC - baseTemp) > TEMP_RISE_THRESHOLD;
  bool magShift = fabs(magMetric - baseMag) > MAG_SHIFT_THRESHOLD;

  int clearDiff = abs(clearVal - baseClear);
  int colorDiff = abs(rVal - baseR) + abs(gVal - baseG) + abs(bVal - baseB);
  bool lightOrColorChange = (clearDiff > CLEAR_CHANGE_THRESHOLD) || (colorDiff > COLOR_CHANGE_THRESHOLD);


  // during cooldown, suppress repeated non-baseline event outputs
  String eventLabel = "BASELINE_NORMAL";
  bool inCooldown = (millis() - lastEventTime) < COOLDOWN_MS;

  if (!inCooldown) {
    eventLabel = classifyEvent(humidJump, tempRise, magShift, lightOrColorChange);

    if (eventLabel != "BASELINE_NORMAL") {
      lastEventTime = millis();
    }
  }

  if (!humidJump && !tempRise && !magShift && !lightOrColorChange) {
    if (millis() - lastBaselineRefresh > BASELINE_REFRESH_MS) {
      captureBaseline();
    }
  }

  Serial.print("raw,rh=");
  Serial.print(rh, 2);
  Serial.print(",temp=");
  Serial.print(tempC, 2);
  Serial.print(",mag=");
  Serial.print(magMetric, 2);
  Serial.print(",r=");
  Serial.print(rVal);
  Serial.print(",g=");
  Serial.print(gVal);
  Serial.print(",b=");
  Serial.print(bVal);
  Serial.print(",clear=");
  Serial.println(clearVal);

  Serial.print("flags,humid_jump=");
  Serial.print(humidJump ? 1 : 0);
  Serial.print(",temp_rise=");
  Serial.print(tempRise ? 1 : 0);
  Serial.print(",mag_shift=");
  Serial.print(magShift ? 1 : 0);
  Serial.print(",light_or_color_change=");
  Serial.println(lightOrColorChange ? 1 : 0);

  Serial.print("event,");
  Serial.println(eventLabel);
}

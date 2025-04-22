#include <Arduino.h>
#include <VescUart.h>
#include <esp_task_wdt.h> //watchdog timer, do we actually need this?
#include <U8g2lib.h> //display library
#include <cmath>
#include "CyclingDynamics.h"
#include <vector>

/** Initiate VescUart class */
VescUart UART;

// UART Pins
#define RXD2 16
#define TXD2 17

const int CLK = 33; //Set the CLK pin connection to the display
const int DIO = 25; //Set the DIO pin connection to the display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Bicycle and other physical variables and constants (for now, add them to tabletUI later)
    double velocity = 10.0; // Geschwindigkeit in m/s
    double mass = 75.0; // Masse des Fahrers + Rads in kg
    double CdA = 0.5; // Luftwiderstandsfläche in m²
    double Crr = 0.0033; // Rollwiderstandskoeffizient
    double slope = 0.0; // Steigung in %
    const double g = 9.81; // Erdbeschleunigung in m/s^2
    double number_poles = 18.0;
    double gear_ratio = 2.8;
    double wheel_diameter = 0.622; // 622 mm
    double chainring = 48.0;
    double sprocket = 11.0;
    CyclingDynamics cd(mass, CdA, Crr, number_poles, gear_ratio, wheel_diameter);


void setup() {

  /** Setup Serial port to display data */
  Serial.begin(115200);
    while (!Serial) {
    esp_task_wdt_reset();  // Reset the watchdog timer
  }
Serial.println("Serial started");
  
  /** Setup UART port (Serial1 on ESP32) */
  const int RX_PIN = 3;
  const int TX_PIN = 1;
  //Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  
  while (!Serial2) {
    esp_task_wdt_reset();  // Reset the watchdog timer
  }
  Serial.println("Serial2 started");
  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial2);
  
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font

}


int i = 0;
unsigned long currentTime = millis();
unsigned long newTime = millis();
unsigned long lastTime = millis();
unsigned long deltaT = 0;
double newTimeMinutes = 0.0;
double max_rpm = 20000.0;
double threshold_rpm = 500;
bool rpm_control = false;
bool verbose = true;

struct Protocol {
    double time;
    double cadence;
};

  std::vector<Protocol> cadenceTable = {
      {0, 90},
      {2, 90},   // From start to 6 min free warmup
      {4, 60},   // 2 minutes at 60 RPM, target 50W
      {6, 70},  // 2 minutes at 70 RPM
      {8, 80}, // 2 minutes at 80 RPM
      {10, 0}, // 3 minutes free cycling
      {12, 60}, // 2 minutes at 60 RPM, target 100W
      {14, 70}, // 2 minutes at 70 RPM
      {16, 80}, // 2 minutes at 80 RPM
      {18, 0}, // 3 minutes free cycling
      {20, 60}, // 2 minutes at 60 RPM, target 150W
      {22, 70}, // 2 minutes at 70 RPM
      {24, 80}, // 2 minutes at 80 RPM
      {26, 0}, // 3 minutes free cycling
      {28, 60}, // 2 minutes at 60 RPM, target 200W
      {30, 70}, // 2 minutes at 70 RPM
      {32, 80}, // 2 minutes at 80 RPM
      {34, 0} // 6 minutes free cycling
  };

double getCadenceAtTime(double queryTime, const std::vector<Protocol>& table) {
    if (table.empty()) {
        return 0.0;  // Return 0 if the table is empty
    }

    // If the query time is before the first entry, return the first cadence
    if (queryTime <= table.front().time) {
        return table.front().cadence;
    }

    // If the query time is after the last entry, return the last cadence
    if (queryTime >= table.back().time) {
        return table.back().cadence;
    }

    // Find the two closest time points
    for (size_t i = 0; i < table.size() - 1; ++i) {
        if (queryTime >= table[i].time && queryTime <= table[i + 1].time) {
            double c2 = table[i + 1].cadence;
            return c2;
        }
    }

    return 0.0;  // Fallback (should never reach here)
}


void loop() {
  i=i+1;
    if (millis() - currentTime > 1000) {
        currentTime = millis();
        Serial.print(i);
        i=0;
    }
  
  newTime = millis();
  deltaT =  newTime - lastTime;
  lastTime = newTime;
  newTimeMinutes = newTime / 60000.0;
  Serial.print("newTimeMinutes: ");
  Serial.print((int)newTimeMinutes);
  Serial.print(":");
  Serial.print((int)(newTimeMinutes * 60) % 60);
  Serial.print("    ");

  double target_cadence = getCadenceAtTime(newTimeMinutes, cadenceTable);
  double new_rpm_value = target_cadence * gear_ratio * number_poles * chainring / sprocket ;

  if (new_rpm_value > max_rpm) 
  {
    new_rpm_value = max_rpm;
  }

  //Serial.println("Waiting started");
  //delay(10);
  u8g2.clearBuffer();					// clear the internal memory

  /** Call the function getVescValues() to acquire data from VESC   */
  if ( UART.getVescValues() && deltaT > 0) 
  {
    if (verbose) 
    {
      Serial.print("\rRPM: ");
      Serial.print(UART.data.rpm);
      Serial.print(" | Input Voltage: ");
      Serial.print(UART.data.inpVoltage);
      Serial.print(" | Avg Input Current: ");
      Serial.print(UART.data.avgInputCurrent);
      Serial.print("   "); // Add some spaces to clear any leftover characters from previous prints
    }
    if (UART.data.rpm < threshold_rpm) 
    {
      UART.setBrakeCurrent(0.0);
      rpm_control = false;
    } 
    else if (UART.data.rpm >= threshold_rpm && new_rpm_value > 0) 
    { 
      if (!rpm_control) 
      {
        UART.setRPM(threshold_rpm);
        rpm_control = true;
      }
      else
      {
        if (verbose)
        {
          Serial.print("newRPM: ");
          Serial.print(new_rpm_value);

          u8g2.drawStr(0,20,("RPM" + String(UART.data.rpm)).c_str());
          u8g2.drawStr(0,10,("Target Cad" + String(target_cadence)).c_str());
          u8g2.drawStr(0,40,("target rpm " + String(new_rpm_value)).c_str());
          u8g2.drawStr(0,50,("Inp. Curr." + String(UART.data.avgInputCurrent)).c_str());
        }
        UART.setRPM(new_rpm_value);
      }
    } 
    else 
    {
      UART.setBrakeCurrent(0.0);
    }
  } 
  else
  {
    Serial.print("\r Failed to get data!");
    u8g2.drawStr(0,10,"Failed to get data!");
  }
  u8g2.sendBuffer();					// transfer internal memory to the display  
}
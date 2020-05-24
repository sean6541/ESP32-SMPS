#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <driver/mcpwm.h>

#define PIN_OUT_A 27
#define PIN_OUT_B 26
#define PIN_V 32
#define PIN_C 33

Preferences prefs;

int single_ended;
int invert_out;
int invert_vin;
int invert_cin;
int min_duty;
int max_duty;
int freq;
int max_v;
int max_c;
int target_v;
int target_c;
int duty = 0;

String wifi_ssid;
String wifi_pass;

AsyncWebServer server(80);

static int _analogRead(int pin) { // Invert ADC readings if needed
  int value = analogRead(pin);
  if((pin == PIN_V && invert_vin) || (pin == PIN_C && invert_cin)) {
    value = 4095 - value;
  }
  return value;
}

static void setDuty() { // Calculate and set duty cycle
  float f_duty = 0.0;
  if(single_ended) {
    f_duty = duty * 100.0 / 256.0; // Map duty from 0-256 to 0-100
    if(invert_out) {
      f_duty = 100.0 - f_duty; // Invert duty if needed
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, f_duty);
  } else {
    f_duty = duty * 50.0 / 256.0; // Map duty from 0-256 to 0-50
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 100.0 - f_duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, f_duty);
  }
}

void setup() {
  disableCore0WDT();
  disableCore1WDT();
  // Read configuration
  prefs.begin("a", false);
  single_ended = prefs.getInt("single_ended", 0);
  invert_out = prefs.getInt("invert_out", 0);
  invert_vin = prefs.getInt("invert_vin", 0);
  invert_cin = prefs.getInt("invert_cin", 0);
  min_duty = prefs.getInt("min_duty", 0);
  max_duty = prefs.getInt("max_duty", 0);
  freq = prefs.getInt("freq", 0);
  max_v = prefs.getInt("max_v", 0);
  max_c = prefs.getInt("max_c", 0);
  target_v = prefs.getInt("target_v", 0);
  target_c = prefs.getInt("target_c", 0);
  wifi_ssid = prefs.getString("wifi_ssid", "ESP32-SMPS");
  wifi_pass = prefs.getString("wifi_pass", "password");

  analogSetAttenuation(ADC_0db);

  // Initilize MCPWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_OUT_A);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_OUT_B);
  mcpwm_config_t mcpwm_config;
  if(single_ended) {
    mcpwm_config = {
      .frequency = freq,
      .cmpr_a = 0.0,
      .cmpr_b = 0.0,
      .duty_mode = MCPWM_DUTY_MODE_0,
      .counter_mode = MCPWM_UP_COUNTER
    };
  } else {
    mcpwm_config = {
      .frequency = freq * 2, // For push-pull frequency is twice
      .cmpr_a = 100.0,
      .cmpr_b = 0.0,
      .duty_mode = MCPWM_DUTY_MODE_0,
      .counter_mode = MCPWM_UP_DOWN_COUNTER
    };
  }
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwm_config);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);

  // PID task reads voltage and current; sets duty cycle accordingly
  xTaskCreate([] (void* pvParameters) {
    while(true) {
      int voltage = _analogRead(PIN_V);
      int current = _analogRead(PIN_C);
      if(voltage < target_v && current < target_c && duty < max_duty) {
        duty++;
        setDuty();
      } else if ((voltage > target_v || current > target_c) && duty > min_duty) {
        duty--;
        setDuty();
      }
    }
  }, "pid_task", 2048, NULL, 1, NULL);

  // Initilize WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(wifi_ssid.c_str(), wifi_pass.c_str());

  // Handle voltage/current adjustments
  server.on("/", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if(request->hasParam("voltage") || request->hasParam("current")) {
      int err_code = 200;
      String msg = "Configuration set.";
      if(request->hasParam("voltage")) {
        int voltage = request->getParam("voltage")->value().toInt();
        if (voltage > 0 && voltage <= max_v) {
          target_v = map(voltage, 0, max_v, 0, 4095); // Map set voltage to raw ADC value; reduces work within PID loop
          prefs.putInt("target_v", target_v);
        } else {
          err_code = 400;
          msg = "Error: Voltage " + String(voltage) + " is out of bounds (1-" + String(max_v) + " Volts)";
        }
      }
      if(request->hasParam("current")) {
        int current = request->getParam("current")->value().toInt();
        if (current > 0 && current <= max_c) {
          target_c = map(current, 0, max_c, 0, 4095); // Map set current to raw ADC value; reduces work within PID loop
          prefs.putInt("target_c", target_c);
        } else {
          err_code = 400;
          msg = "Error: Current " + String(current) + " is out of bounds (1-" + String(max_c) + " milliAmps)";
        }
      }
      request->send(err_code, "text/plain", msg);
    } else {
      int voltage = map(_analogRead(PIN_V), 0, 4095, 0, max_v);
      int current = map(_analogRead(PIN_C), 0, 4095, 0, max_c);
      request->send(200, "text/plain", "Voltage: " + String(voltage) + " Volts\nCurrent: " + String(current) + " milliAmps");
    }
  });

  // Dump configuration
  server.on("/config", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/plain",
      "single_ended: " + String(single_ended)
      + "\ninvert_out: " + String(invert_out)
      + "\ninvert_vin: " + String(invert_vin)
      + "\ninvert_cin: " + String(invert_cin)
      + "\nmin_duty: " + String(min_duty)
      + "\nmax_duty: " + String(max_duty)
      + "\nfreq: " + String(freq)
      + "\nmax_v: " + String(max_v)
      + "\nmax_c: " + String(max_c)
      + "\ntarget_v: " + String(target_v)
      + "\ntarget_c: " + String(target_c)
      + "\nwifi_ssid: " + wifi_ssid
      + "\nwifi_pass: " + wifi_pass
  );
  });

  // Set configuration
  server.on("/config", HTTP_POST, [] (AsyncWebServerRequest *request) {
    bool reboot = false;
    if(request->hasParam("single_ended", true)) {
      single_ended = request->getParam("single_ended", true)->value().toInt();
      prefs.putInt("single_ended", single_ended);
      reboot = true;
    }
    if(request->hasParam("invert_out", true)) {
      invert_out = request->getParam("invert_out", true)->value().toInt();
      prefs.putInt("invert_out", invert_out);
    }
    if(request->hasParam("invert_vin", true)) {
      invert_vin = request->getParam("invert_vin", true)->value().toInt();
      prefs.putInt("invert_vin", invert_vin);
    }
    if(request->hasParam("invert_cin", true)) {
      invert_cin = request->getParam("invert_cin", true)->value().toInt();
      prefs.putInt("invert_cin", invert_cin);
    }
    if(request->hasParam("min_duty", true)) {
      min_duty = request->getParam("min_duty", true)->value().toInt();
      prefs.putInt("min_duty", min_duty);
    }
    if(request->hasParam("max_duty", true)) {
      max_duty = request->getParam("max_duty", true)->value().toInt();
      prefs.putInt("max_duty", max_duty);
    }
    if(request->hasParam("freq", true)) {
      freq = request->getParam("freq", true)->value().toInt();
      prefs.putInt("freq", freq);
      reboot = true;
    }
    if(request->hasParam("max_v", true)) {
      max_v = request->getParam("max_v", true)->value().toInt();
      prefs.putInt("max_v", max_v);
    }
    if(request->hasParam("max_c", true)) {
      max_c = request->getParam("max_c", true)->value().toInt();
      prefs.putInt("max_c", max_c);
    }
    if(request->hasParam("target_v", true)) {
      target_v = request->getParam("target_v", true)->value().toInt();
      prefs.putInt("target_v", target_v);
    }
    if(request->hasParam("target_c", true)) {
      target_c = request->getParam("target_c", true)->value().toInt();
      prefs.putInt("target_c", target_c);
    }
    if(request->hasParam("wifi_ssid", true)) {
      wifi_ssid = request->getParam("wifi_ssid", true)->value();
      prefs.putString("wifi_ssid", wifi_ssid);
      reboot = true;
    }
    if(request->hasParam("wifi_pass", true)) {
      wifi_pass = request->getParam("wifi_pass", true)->value();
      prefs.putString("wifi_pass", wifi_pass);
      reboot = true;
    }
    if(reboot) {
      request->send(200, "text/plain", "Configuration set. Rebooting...");
      ESP.restart();
    } else {
      request->send(200, "text/plain", "Configuration set.");
    }
  });

  server.begin();
  vTaskDelete(NULL);
}

void loop() {}

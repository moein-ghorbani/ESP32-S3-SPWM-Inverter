/*
  ESP32 SPWM Inverter Full Version (Web + Monitoring + Protections)
  ---------------------------------------------------------------
  Features:
  - SPWM 50Hz from 12V DC to H-Bridge
  - Carrier 20kHz with Dead-Time
  - Soft-start ramp
  - ADC monitoring: Voltage & Current
  - OverVoltage / OverCurrent protection with latch
  - Web interface (AP mode)
    - Live display of V/I, amplitude factor, Fault
    - Reset fault
    - Set Soft-start time & Target amplitude
  ---------------------------------------------------------------
  WARNING: High voltage on H-Bridge output. Use proper protection.
*/

#include <Arduino.h>
#include "driver/mcpwm.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ----------------- CONFIG -----------------
const float OUTPUT_FREQ = 50.0f;
const int SINE_SAMPLES = 256;
const int CARRIER_FREQ = 20000;
const float DEAD_TIME_US = 2.0f;  // dead-time microseconds

// MCPWM pins
const int PIN_MCPWM_A = 25;
const int PIN_MCPWM_B = 26;

// ADC pins
const int PIN_VSENSE = 34;
const int PIN_ISENSE = 35;

// Fault reset pin
const int PIN_FAULT_RESET = 4;
const int PIN_LED_STATUS = 2;

// WiFi AP
const char* ssid = "ESP32_INVERTER";
const char* password = "12345678";
AsyncWebServer server(80);

// ----------------- ADC Scaling -----------------
const float ADC_REF_VOLTS = 3.3f;
const int ADC_RESOLUTION = 4095;
const float VOLTAGE_DIVIDER_RATIO = 330.0f/3.3f;
const float CURRENT_SENSE_V_PER_A = 0.1f;

// Protections
const float OV_BUS_V = 380.0f;
const float OC_I_RMS = 20.0f;

// Soft-start
float SOFT_START_TIME_SEC = 3.0f;
float targetAmplitude = 1.0f; // normalized

// ----------------- GLOBALS -----------------
volatile bool faultLatched = false;
volatile bool updateDutyFlag = false;

uint16_t sineTable[SINE_SAMPLES];
float amplitudeFactor = 0.0f;

// Timer
hw_timer_t *sampleTimer = NULL;
double sampleRateHz = SINE_SAMPLES * OUTPUT_FREQ;

// RMS accumulation
volatile double v_acc_sq = 0.0;
volatile double i_acc_sq = 0.0;
uint32_t lastRmsTime = 0;

// ----------------- FUNCTIONS -----------------

void buildSineTable() {
  for(int i=0;i<SINE_SAMPLES;i++){
    float angle=(2.0*PI*i)/SINE_SAMPLES;
    sineTable[i]=(uint16_t)((sin(angle))*10000.0f);
  }
}

float readVoltage(){
  int raw=analogRead(PIN_VSENSE);
  float v=(float)raw*(ADC_REF_VOLTS/ADC_RESOLUTION);
  return v*VOLTAGE_DIVIDER_RATIO;
}

float readCurrent(){
  int raw=analogRead(PIN_ISENSE);
  float v=(float)raw*(ADC_REF_VOLTS/ADC_RESOLUTION);
  return v/CURRENT_SENSE_V_PER_A;
}

void enablePWMOutputs(bool enable){
  if(enable) mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  else{
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,0.0);
  }
}

void tripFault(const char* reason){
  faultLatched=true;
  enablePWMOutputs(false);
  digitalWrite(PIN_LED_STATUS,HIGH);
  Serial.print("FAULT: "); Serial.println(reason);
}

void resetFault(){
  if(!faultLatched) return;
  Serial.println("Resetting fault");
  faultLatched=false;
  digitalWrite(PIN_LED_STATUS,LOW);
  amplitudeFactor=0.0f;
  enablePWMOutputs(true);
}

void IRAM_ATTR sampleTimerISR(){
  static int idx=0;
  if(faultLatched){
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,0.0);
    return;
  }
  int32_t s=(int32_t)sineTable[idx];
  float norm=(float)s/10000.0f;
  float mod=(norm*0.5+0.5)*amplitudeFactor*100.0f;
  if(mod<0.0) mod=0.0; if(mod>100.0) mod=100.0;
  float dutyA=mod, dutyB=100.0-mod;
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_A,dutyA);
  mcpwm_set_duty(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_OPR_B,dutyB);
  idx++; if(idx>=SINE_SAMPLES) idx=0;
  updateDutyFlag=true;
}

void mcpwmSetup(){
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0A,PIN_MCPWM_A);
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0B,PIN_MCPWM_B);
  mcpwm_config_t pwm_config;
  pwm_config.frequency=CARRIER_FREQ;
  pwm_config.cmpr_a=0.0;
  pwm_config.cmpr_b=0.0;
  pwm_config.counter_mode=MCPWM_UP_COUNTER;
  pwm_config.duty_mode=MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_0,&pwm_config);
  uint32_t dead_ticks=(uint32_t)(DEAD_TIME_US*80.0+0.5);
#ifdef CONFIG_IDF_TARGET_ESP32
  mcpwm_deadtime_enable(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,dead_ticks,dead_ticks);
#endif
  enablePWMOutputs(true);
}

// ----------------- Web -----------------
void setupWebServer(){
  WiFi.softAP(ssid,password);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/",HTTP_GET,[](AsyncWebServerRequest *request){
    String html="<!DOCTYPE html><html><head><title>ESP32 Inverter</title></head><body>";
    html+="<h2>ESP32 SPWM Inverter</h2>";
    html+="<p>Bus Voltage: "+String(readVoltage(),1)+" V</p>";
    html+="<p>Load Current: "+String(readCurrent(),2)+" A</p>";
    html+="<p>Amplitude Factor: "+String(amplitudeFactor,2)+"</p>";
    html+="<p>Fault: "+String(faultLatched?"YES":"NO")+"</p>";
    html+="<form action='/reset' method='get'><button type='submit'>Reset Fault</button></form>";
    html+="<form action='/set' method='get'>SoftStart(s): <input type='number' step='0.1' name='ss' value='"+String(SOFT_START_TIME_SEC)+"'>";
    html+=" Target Amp (0-1): <input type='number' step='0.01' name='amp' value='"+String(targetAmplitude)+"'>";
    html+="<button type='submit'>Set</button></form>";
    html+="</body></html>";
    request->send(200,"text/html",html);
  });

  server.on("/reset",HTTP_GET,[](AsyncWebServerRequest *request){
    resetFault();
    request->redirect("/");
  });

  server.on("/set",HTTP_GET,[](AsyncWebServerRequest *request){
    if(request->hasParam("ss")) SOFT_START_TIME_SEC=request->getParam("ss")->value().toFloat();
    if(request->hasParam("amp")) targetAmplitude=request->getParam("amp")->value().toFloat();
    request->redirect("/");
  });

  server.begin();
  Serial.println("Web server started");
}

// ----------------- Protections -----------------
void checkProtections(){
  float vbus=readVoltage();
  float iinst=readCurrent();
  static uint32_t lastAccTime=millis();
  uint32_t now=millis();
  float dt=(now-lastAccTime)/1000.0f;
  lastAccTime=now;
  v_acc_sq+=vbus*vbus*dt;
  i_acc_sq+=iinst*iinst*dt;
  if(now-lastRmsTime>=500){
    float T=(now-lastRmsTime)/1000.0f;
    float v_rms=sqrt(v_acc_sq/T);
    float i_rms=sqrt(i_acc_sq/T);
    v_acc_sq=0;i_acc_sq=0;
    lastRmsTime=now;
    Serial.printf("V_rms: %.2f V  I_rms: %.2f A  amp: %.3f\n",v_rms,i_rms,amplitudeFactor);
    if(v_rms>OV_BUS_V) tripFault("OV Bus");
    if(i_rms>OC_I_RMS) tripFault("OC RMS");
  }
  if(vbus>OV_BUS_V*1.2) tripFault("Bus Catastrophic");
}

// ----------------- SETUP -----------------
void setup(){
  Serial.begin(115200);
  delay(100);
  pinMode(PIN_FAULT_RESET,INPUT_PULLUP);
  pinMode(PIN_LED_STATUS,OUTPUT); digitalWrite(PIN_LED_STATUS,LOW);
  buildSineTable();
  mcpwmSetup();
  analogReadResolution(12);

  // Timer
  sampleRateHz=SINE_SAMPLES*OUTPUT_FREQ;
  double period_us=1e6/sampleRateHz;
  uint32_t alarm_us=(uint32_t)(period_us+0.5);
  sampleTimer=timerBegin(0,80,true);
  timerAttachInterrupt(sampleTimer,&sampleTimerISR,true);
  timerAlarmWrite(sampleTimer,alarm_us,true);
  timerAlarmEnable(sampleTimer);

  lastRmsTime=millis();

  // Web
  setupWebServer();

  Serial.println("System ready");
}

// ----------------- LOOP -----------------
void loop(){
  static uint32_t ssStart=millis();
  static bool ssActive=true;
  if(!faultLatched && ssActive){
    uint32_t now=millis();
    float elapsed=(now-ssStart)/1000.0f;
    if(elapsed>=SOFT_START_TIME_SEC){ amplitudeFactor=targetAmplitude; ssActive=false; }
    else amplitudeFactor=(elapsed/SOFT_START_TIME_SEC)*targetAmplitude;
  }
  // Reset button
  if(digitalRead(PIN_FAULT_RESET)==LOW){ delay(50); if(digitalRead(PIN_FAULT_RESET)==LOW){ resetFault(); delay(300); } }
  // Protections
  checkProtections();
  delay(20);
}
// Units
const double  M = 1000000.0;
const double  k = 1000.0;
const double  kHz = 1000.0;
const double  s = 1.0;
const double  K = 1.0;
const double  C = 1.0;
const double  n = 0.000000001;
const double  A = 1.0;
const double  mA = 0.001;
const double  PERCENT = 1.0;
const int     TO_MILLISECONDS = 1000;
const long    TO_MICROSECONDS = 1000000;
const double  TO_KELVIN = 273.15;
const double  TO_CELSIUS = -TO_KELVIN;

// Constants
const bool    OFF = HIGH; // Fan ONOFF control
const bool    ON = LOW;   // Fan ONOFF control
const bool    HIGH_SPEED = LOW; // Fan speed control
const bool    LOW_SPEED = HIGH; // Fan speed control
const bool    NO_FAILURE_DETECTED = HIGH; // Failsafe control
const bool    FAILURE_DETECTED = LOW;     // Failsafe control
const double  STATE_MACHINE_TICK = 1.0*s;
const double  THERMISTOR_TIME_CONSTANT = 10*k * 1*n;
const double  INTERNAL_THERMISTOR_B_CONSTANT = 3455*K;
const double  EXTERNAL_THERMISTOR_B_CONSTANT = 3455*K; // TODO: Empirically work this out
const bool    ENABLE_DIAGNOSTICS = true;
const double  DIAGNOSTICS_PRINT_INTERVAL = 1.0*s;
const double  FAN_CONTROLLER_SAMPLE_INTERVAL = 1.0*s;
const double  RELAY_CURRENT = 10*mA;  // TODO: Empirically work this out
const bool    ENABLE_EXTERNAL_TEMPERATURE_SENSOR = false;
const bool    ENABLE_PCB_TEMPERATURE_SENSOR = true;
const bool    ENABLE_DIGITAL_FAN_SENSOR = true;
const bool    ENABLE_ANALOG_FAN_SENSOR = false;
const long    BAUD = 57600;

// Analog Inputs
const int     PSU_3V3 = A0;
const int     EXTERNAL_THERMISTOR = A1;
const int     INTERNAL_THERMISTOR = A2;
const int     CURRENT = A3;
const int     PSU_12V = A4;
const int     PSU_VIN = A5;
const int     PWM_ADC = A6;
const int     PSU_5V0 = A7;

// Digital Outputs
const int     INTERNAL_TEMP_ENABLE = 7;
const int     EXTERNAL_TEMP_ENABLE = 8;
const int     FAILSAFE_ENABLE = 5;
const int     ONOFF_ENABLE = 12;
const int     FAN_SPEED = 9;
const int     LED_D13 = 13;

// Digital Inputs
const int     FAN_SPEED_D = 2;
const int     FAN_SPEED_A = 3;

// Variables
double blink_period;

// Type definitions
typedef void* State ();

// Functions

void blink(int pin, double period) {
  int p = period*TO_MILLISECONDS;
  digitalWrite(pin,millis()%p<p/2);
}

void settling_time(double tau) {
  const int NUM_TAU = 5;
  delay(NUM_TAU*tau*TO_MILLISECONDS+1);
}

double voltage_raw(int pin) {
  const double reference_voltage = 5.0;
  const double adc_max = 1024;
  return analogRead(pin)*reference_voltage/adc_max;
}

double voltage_scaled(int pin) {
  const double Rt = 1*M;
  const double Rb = 100*k;
  return voltage_raw(pin)*(Rt+Rb)/Rb;
}

double voltage_percent(int pin) {
  const double adc_max = 1024;
  return voltage_raw(pin)*100.0/adc_max;
}

double thermistor(int enable_pin, int adc_pin, int ref_pin) {
  const double pullup = 10*k;
  double vref = voltage_scaled(ref_pin);
  digitalWrite(enable_pin, HIGH);
  settling_time(THERMISTOR_TIME_CONSTANT);
  double thermistor_resistance = pullup/(vref/voltage_raw(adc_pin)-1);
  digitalWrite(enable_pin, LOW);
  return thermistor_resistance;
}

double temperature(int enable_pin, int adc_pin, int ref_pin, double Bconst) {
  const double Rref = 10*k;
  const double Tref = 25.0*C+TO_KELVIN;
  double r = thermistor(enable_pin, adc_pin, ref_pin);
  return Tref*Bconst/(Bconst+Tref*log(r/Rref))+TO_CELSIUS;
}

double pcb_temperature() {
  return temperature(
    INTERNAL_TEMP_ENABLE,
    INTERNAL_THERMISTOR,
    PSU_5V0,
    INTERNAL_THERMISTOR_B_CONSTANT);
}

double engine_temperature() {
  return temperature(
    EXTERNAL_TEMP_ENABLE,
    EXTERNAL_THERMISTOR,
    PSU_5V0,
    EXTERNAL_THERMISTOR_B_CONSTANT);
}

double switch_current() {
  return voltage_raw(CURRENT);
}

double fan_speed_analogue() {
  return voltage_percent(PWM_ADC);
}

// Class definitions

class Timer {
public:
  Timer(double interval) : 
    _interval_ms(interval*TO_MILLISECONDS),
    _next(0),
    _elapsed(false) {
  }

  bool is_elapsed() {
    if(millis()>_next) {
      _next += _interval_ms;
      _elapsed = true;
    }
    bool rtnval = _elapsed;
    _elapsed = false;
    return rtnval;
  }

private:
  int _interval_ms;
  long _next; 
  bool _elapsed;
};

class PwmCounter {
public:
  PwmCounter(const unsigned pin, const double sample_time) :
    _pin(pin),
    _sample_us(sample_time*TO_MICROSECONDS),
    _high_count(0),
    _total_count(0),
    _percent(0),
    _last_reset(0) {
  }

  void update() {
    if(digitalRead(_pin))
      _high_count++;
    _total_count++;
    unsigned long now = micros();
    if(now-_last_reset<_sample_us)
      return;
    _percent = 100.0*_high_count/_total_count;
    _last_reset = now;
    _high_count = 0;
    _total_count = 0;
  }

  operator double() {
    update();
    return _percent;
  }

private:
  const unsigned _pin;
  const unsigned _sample_us;
  unsigned _high_count;
  unsigned _total_count;
  double _percent;
  unsigned long _last_reset;
};

// Objects

State* state;
Timer state_machine_timer(STATE_MACHINE_TICK);
Timer diagnostics_timer(DIAGNOSTICS_PRINT_INTERVAL);
PwmCounter fan_speed_digital(FAN_SPEED_D, FAN_CONTROLLER_SAMPLE_INTERVAL);

// States of operation

State* engine_cold() {
  blink_period = 1.0*s;
  digitalWrite(FAILSAFE_ENABLE, NO_FAILURE_DETECTED);
  digitalWrite(ONOFF_ENABLE, OFF);
  digitalWrite(FAN_SPEED, LOW_SPEED);
  double expected_current = (OFF?RELAY_CURRENT:0)+(LOW_SPEED?RELAY_CURRENT:0);

  if(ENABLE_DIGITAL_FAN_SENSOR)
    if(fan_speed_digital>50.0*PERCENT)
      return engine_warm;

  if(ENABLE_ANALOG_FAN_SENSOR)
    if(fan_speed_analogue()>50.0*PERCENT)
      return engine_warm;
    
  if(ENABLE_EXTERNAL_TEMPERATURE_SENSOR)
    if(engine_temperature()>80.0*C)
      return engine_warm;

  if(ENABLE_PCB_TEMPERATURE_SENSOR)
    if(pcb_temperature()>50.0*C)
      return engine_warm;
  
  if(switch_current()>expected_current+RELAY_CURRENT/2)
    return failure_detected;

  if(switch_current()<expected_current-RELAY_CURRENT/2)
    return failure_detected;

  return engine_cold;
}

State* engine_warm() {
  blink_period = 1.0*s;
  digitalWrite(FAILSAFE_ENABLE, NO_FAILURE_DETECTED);
  digitalWrite(ONOFF_ENABLE, ON);
  digitalWrite(FAN_SPEED, LOW_SPEED);
  double expected_current = (ON?RELAY_CURRENT:0)+(LOW_SPEED?RELAY_CURRENT:0);

  if(ENABLE_DIGITAL_FAN_SENSOR)
    if(fan_speed_digital>80.0*PERCENT)
      return engine_hot;

  if(ENABLE_ANALOG_FAN_SENSOR)
    if(fan_speed_analogue()>80.0*PERCENT)
      return engine_hot;
    
  if(ENABLE_EXTERNAL_TEMPERATURE_SENSOR)
    if(engine_temperature()>95.0*C)
      return engine_hot;

  if(ENABLE_PCB_TEMPERATURE_SENSOR)
    if(pcb_temperature()>80.0*C)
      return engine_hot;

  if(ENABLE_DIGITAL_FAN_SENSOR)
    if(fan_speed_digital<40.0*PERCENT)
      return engine_cold;

  if(ENABLE_ANALOG_FAN_SENSOR)
    if(fan_speed_analogue()<40.0*PERCENT)
      return engine_cold;

  if(ENABLE_EXTERNAL_TEMPERATURE_SENSOR)
    if(engine_temperature()<30.0*C)
      return engine_cold;

  if(ENABLE_PCB_TEMPERATURE_SENSOR)
    if(pcb_temperature()<10.0*C)
      return engine_cold;

  if(switch_current()>expected_current+RELAY_CURRENT/2)
    return failure_detected;

  if(switch_current()<expected_current-RELAY_CURRENT/2)
    return failure_detected;

  return engine_warm;
}

State* engine_hot() {
  blink_period = 1.0*s;
  digitalWrite(FAILSAFE_ENABLE, NO_FAILURE_DETECTED);
  digitalWrite(ONOFF_ENABLE, ON);
  digitalWrite(FAN_SPEED, HIGH_SPEED);
  double expected_current = (ON?RELAY_CURRENT:0)+(HIGH_SPEED?RELAY_CURRENT:0);

  if(ENABLE_DIGITAL_FAN_SENSOR)
    if(fan_speed_digital<70.0*PERCENT)
      return engine_warm;

  if(ENABLE_ANALOG_FAN_SENSOR)
    if(fan_speed_analogue()<70.0*PERCENT)
      return engine_warm;
    
  if(ENABLE_EXTERNAL_TEMPERATURE_SENSOR)
    if(engine_temperature()<90.0*C)
      return engine_warm;

  if(ENABLE_PCB_TEMPERATURE_SENSOR)
    if(pcb_temperature()<70.0*C)
      return engine_warm;

  if(switch_current()>expected_current+RELAY_CURRENT/2)
    return failure_detected;

  if(switch_current()<expected_current-RELAY_CURRENT/2)
    return failure_detected;

  return engine_hot;
}

State* failure_detected() {
  blink_period = 0.25*s;
  digitalWrite(FAILSAFE_ENABLE, FAILURE_DETECTED);
  digitalWrite(ONOFF_ENABLE, ON);
  digitalWrite(FAN_SPEED, HIGH_SPEED);
  return failure_detected;
}

// Diagnostics

void diagnostics() {
  Serial.println();
  Serial.println("PSU_3V3\t"+String(voltage_scaled(PSU_3V3)));
  Serial.println("PSU_5V0\t"+String(voltage_scaled(PSU_5V0)));
  Serial.println("PSU_VIN\t"+String(voltage_scaled(PSU_VIN)));
  Serial.println("PSU_12V\t"+String(voltage_scaled(PSU_12V)));
  Serial.println("CURRENT\t"+String(switch_current()));
  Serial.println("INTERNAL_THERMISTOR\t"
    +String(thermistor(
      INTERNAL_TEMP_ENABLE,
      INTERNAL_THERMISTOR,
      PSU_5V0)));
  Serial.println("INTERNAL_TEMPERATURE\t"+String(pcb_temperature()));
  Serial.println("EXTERNAL_THERMISTOR\t"
    +String(thermistor(
      EXTERNAL_TEMP_ENABLE,
      EXTERNAL_THERMISTOR,
      PSU_5V0)));
  Serial.println("EXTERNAL_TEMPERATURE\t"+String(engine_temperature()));
  Serial.println("FAN_SPEED_D\t"+String((double)fan_speed_digital));
  Serial.println("FAN_SPEED_A\t"+String(fan_speed_analogue()));
  if(state==engine_cold)
    Serial.println("STATE\t\tENGINE_COLD");
  else if(state==engine_warm)
    Serial.println("STATE\t\tENGINE_WARM");
  else if(state==engine_hot)
    Serial.println("STATE\t\tENGINE_HOT");
  else if(state==failure_detected)
    Serial.println("STATE\t\tFAILURE_DETECTED");
  else
    Serial.println("STATE\t\tUNKNOWN");
}

void setup() {
  pinMode(FAN_SPEED_D, INPUT);
  pinMode(FAN_SPEED_A, INPUT);
  pinMode(INTERNAL_TEMP_ENABLE, OUTPUT);
  pinMode(EXTERNAL_TEMP_ENABLE, OUTPUT);
  pinMode(FAILSAFE_ENABLE, OUTPUT);
  pinMode(ONOFF_ENABLE, OUTPUT);
  pinMode(FAN_SPEED, OUTPUT);
  pinMode(LED_D13, OUTPUT);
  state = engine_cold;
  blink_period = 1.0*s;
  if(ENABLE_DIAGNOSTICS)
    Serial.begin(BAUD);
}

void loop() {
  blink(LED_D13, blink_period);
  fan_speed_digital.update();
  if(state_machine_timer.is_elapsed())
    state = (*state)();
  if(ENABLE_DIAGNOSTICS)
    if(diagnostics_timer.is_elapsed())
      diagnostics();
}

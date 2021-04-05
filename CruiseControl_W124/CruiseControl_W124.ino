/*
 * MIT license 
 * 
 * Tested exclusively on an Arduino Nano Every
 * This code is very specific to the MB W124 series of cars
 * 
 * Atmega4809 Datasheet Reference:
 * http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173B.pdf
 * 
 * Arduino Nano Every pinout:
 * https://camo.githubusercontent.com/e9b404717587347e0bc5c555f2f3db0522f073a7/68747470733a2f2f692e696d6775722e636f6d2f6c5635424f61762e706e67
 
  Used pins Reference
 
  INPUTS
  ACCEL_BUTTON_PIN (7)      // PA1
  DECEL_BUTTON_PIN (8)      // PE3
  CANCEL_BUTTON_PIN (9)     // PB0
  RESUME_BUTTON_PIN (10)    // PB1
  BRAKE_INPUT_PIN (11)      // PE0
  SPEED_INPUT_PIN (12)      // PE1
  SERVO_POSITION_PIN (14)   // PD3

  OUTPUTS
  SERVO_CLUTCH_PIN (6)      // PF4
  SERVO_DIRECTION_A_PIN (5) // PB2
  SERVO_DIRECTION_B_PIN (4) // PC6
  SERVO_MOTOR_PWM_PIN (3)   // PF5
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <AceButton.h>
#include "PIDController.h"

using namespace ace_button;
using namespace pid;

// Uncomment to enable debug output
#define DEBUG_MODE

/*
 * RTC ticks per second
 * Assumes 32.768kHz RTC clock and /32 prescaler
 */
#define CLOCK_TICKS_PER_SECOND (1024)
/*
 * RTC PIT counter and sampled counter
 */
volatile uint32_t RTC_pit_counter = 0;
uint32_t s_RTC_pit_counter = 0;

/*
 * When using TCA clock with /64 default prescaler
 * TCB0 will reset after ~200ms if no edge is detected
 */
#if F_CPU == 20000000L
  #define TCB_TIMEOUT_VALUE 0xefff
#else 
  #define TCB_TIMEOUT_VALUE 0xbfff
#endif

/*
 * ADC sample accumulation and result bit shift value
 * Bit shift value must match SAMPNUM_ACC value 
 * 
 * The servo position feedback pot is noisy due to
 * temperature drift, vibration and long cable runs 
 * from the amp to the actuator under the hood.
 * 
 * Oversampling helps smooth out the signal.
 */
#define ADC_SAMPNUM_ACC_VALUE (ADC_SAMPNUM_ACC32_gc)
#define ADC_RES_SHIFT_DIV (5)
// 4.34V reference voltage for ADC
#define ADC_VREF (VREF_ADC0REFSEL_4V34_gc)

// Input pins for control stock
#define ACCEL_BUTTON_PIN (7)
#define DECEL_BUTTON_PIN (8)
#define CANCEL_BUTTON_PIN (9)
#define RESUME_BUTTON_PIN (10)

// Analog output pin for actuator motor
#define SERVO_MOTOR_PWM_PIN (3)

/*
 * Use RTC PIT counter as clock source 
 * for button handler and PID controllers
 */
class CruisePIDConfig: public PIDConfig {
  public:
    uint32_t getClock() override {
      return s_RTC_pit_counter;
    }
};

class CruiseButtonConfig: public ButtonConfig {
  public:
    uint32_t getClock() override {
      return s_RTC_pit_counter;
    }
  
    int readButton(uint8_t buttonPin) override {
      switch (buttonPin) {
        default:
          return 0;

        case ACCEL_BUTTON_PIN:
          return bitRead(PORTA.IN, 1);

        case DECEL_BUTTON_PIN:
          return bitRead(PORTE.IN, 3);

        case CANCEL_BUTTON_PIN:
          return bitRead(PORTB.IN, 0);

        case RESUME_BUTTON_PIN:
          return bitRead(PORTB.IN, 1);
      }
    }
};

// Sped controller config
#define SPEED_SAMPLE_INTERVAL (128) // ~100ms
#define SPEED_PID_KP  (22.0) // proportional gain (22.0)
#define SPEED_PID_KI  (0.25) // integral gain (0.25)
#define SPEED_PID_KD  (0) // derivative gain (0)
PIDController speedController;
CruisePIDConfig speedPIDConfig;

// Physical limits (arming + control, km/h)
#define MINIMUM_SPEED (50.0)
#define MAXIMUM_SPEED (150.0)

// Servo controller config
#define SERVO_SAMPLE_INTERVAL (16) // ~10ms
#define SERVO_PID_KP  (10.5) // proportional gain
#define SERVO_PID_KI  (0) // integral gain
#define SERVO_PID_KD  (0.0005) // derivative gain
PIDController servoController;
CruisePIDConfig servoPIDConfig;

// Physical limits (inversed range)
#define SERVO_MAX_POSITION (20) // full throttle
#define SERVO_MIN_POSITION (240) // idle
#define SERVO_POSITION_TOLERANCE (2) // approx. 2% position tolerance
#define SERVO_MIN_PWM_VALUE (100)

float speedScaleFactor;
float currentSpeed = 0;
float targetSpeed = 0; 
float lastTargetSpeed = 0;
float throttleTarget = 0;
float servoPower = 0;

volatile uint8_t throttlePosition;
float s_throttlePosition;

volatile uint8_t seenSpeedSamples = 0;
volatile uint16_t speedSignalPeriod = 0;

bool systemArmed = false;
bool cruiseControlEnabled = false;
bool servoEnabled = false;

AceButton accelButton, decelButton, cancelButton, resumeButton;
CruiseButtonConfig* buttonConfig;

void disableServo() {
  // SERVO_MOTOR_PWM off
  PORTF.OUT &= ~PIN5_bm;
  // SERVO_DIRECTION_B off
  PORTC.OUT &= ~PIN6_bm;
  // SERVO_DIRECTION_A off
  PORTB.OUT &= ~PIN2_bm;
  // SERVO_CLUTCH off
  PORTF.OUT &= ~PIN4_bm;
  servoEnabled = false;
}

void enableServo() {
  // SERVO_MOTOR_PWM off
  PORTF.OUT &= ~PIN5_bm;
  // SERVO_DIRECTION_B off
  PORTC.OUT &= ~PIN6_bm;
  // SERVO_DIRECTION_A off
  PORTB.OUT &= ~PIN2_bm;
  // SERVO_CLUTCH on
  PORTF.OUT |= PIN4_bm;
  servoEnabled = true;
}

void pidSwitchToAuto() {
  speedController.setMode(PIDConfig::kModeAutomatic);
  servoController.setMode(PIDConfig::kModeAutomatic);
}

void pidSwitchToManual() {
  speedController.setMode(PIDConfig::kModeManual);
  servoController.setMode(PIDConfig::kModeManual);
}

void enableCruiseControl() {
  if (!systemArmed) {
    return;
  }
  
  // LED on
  PORTE.OUT |= PIN2_bm;

  enableServo();
  pidSwitchToAuto();
  cruiseControlEnabled = true;
}

void disableCruiseControl() {
  pidSwitchToManual();
  disableServo();

  lastTargetSpeed = targetSpeed;
  targetSpeed = 0;
  throttleTarget = 0;
  cruiseControlEnabled = false;

  // LED off
  PORTE.OUT &= ~PIN2_bm;
}

void handleAccelButtonEvent(uint8_t eventType) {
  if (!systemArmed) {
    return;
  }
  
  switch (eventType) {
    case AceButton::kEventReleased:
      if (cruiseControlEnabled && (targetSpeed + 1) < MAXIMUM_SPEED) {
        targetSpeed++;
      }
      break;

    case AceButton::kEventLongPressed:
      // TODO: accelerate-until-released
      break;
      
    case AceButton::kEventLongReleased:
      targetSpeed = currentSpeed;
      enableCruiseControl();
      break;
  }
}

void handleDecelButtonEvent(uint8_t eventType) {
  if (!systemArmed) {
    return;
  }
  
  switch (eventType) {
    case AceButton::kEventReleased:
      if (cruiseControlEnabled && (targetSpeed - 1) > MINIMUM_SPEED) {
        targetSpeed--;
      }
      break;

    case AceButton::kEventLongPressed:
      // TODO: decelerate-until-released
      break;
      
    case AceButton::kEventLongReleased:
      targetSpeed = currentSpeed;
      enableCruiseControl();
      break;
  }
}

void handleCancelButtonEvent(uint8_t eventType) {
  if (!cruiseControlEnabled) {
    return;
  }
  
  if (eventType == AceButton::kEventReleased || eventType == AceButton::kEventLongReleased) {
      disableCruiseControl();
  }
}

void handleResumeButtonEvent(uint8_t eventType) {
  if (!systemArmed) {
    return;
  }
  
  if (eventType == AceButton::kEventReleased || eventType == AceButton::kEventLongReleased) {
      targetSpeed = lastTargetSpeed;
      enableCruiseControl();
  }
}

void handleButtonEvent(AceButton* button, uint8_t eventType, uint8_t /*buttonState*/) {   
  switch (button->getPin()) {
    default:
      break;

    case ACCEL_BUTTON_PIN:
      handleAccelButtonEvent(eventType);
      break;

    case DECEL_BUTTON_PIN:
      handleDecelButtonEvent(eventType);
      break;

    case CANCEL_BUTTON_PIN:
      handleCancelButtonEvent(eventType);
      break;

    case RESUME_BUTTON_PIN:
      handleResumeButtonEvent(eventType);
      break;
  }
}

void PORT_init() {
  // All ports configured as inputs by default
  PORTA.DIR = PORTB.DIR = PORTC.DIR = PORTD.DIR = PORTE.DIR = PORTF.DIR = 0;
  /*
   * Unused and input ports: 
   *  - Enable pullup resistor
   *  - Disable input buffer and interrupt
   */
  PORTA.PIN0CTRL = PORTA.PIN1CTRL = PORTA.PIN2CTRL = PORTA.PIN3CTRL = 
  PORTA.PIN4CTRL = PORTA.PIN5CTRL = PORTA.PIN6CTRL = PORTA.PIN7CTRL = 
  PORTB.PIN0CTRL = PORTB.PIN1CTRL = PORTB.PIN3CTRL = PORTB.PIN4CTRL = 
  PORTB.PIN5CTRL = PORTB.PIN6CTRL = PORTB.PIN7CTRL = PORTC.PIN0CTRL = 
  PORTC.PIN1CTRL = PORTC.PIN2CTRL = PORTC.PIN3CTRL = PORTC.PIN4CTRL = 
  PORTC.PIN5CTRL = PORTC.PIN7CTRL = PORTD.PIN0CTRL = PORTD.PIN1CTRL = 
  PORTD.PIN2CTRL = PORTD.PIN3CTRL = PORTD.PIN4CTRL = PORTD.PIN5CTRL = 
  PORTD.PIN6CTRL = PORTD.PIN7CTRL = PORTE.PIN0CTRL = PORTE.PIN1CTRL = 
  PORTE.PIN3CTRL = PORTE.PIN4CTRL = PORTE.PIN5CTRL = PORTE.PIN6CTRL = 
  PORTE.PIN7CTRL = PORTF.PIN0CTRL = PORTF.PIN1CTRL = PORTF.PIN2CTRL = 
  PORTF.PIN3CTRL = PORTF.PIN6CTRL = PORTF.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
  
  /* -------- INPUT PINS -------- */
  
  // BRAKE_INPUT PE0 (11)
  PORTE.DIR &= ~PIN0_bm;
  PORTE.PIN0CTRL |= PORT_ISC_FALLING_gc;

  // ACCEL_BUTTON PA1 (7)
  PORTA.DIR &= ~PIN1_bm;
  PORTA.PIN1CTRL |= PORT_ISC_RISING_gc;

  // DECEL_BUTTON PE3 (8)
  PORTE.DIR &= ~PIN3_bm;
  PORTE.PIN3CTRL |= PORT_ISC_RISING_gc;

  // CANCEL_BUTTON PB0 (9)
  PORTB.DIR &= ~PIN0_bm;
  PORTB.PIN0CTRL |= PORT_ISC_RISING_gc;

  // RESUME_BUTTON PB1 (10)
  PORTB.DIR &= ~PIN1_bm;
  PORTB.PIN1CTRL |= PORT_ISC_RISING_gc;
 
  // SPEED_INPUT PE1 (12)
  PORTE.DIR &= ~PIN1_bm;
  PORTE.PIN1CTRL |= PORT_ISC_RISING_gc;

  // SERVO_POSITION PD3 (14) - pot value
  PORTD.DIR &= ~PIN3_bm;

  /* -------- OUTPUT PINS -------- */
  
  PORTE.PIN2CTRL = PORTF.PIN5CTRL = PORTC.PIN6CTRL = PORTB.PIN2CTRL = 
  PORTF.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc; 
  
  PORTE.PIN2CTRL = PORTF.PIN5CTRL = PORTC.PIN6CTRL = PORTB.PIN2CTRL = 
  PORTF.PIN4CTRL &= ~PORT_PULLUPEN_bm;
  
  // Board indicator LED on PE2 (13)
  PORTE.DIR |= PIN2_bm;
  PORTE.OUT &= ~PIN2_bm;
  
  // SERVO_MOTOR_PWM
  PORTF.DIR |= PIN5_bm;
  PORTF.OUT &= ~PIN5_bm;

  // SERVO_DIRECTION_A
  PORTB.DIR |= PIN2_bm;
  PORTB.OUT &= ~PIN2_bm;

  // SERVO_DIRECTION_B
  PORTC.DIR |= PIN6_bm;
  PORTC.OUT &= ~PIN6_bm;

  // SERVO_CLUTCH
  PORTF.DIR |= PIN4_bm;
  PORTF.OUT &= ~PIN4_bm;
}

void BUTTON_init() {
  buttonConfig = (CruiseButtonConfig*) CruiseButtonConfig::getSystemButtonConfig();
  buttonConfig->setDebounceDelay(32); // ~30ms
  buttonConfig->setLongPressDelay(CLOCK_TICKS_PER_SECOND);
  buttonConfig->setFeature(CruiseButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(CruiseButtonConfig::kFeatureSuppressAfterLongPress); 
  buttonConfig->setEventHandler(handleButtonEvent);

  accelButton.init(ACCEL_BUTTON_PIN, HIGH);
  decelButton.init(DECEL_BUTTON_PIN, HIGH);
  resumeButton.init(RESUME_BUTTON_PIN, HIGH);
  cancelButton.init(CANCEL_BUTTON_PIN, HIGH);
}

void SPEED_PID_init() {
  speedPIDConfig = CruisePIDConfig();
  speedPIDConfig.setGains(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, PIDConfig::kActionDirect);
  speedPIDConfig.setSampleTime(SPEED_SAMPLE_INTERVAL);
  speedPIDConfig.setClockTicksPerSecond(CLOCK_TICKS_PER_SECOND);
  speedPIDConfig.setOutputLimits(SERVO_MAX_POSITION, SERVO_MIN_POSITION);
  speedPIDConfig.setMode(PIDConfig::kModeManual);
  
  speedController.init(&speedPIDConfig, &currentSpeed, &targetSpeed, &throttleTarget);
}

void SERVO_PID_init() {
  servoPIDConfig = CruisePIDConfig();
  servoPIDConfig.setGains(SERVO_PID_KP, SERVO_PID_KI, SERVO_PID_KD, PIDConfig::kActionDirect);
  servoPIDConfig.setSampleTime(SERVO_SAMPLE_INTERVAL);
  servoPIDConfig.setClockTicksPerSecond(CLOCK_TICKS_PER_SECOND);
  servoPIDConfig.setOutputLimits(-255, 255);
  servoPIDConfig.setMode(PIDConfig::kModeManual);
  
  servoController.init(&servoPIDConfig, &s_throttlePosition, &throttleTarget, &servoPower);
}

void EVENT_SYSTEM_init() {
  /*
   * Use event system signaling for time critical elements of the control loop:
   *  - speed pulse counting and period measurement
   *  - servo position feedback measurement
   */
  // SPEED_INPUT (PE1) as event generator on channel 4
  EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT0_PIN1_gc;
  // Connect TCB timer to event channel 4
  EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL4_gc;
  
  // RTC periodic interrupt as event generator on channel 5
  EVSYS.CHANNEL5 = EVSYS_GENERATOR_RTC_PIT0_gc;
  // Connect ADC0 to event channel 5
  EVSYS.USERADC0 = EVSYS_CHANNEL_CHANNEL5_gc;
}

void TCB0_init() {
  // Disable TCB0
  TCB0.CTRLA &= ~TCB_ENABLE_bm;
  // Preload TCB0 timeout counter
  TCB0.CCMP = TCB_TIMEOUT_VALUE;
  // Frequency measurement mode 
  TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;
  // Interrupt on capture event (rising signal edge)
  TCB0.INTCTRL = TCB_CAPT_bm;
  // Capture event, and noise cancellation filter enabled
  TCB0.EVCTRL = TCB_CAPTEI_bm | 1 << TCB_FILTER_bp;
  // Use TCA clock and enable TCB0
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
}

void RTC_init() {
  uint8_t temp;

  /*
   * Enable internal 32k oscillator
   * It's innacurate, but good enough for this job
   */
  temp = CLKCTRL.OSC32KCTRLA;
  temp &= ~CLKCTRL_ENABLE_bm;
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.OSC32KCTRLA = temp;

  // Wait for clocks to switch and settle
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) {;}
  // Wait for RTC synchronization
  while (RTC.STATUS > 0 || RTC.PITSTATUS > 0) {;}

  // Disable RTC
  RTC.CTRLA &= ~RTC_RTCEN_bm;  
  // Enable periodic interrupt
  RTC.PITINTCTRL = RTC_PI_bm;
  // Set RTC to use internal 32k oscillator
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
  // Set clock prescaler and enable RTC periodic interrupt mode
  RTC.PITCTRLA = RTC_PERIOD_CYC32_gc | RTC_PITEN_bm;
  // Enable RTC
  RTC.CTRLA |= RTC_RTCEN_bm;

  // Wait for RTC synchronization
  while (RTC.STATUS > 0 || RTC.PITSTATUS > 0) {;}
}

void ADC_init() {
  // Configure voltage reference for ADC
  VREF.CTRLA = ADC_VREF;
  /*
   * Enable ADC
   * 8-bit resolution
   * Disable free-running mode
   */
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_8BIT_gc | 0 << ADC_FREERUN_bp;
  // Accumulate samples
  ADC0.CTRLB = ADC_SAMPNUM_ACC_VALUE;
  /*
   * Reduced sampling capacitance (as VREF >= 1V)
   * Internal voltage reference
   * Configure prescaler
   */
  ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV8_gc;
  // Use auto sample delay variation to help with noise
  ADC0.CTRLD |=  1 << ADC_ASDV_bp;
  // Throttle position input on AIN3 (port PD3) as source
  ADC0.MUXPOS = ADC_MUXPOS_AIN3_gc;
  /*
   * Result ready interrupt enabled
   * Window comparator interrupt disabled
   */
  ADC0.INTCTRL = ADC_RESRDY_bm | 0 << ADC_WCMP_bp;
  // Enable event triggered conversion (fires via EVSYS on RTC PIT)
  ADC0.EVCTRL |= ADC_STARTEI_bm;
}

// RTC periodic interrupt handler
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;
  RTC_pit_counter++;
  checkButtons();
  if (servoEnabled) {
    runServo(s_throttlePosition, throttleTarget, servoPower);
  }
}

// TCB0 interrupt handler
ISR(TCB0_INT_vect) {
  // Reading TCBn.CCMP in FRQ mode automatically clears TCBn.INTFLAGS
  speedSignalPeriod += TCB0.CCMP;
  seenSpeedSamples++;
}

// ADC result ready interrupt handler
ISR(ADC0_RESRDY_vect) {
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  // Read result and byte-shift to account for accumulated samples
  throttlePosition = ADC0.RES >> ADC_RES_SHIFT_DIV;
}

// PORT(E) pin interrupt handler
ISR(PORTE_PORT_vect) {
  // Check if interrupt was triggered by brake pedal input
  if (PORTE.INTFLAGS & (PORT_INT0_bm)) {
    handleCancelButtonEvent(AceButton::kEventReleased);
  }
  PORTE.INTFLAGS |= PORT_INT0_bm;
}

uint16_t getSpeedFrequencyScaleFactor() {
  switch (TCA0.SINGLE.CTRLA >> 1) {
    default:
      return 1;
      
    case 0:
      return 2;

    case 1:
      return 4;

    case 2:
      return 8;

    case 3:
      return 16;

    case 4:
      return 32;

    case 5:
      return 128;

    case 6:
      return 512;

    case 7:
      return 2048;
  }
}

void checkButtons() {
  cancelButton.check();
  accelButton.check();
  decelButton.check();
  resumeButton.check();
}

float calculateCurrentSpeed(
  uint16_t& period, 
  uint8_t& samples
) {  
  if (period == 0 || samples == 0) {
    return 0;
  }

  float calculatedSpeed = 0;
  #if F_CPU == 20000000L
    calculatedSpeed = lroundf(1250.0/((period/samples)/speedScaleFactor));
  #else
    calculatedSpeed = lroundf(1000.0/((period/samples)/speedScaleFactor));
  #endif
  
  if (!(calculatedSpeed >= MINIMUM_SPEED && calculatedSpeed <= MAXIMUM_SPEED)) {
    return 0;
  }

  return calculatedSpeed;
}

void runServo(float fromPosition, float toPosition, float motorPWM) {
  float positionError = fromPosition - lroundf(toPosition);
  uint8_t pwmValue = max(SERVO_MIN_PWM_VALUE, (uint8_t) fabsf(motorPWM));

  if (positionError > SERVO_POSITION_TOLERANCE) {
    // Enable direction B
    PORTC.OUT |= PIN6_bm;
    // Disable direction A
    PORTB.OUT &= ~PIN2_bm;

    // Run motor
    analogWrite(SERVO_MOTOR_PWM_PIN, pwmValue);
    return;
    
  } 
  
  if (positionError < -SERVO_POSITION_TOLERANCE) {   
    // Disable direction B 
    PORTC.OUT &= ~PIN6_bm;
    // Enable direction A
    PORTB.OUT |= PIN2_bm;
   
    // Run motor
    analogWrite(SERVO_MOTOR_PWM_PIN, pwmValue);
    return;
  } 

  // Hold position
  PORTF.OUT &= ~PIN5_bm; // motor off
  PORTC.OUT |= PIN6_bm;  // B on
  PORTB.OUT |= PIN2_bm;  // A on
}

void setup() {  
  targetSpeed = 0;
  currentSpeed = 0;
  throttleTarget = 0;
  systemArmed = false;
  cruiseControlEnabled = false;
    
  speedScaleFactor = float(getSpeedFrequencyScaleFactor());
    
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ADC_init();
    RTC_init();
    EVENT_SYSTEM_init();
    TCB0_init();
    PORT_init();
    BUTTON_init();
    SPEED_PID_init();
    SERVO_PID_init();
  }

  #ifdef DEBUG_MODE
    Serial.begin(115200);
  #endif
}

void loop() {  
  /* 
   *  This is where the "magic" happens
   *  With CC enabled each loop takes 25us on average, 
   *  and 195us worst-case
   */
  static uint8_t s_seenSpeedSamples;
  static uint16_t s_speedSignalPeriod;
  static uint32_t lastSeenSpeedSample_RTC_pit_counter = 0;
  static bool checkSpeed;

  // Disable interrupts, sample volatile data, restore state
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    s_RTC_pit_counter = RTC_pit_counter; 
    s_seenSpeedSamples = seenSpeedSamples;
    s_speedSignalPeriod = speedSignalPeriod;
    s_throttlePosition = throttlePosition;
    seenSpeedSamples = 0;
    speedSignalPeriod = 0;
  }
  
  checkSpeed = s_seenSpeedSamples > 0 && s_speedSignalPeriod > 0;

  // Force set speed to zero if no speed samples seen in certain number of RTC cycles
  if ((s_RTC_pit_counter - lastSeenSpeedSample_RTC_pit_counter) > SPEED_SAMPLE_INTERVAL && !checkSpeed) {
    currentSpeed = 0;
  } else if (checkSpeed) {
    currentSpeed = calculateCurrentSpeed(s_speedSignalPeriod, s_seenSpeedSamples);
    lastSeenSpeedSample_RTC_pit_counter = s_RTC_pit_counter;
  }

  /* 
   * Arm the system only above a minimum speed.
   * In cases of automatic gearboxes, it's the minimum speed 
   * at which the gearbox switches to top gear + some margin to avoid kickdown on acceleration
   */
  systemArmed = currentSpeed >= MINIMUM_SPEED;
  
  if (!systemArmed && cruiseControlEnabled) {
    disableCruiseControl();
  } else if (cruiseControlEnabled) {
    /* 
     *  PIDs will recompute as needed
     *  Servo runs on every RTC PIT
     */
    speedController.compute();
    servoController.compute();
    

  #ifdef DEBUG_MODE
    if (s_RTC_pit_counter % 256 == 0) {
      Serial.print(systemArmed);
      Serial.print(" ");
      Serial.print(cruiseControlEnabled);
      Serial.print(" ");
      Serial.print(currentSpeed);
      Serial.print(" ");
      Serial.print(targetSpeed);
      Serial.print(" ");
      Serial.print(s_throttlePosition);
      Serial.print(" ");
      Serial.print(throttleTarget);
      Serial.print(" ");
      Serial.println(servoPower);
    }    
  #endif
  }
}

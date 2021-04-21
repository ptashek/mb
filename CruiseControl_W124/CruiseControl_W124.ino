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
#include <avr/wdt.h>
#include <util/atomic.h>
#include <AceButton.h>
#include "PIDController.h"

using namespace ace_button;
using namespace pid;

// Uncomment to enable debug output
#define DEBUG_MODE

/*
 * RTC ticks per second
 * Assumes 32kHz RTC clock and /32 prescaler
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
  #define TCB0_TIMEOUT_VALUE 0xefff
#else 
  #define TCB0_TIMEOUT_VALUE 0xbfff
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
#define SPEED_SAMPLE_INTERVAL_SHIFT (1) // ~250ms
#define SPEED_SAMPLE_INTERVAL (CLOCK_TICKS_PER_SECOND >> SPEED_SAMPLE_INTERVAL_SHIFT)
// 2km/h = 4Hz = 8 pulses / s
#define SPEED_BUMP_PULSE_COUNT (8 >> SPEED_SAMPLE_INTERVAL_SHIFT)
#define SPEED_PULSE_TOLERANCE (1)
#define SPEED_PID_KP  (8.12)// proportional gain (22.0) 8.12
#define SPEED_PID_KI  (1.3) // integral gain (0.25) 1.3
#define SPEED_PID_KD  (2.6) // derivative gain (0) 2.6
PIDController speedController;
CruisePIDConfig speedPIDConfig;

// Physical limits (arming + control, pulses/s)
#define MINIMUM_SPEED (200 >> SPEED_SAMPLE_INTERVAL_SHIFT) // 50km/h
#define MAXIMUM_SPEED (1000 >> SPEED_SAMPLE_INTERVAL_SHIFT) // 250km/h
#define THROTTLE_MIN (0)
#define THROTTLE_MAX (100)

// Servo controller config
#define SERVO_SAMPLE_INTERVAL (16) // ~15ms
#define SERVO_PID_KP  (3.9755)  // proportional gain
#define SERVO_PID_KI  (1.7989) // integral gain
#define SERVO_PID_KD  (0.36589) // derivative gain
PIDController servoController;
CruisePIDConfig servoPIDConfig;

// Physical limits
#define SERVO_POSITION_FULL (0) // full throttle
#define SERVO_POSITION_PARK (225) // park
#define SERVO_POSITION_IDLE (225) // idle
#define SERVO_POSITION_TOLERANCE (2) // approx. 2% position tolerance
#define SERVO_MIN_PWM_VALUE (115)

float currentSpeed = 0;
float targetSpeed = 0; 
float lastTargetSpeed = 0;
float throttleTarget = 0;
float servoTarget = 0;
float servoPower = 0;

volatile uint8_t servoPosition;
float s_servoPosition;

volatile bool sampleSpeed = false;
volatile uint32_t seenSpeedPulses = 0;
volatile uint32_t lastSpeedSample_RTC_pit_counter = 0;

bool systemArmed = false;
bool cruiseControlEnabled = false;
bool cruiseControlPausing = false;
bool servoEnabled = false;

AceButton accelButton, decelButton, cancelButton, resumeButton; //, brakeButton;
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

void pauseCruiseControl() {
  if (cruiseControlEnabled && !cruiseControlPausing) {
    pidSwitchToManual();
    lastTargetSpeed = targetSpeed;
    servoTarget = SERVO_POSITION_PARK;
    servoPower = 200;
    cruiseControlPausing = true;
    cruiseControlEnabled = false;
  }
}

void resumeCruiseControl() {
  if (!systemArmed || cruiseControlEnabled) {
    return;
  }
  
  if (lastTargetSpeed >= MINIMUM_SPEED && lastTargetSpeed <= MAXIMUM_SPEED) {
    targetSpeed = lastTargetSpeed;
    enableCruiseControl();
  }
}

void enableCruiseControl() {
  if (!systemArmed || cruiseControlEnabled) {
    return;
  }
  
  // LED on
  PORTE.OUT |= PIN2_bm;

  enableServo();
  pidSwitchToAuto();
  cruiseControlPausing = false;
  cruiseControlEnabled = true;
}

void disableCruiseControl() {
  pidSwitchToManual();
  disableServo();
  lastTargetSpeed = targetSpeed;
  targetSpeed = 0;
  throttleTarget = 0;
  servoPower = 0;
  servoTarget = SERVO_POSITION_PARK;
  cruiseControlPausing = false;
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
      if (cruiseControlEnabled && (targetSpeed + SPEED_BUMP_PULSE_COUNT) < MAXIMUM_SPEED) {
        targetSpeed += SPEED_BUMP_PULSE_COUNT;
      }
      break;

    case AceButton::kEventLongPressed:
      // TODO: accelerate-until-released
      break;
      
    case AceButton::kEventLongReleased:
      targetSpeed = currentSpeed;
      lastTargetSpeed = targetSpeed;
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
      if (cruiseControlEnabled && (targetSpeed - SPEED_BUMP_PULSE_COUNT) > MINIMUM_SPEED) {
        targetSpeed -= SPEED_BUMP_PULSE_COUNT;
      }
      break;

    case AceButton::kEventLongPressed:
      // TODO: decelerate-until-released
      break;
      
    case AceButton::kEventLongReleased:
      targetSpeed = currentSpeed;
      lastTargetSpeed = targetSpeed;
      enableCruiseControl();
      break;
  }
}

void handleCancelButtonEvent(uint8_t eventType) {
  if (!cruiseControlEnabled) {
    return;
  }
  
  if (eventType == AceButton::kEventReleased || eventType == AceButton::kEventLongReleased) {
      pauseCruiseControl();
  }
}

void handleResumeButtonEvent(uint8_t eventType) {
  if (!systemArmed || cruiseControlEnabled) {
    return;
  }
  
  if (eventType == AceButton::kEventReleased || eventType == AceButton::kEventLongReleased) {
      resumeCruiseControl();
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
   * Unused ports: 
   *  - Enable pullup resistor
   *  - Disable input buffer and interrupt
   *  
   *  Reduces noise, and power requirements
   */
  PORTA.PIN0CTRL = PORTA.PIN2CTRL = PORTA.PIN3CTRL = PORTA.PIN4CTRL = 
  PORTA.PIN5CTRL = PORTA.PIN6CTRL = PORTA.PIN7CTRL = PORTB.PIN1CTRL = 
  PORTB.PIN3CTRL = PORTB.PIN4CTRL = PORTB.PIN5CTRL = PORTB.PIN6CTRL = 
  PORTB.PIN7CTRL = PORTC.PIN0CTRL = PORTC.PIN1CTRL = PORTC.PIN2CTRL = 
  PORTC.PIN3CTRL = PORTC.PIN4CTRL = PORTC.PIN5CTRL = PORTC.PIN7CTRL = 
  PORTD.PIN0CTRL = PORTD.PIN1CTRL = PORTD.PIN2CTRL = PORTD.PIN4CTRL = 
  PORTD.PIN5CTRL = PORTD.PIN6CTRL = PORTD.PIN7CTRL = PORTE.PIN0CTRL = 
  PORTE.PIN4CTRL = PORTE.PIN5CTRL = PORTE.PIN6CTRL = PORTE.PIN7CTRL = 
  PORTF.PIN0CTRL = PORTF.PIN1CTRL = PORTF.PIN2CTRL = PORTF.PIN3CTRL = 
  PORTF.PIN6CTRL = PORTF.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
  
  /* -------- INPUT PINS -------- */
  
  // BRAKE_INPUT PE0 (11)
  PORTE.DIR &= ~PIN0_bm;
  PORTE.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

  // ACCEL_BUTTON PA1 (7)
  PORTA.DIR &= ~PIN1_bm;
  PORTA.PIN1CTRL = PORT_PULLUPEN_bm;

  // DECEL_BUTTON PE3 (8)
  PORTE.DIR &= ~PIN3_bm;
  PORTE.PIN3CTRL = PORT_PULLUPEN_bm;

  // CANCEL_BUTTON PB0 (9)
  PORTB.DIR &= ~PIN0_bm;
  PORTB.PIN0CTRL = PORT_PULLUPEN_bm;

  // RESUME_BUTTON PB1 (10)
  PORTB.DIR &= ~PIN1_bm;
  PORTB.PIN1CTRL = PORT_PULLUPEN_bm;
 
  // SPEED_INPUT PE1 (12)
  PORTE.DIR &= ~PIN1_bm;
  PORTE.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;

  // SERVO_POSITION PD3 (14) - pot value
  PORTD.DIR &= ~PIN3_bm;
  PORTD.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
  

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
  buttonConfig->setDebounceDelay(16); // ~10ms
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
  speedPIDConfig.setOutputLimits(0.2 * THROTTLE_MAX, THROTTLE_MAX);
  speedPIDConfig.setMode(PIDConfig::kModeManual);

  speedController.init(&speedPIDConfig, &currentSpeed, &targetSpeed, &throttleTarget);
}

void SERVO_PID_init() {
  servoPIDConfig = CruisePIDConfig();
  servoPIDConfig.setGains(SERVO_PID_KP, SERVO_PID_KI, SERVO_PID_KD, PIDConfig::kActionReverse);
  servoPIDConfig.setSampleTime(SERVO_SAMPLE_INTERVAL);
  servoPIDConfig.setClockTicksPerSecond(CLOCK_TICKS_PER_SECOND);
  servoPIDConfig.setOutputLimits(-255, 255);
  servoPIDConfig.setMode(PIDConfig::kModeManual);
  
  servoController.init(&servoPIDConfig, &s_servoPosition, &servoTarget, &servoPower);
}

void WDT_init() {
  /*
   * Should the code deadlock for whatever reason
   * the hardware watchdog will reset the system about
   * 8 milliseconds after the last wdt_reset call.
   * 
   * The main loop should take <<1ms to execute
   */
  // Allow writing to protected registers
  CPU_CCP = CCP_IOREG_gc;
  // Set 8ms watchdog reset period
  WDT.CTRLA = WDT_PERIOD_8CLK_gc;
  // Wait for WDT  to sync
  while (WDT.STATUS & WDT_SYNCBUSY_bm != 0) {};
  // Reset watchdog
  wdt_reset();
}

void EVSYS_init() {
  /*
   * Use event system signaling for servo position feedback measurement
   */
  // RTC periodic interrupt as event generator on channel 5
  EVSYS.CHANNEL5 = EVSYS_GENERATOR_RTC_PIT0_gc;
  // Connect ADC0 to event channel 5
  EVSYS.USERADC0 = EVSYS_CHANNEL_CHANNEL5_gc;
}

void TCB1_init() {
  // Disable TCB1 until needed
  TCB1.CTRLA &= ~TCB_ENABLE_bm;
  /* 
   * TCB1 uses TCA clock 
   * This results in <1kHz PWM frequency with the default /64 prescaler,
   * which makes the servo extremely noisy.
   * 
   * Unfortunately TCA, DIV1, DIV2 are the only clock options
   * DIV1/DIV2 are too fast (need PWM <= 20kHz).
   * Setting TCA prescaler to /8 gives PWM @ ~15kHz, 
   * but messes up speed calculation as TCB0 is now too fast.
   * 
   */
  TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc;
  /*
   * 8-bit PWM mode
   * Async mode
   * Pin output enabled 
   * Pin initial state: disabled
   */
  TCB1.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_ASYNC_bm | TCB_CCMPEN_bm  | 0 << TCB_CCMPINIT_bp; 
  // PWM signal Period
  TCB1.CCMPH = 0;
  // Set port PF5 as output  for TCB1
  PORTMUX.TCBROUTEA |= PORTMUX_TCB1_bm;
}

void RTC_init() {
  uint8_t osc32k_CTRLA;

  /*
   * Enable internal 32k oscillator
   * It's innacurate, but good enough for this job
   */
  osc32k_CTRLA = CLKCTRL.OSC32KCTRLA;
  osc32k_CTRLA &= ~CLKCTRL_ENABLE_bm;
  // Allow writing to protected registers
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.OSC32KCTRLA = osc32k_CTRLA;

  // Wait for clocks to switch and settle
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) {;}
 
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
   * Configure prescaler (2MHz ADC clock at 8bit resolution is OK)
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
  
  if ((RTC_pit_counter - lastSpeedSample_RTC_pit_counter) == SPEED_SAMPLE_INTERVAL) {
    lastSpeedSample_RTC_pit_counter = RTC_pit_counter;
    sampleSpeed = true;
  }
}

// ADC result ready interrupt handler
ISR(ADC0_RESRDY_vect) {
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  // Read result and byte-shift to account for accumulated samples
  servoPosition = ADC0.RES >> ADC_RES_SHIFT_DIV;
}

ISR(PORTE_PORT_vect) {
  if (PORTE.INTFLAGS & PIN1_bm) {
    PORTE.INTFLAGS |= PIN1_bm;
    seenSpeedPulses++;
  } 
  
  if (PORTE.INTFLAGS & PIN0_bm) {
    PORTE.INTFLAGS |= PIN0_bm;
    disableCruiseControl();
  }
}

void checkButtons() {
  cancelButton.check();
  accelButton.check();
  decelButton.check();
  resumeButton.check();
}

/*
 *  Simplified version of analogWrite()
 *  
 *  Does the same thing, without the Arduino library compatibility overhead.
 *  Since a single analog output port is used, the routing
 *  is taken care of during TCB1 setup.
 *  
 */
void pwmWrite(int value) {
  if (value <= 0) {
    VPORTF.OUT &= ~PIN5_bm;
    return;
  }

  if (value >= 255) {
    VPORTF.OUT |= PIN5_bm;
    return;
  }

  // Disable timer
  TCB1.CTRLA &= ~TCB_ENABLE_bm;
  // PWM signal period
  TCB1.CCMPH = 128; 
  // PWM signal duty cycle
  TCB1.CCMPL = value; 
  // Enable timer
  TCB1.CTRLA |= TCB_ENABLE_bm;
}

#ifdef DEBUG_MODE
void report() {
  static uint32_t lastReport_RTC_pit_counter = 0;
  
  if (s_RTC_pit_counter - lastReport_RTC_pit_counter >= SPEED_SAMPLE_INTERVAL) {
    lastReport_RTC_pit_counter = s_RTC_pit_counter;

    Serial.print(systemArmed);
    Serial.print(",");
    Serial.print(cruiseControlEnabled);
    Serial.print(",");
    Serial.print(cruiseControlPausing);
    Serial.print(",");
    Serial.print(currentSpeed);
    Serial.print(",");
    Serial.print(targetSpeed);
    Serial.print(",");
    Serial.print(lastTargetSpeed);
    Serial.print(",");
    Serial.print(throttleTarget);
    Serial.print(",");
    Serial.print(servoTarget);
    Serial.print(",");
    Serial.print(s_servoPosition);
    Serial.print(",");
    Serial.println(servoPower);
  }  
}
#endif

void runServo(float fromPosition, float toPosition, float motorPWM) {
  int positionError = int(fromPosition) - int(toPosition);
  int pwmValue = max(SERVO_MIN_PWM_VALUE, abs(int(motorPWM)));

  if (positionError < -SERVO_POSITION_TOLERANCE) {
    PORTC.OUT |= PIN6_bm;  // B on
    PORTB.OUT &= ~PIN2_bm; // A off

    // Run motor
    pwmWrite(pwmValue);
    return;
  } 

  if (positionError > SERVO_POSITION_TOLERANCE) {   
    PORTC.OUT &= ~PIN6_bm;  // B off
    PORTB.OUT |= PIN2_bm;   // A on
   
    // Run motor
    pwmWrite(pwmValue);
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
  servoTarget = SERVO_POSITION_IDLE;
  servoPower = 0;
  servoEnabled = false;
  systemArmed = false;
  cruiseControlPausing = false;
  cruiseControlEnabled = false;
  seenSpeedPulses = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    WDT_init();
    EVSYS_init();
    RTC_init();
    TCB1_init();
    PORT_init();
    ADC_init();
  }

  BUTTON_init();
  SPEED_PID_init();
  SERVO_PID_init();

  #ifdef DEBUG_MODE
    Serial.begin(115200);
  #endif
    
  wdt_reset();
}

void loop() {  
  static bool s_sampleSpeed = false;
  static uint32_t s_seenSpeedPulses = 0;

  #ifdef DEBUG_MODE
    report();
  #endif

  // Disable interrupts, sample volatile data, restore state
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    s_RTC_pit_counter = RTC_pit_counter; 
    s_servoPosition = servoPosition;
    s_sampleSpeed = sampleSpeed;

    if (sampleSpeed) {
      s_seenSpeedPulses = seenSpeedPulses;
      seenSpeedPulses = 0;
      sampleSpeed = false;
    }
  }

  if (s_sampleSpeed) {
    currentSpeed = (float) s_seenSpeedPulses;
  }

  checkButtons();
  
  /* 
   * Arm the system only above/below the minimum/maximum speed.
   * In cases of automatic gearboxes, it's the minimum speed 
   * at which the gearbox switches to top gear + some margin to avoid kickdown on acceleration
   */
  systemArmed = (currentSpeed >= MINIMUM_SPEED) && (currentSpeed <= MAXIMUM_SPEED);

  if (!systemArmed) {
    if (cruiseControlEnabled) {
      pauseCruiseControl();
    }
    wdt_reset();
    return;
  }

  if (cruiseControlPausing && s_servoPosition >= SERVO_POSITION_PARK) {
    disableServo();
    cruiseControlPausing = false;
    wdt_reset();
    return;
  }
    
  if (cruiseControlEnabled) {
    // sometimes we miss an edge due to poor clock precision
    if (abs(targetSpeed - currentSpeed) <= SPEED_PULSE_TOLERANCE) {
      currentSpeed = targetSpeed;
    }
    speedController.compute();
    // invert throttle input and re-map to servo range
    servoTarget = trunc((((THROTTLE_MAX - THROTTLE_MIN) - throttleTarget) + THROTTLE_MIN)/((THROTTLE_MAX - THROTTLE_MIN)) * (SERVO_POSITION_IDLE - SERVO_POSITION_FULL)) + SERVO_POSITION_FULL;
    servoController.compute();
  } 

  if (servoEnabled) {
    runServo(s_servoPosition, servoTarget, servoPower);
  }
  
  // reset hardware watchdog
  wdt_reset();
}

/****************************************************************************************************************************
  ESP32_New_ISR_Servo.h
  For ESP32, ESP32_S2, ESP32_C3 boards with ESP32 core v2.0.0-rc1+
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/ESP32_New_ISR_Servo
  Licensed under MIT license

  The ESP32, ESP32_S2, ESP32_C3 have two timer groups, TIMER_GROUP_0 and TIMER_GROUP_1
  1) each group of ESP32, ESP32_S2 has two general purpose hardware timers, TIMER_0 and TIMER_1
  2) each group of ESP32_C3 has ony one general purpose hardware timer, TIMER_0
  
  All the timers are based on 64 bits counters and 16 bit prescalers. The timer counters can be configured to count up or down 
  and support automatic reload and software reload. They can also generate alarms when they reach a specific value, defined by 
  the software. The value of the counter can be read by the software program.

  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one ESP32-S2 timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.

  Based on SimpleTimer - A timer library for Arduino.
  Author: mromani@ottotecnica.com
  Copyright (c) 2010 OTTOTECNICA Italy

  Based on BlynkTimer.h
  Author: Volodymyr Shymanskyy

  Version: 1.0.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      15/08/2021 Initial coding for ESP32, ESP32_S2, ESP32_C3 boards with ESP32 core v2.0.0-rc1+
 *****************************************************************************************************************************/

#pragma once

#ifndef ESP32_New_ISR_Servo_H
#define ESP32_New_ISR_Servo_H

#if ( ARDUINO_ESP32S2_DEV || ARDUINO_FEATHERS2 || ARDUINO_ESP32S2_THING_PLUS || ARDUINO_MICROS2 || \
        ARDUINO_METRO_ESP32S2 || ARDUINO_MAGTAG29_ESP32S2 || ARDUINO_FUNHOUSE_ESP32S2 || \
        ARDUINO_ADAFRUIT_FEATHER_ESP32S2_NOPSRAM )
  #warning Using ESP32_S2-based board
#elif ( ARDUINO_ESP32C3_DEV )
  #warning Using ESP32_C3-based board
#elif defined(ESP32)
  #warning Using ESP32-based board
#else
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.  
#endif

#define ESP32_NEW_ISR_SERVO_VERSION       "ESP32_New_ISR_Servo v1.1.0"

#include <stddef.h>

#include <inttypes.h>

#if defined(ARDUINO)
  #if ARDUINO >= 100
    #include <Arduino.h>
  #else
    #include <WProgram.h>
  #endif
#endif

#include "ESP32_New_ISR_Servo_Debug.h"
#include "ESP32_New_FastTimerInterrupt.h"

#define ESP32_MAX_PIN           39
#define ESP32_WRONG_PIN         255

// From Servo.h - Copyright (c) 2009 Michael Margolis.  All right reserved.

#define MIN_PULSE_WIDTH         544       // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH         2400      // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH     1500      // default pulse width when servo is attached
#define REFRESH_INTERVAL        20000     // minumim time to refresh servos in microseconds 

extern bool IRAM_ATTR ESP32_ISR_Servo_Handler(void * timerNo);

class ESP32_ISR_Servo
{

  public:
    // maximum number of servos
    const static int MAX_SERVOS = 16;

    // constructor
    ESP32_ISR_Servo();

    // destructor
    ~ESP32_ISR_Servo()
    {
      if (ESP32_ITimer)
      {
        ESP32_ITimer->detachInterrupt();
        delete ESP32_ITimer;
      }
    }

    void IRAM_ATTR run();

    // useTimer select which timer (0-3) of ESP32 to use for Servos
    //Return true if timerN0 in range
    bool useTimer(uint8_t timerNo)
    {
      if (timerNo < MAX_ESP32_NUM_TIMERS)
      {
        _timerNo = timerNo;
        return true;
      }
      
      return false;
    }

    // Bind servo to the timer and pin, return servoIndex
    int setupServo(uint8_t pin, int min = MIN_PULSE_WIDTH, int max = MAX_PULSE_WIDTH);

    // setPosition will set servo to position in degrees
    // by using PWM, turn HIGH 'duration' microseconds within REFRESH_INTERVAL (20000us)
    // returns true on success or -1 on wrong servoIndex
    bool setPosition(unsigned servoIndex, int position);

    // returns last position in degrees if success, or -1 on wrong servoIndex
    int getPosition(unsigned servoIndex);

    // setPulseWidth will set servo PWM Pulse Width in microseconds, correcponding to certain position in degrees
    // by using PWM, turn HIGH 'pulseWidth' microseconds within REFRESH_INTERVAL (20000us)
    // min and max for each individual servo are enforced
    // returns true on success or -1 on wrong servoIndex
    bool setPulseWidth(unsigned servoIndex, unsigned int pulseWidth);

    // returns pulseWidth in microsecs (within min/max range) if success, or 0 on wrong servoIndex
    unsigned int getPulseWidth(unsigned servoIndex);

    // destroy the specified servo
    void deleteServo(unsigned servoIndex);

    // returns true if the specified servo is enabled
    bool isEnabled(unsigned servoIndex);

    // enables the specified servo
    bool enable(unsigned servoIndex);

    // disables the specified servo
    bool disable(unsigned servoIndex);

    // enables all servos
    void enableAll();

    // disables all servos
    void disableAll();

    // enables the specified servo if it's currently disabled,
    // and vice-versa
    bool toggle(unsigned servoIndex);

    // returns the number of used servos
    int getNumServos();

    // returns the number of available servos
    int getNumAvailableServos() {
      return MAX_SERVOS - numServos;
    };

  private:

    // Use 10 microsecs timer, just fine enough to control Servo, normally requiring pulse width (PWM) 500-2000us in 20ms.
#define TIMER_INTERVAL_MICRO        10

    void init()
    {
      ESP32_ITimer = new ESP32FastTimer(_timerNo);

      // Interval in microsecs
      if ( ESP32_ITimer && ESP32_ITimer->attachInterruptInterval(TIMER_INTERVAL_MICRO, (esp32_timer_callback) ESP32_ISR_Servo_Handler ) )
      {
        ISR_SERVO_LOGERROR("Starting  ITimer OK");
      }
      else
      {
        ISR_SERVO_LOGERROR("Fail setup ESP32_ITimer");      }

      for (int servoIndex = 0; servoIndex < MAX_SERVOS; servoIndex++)
      {
        memset((void*) &servo[servoIndex], 0, sizeof (servo_t));
        servo[servoIndex].count    = 0;
        servo[servoIndex].enabled  = false;
        // Intentional bad pin
        servo[servoIndex].pin      = ESP32_WRONG_PIN;
      }

      numServos   = 0;

      // Init timerCount
      timerCount  = 1;

      timerMux = portMUX_INITIALIZER_UNLOCKED;
    }

    // find the first available slot
    int findFirstFreeSlot();

    typedef struct
    {
      uint8_t       pin;                  // pin servo connected to
      unsigned long count;                // In microsecs
      int           position;             // In degrees
      bool          enabled;              // true if enabled
      uint16_t      min;
      uint16_t      max;
    } servo_t;

    volatile servo_t servo[MAX_SERVOS];

    // actual number of servos in use (-1 means uninitialized)
    volatile int numServos;

    // timerCount starts at 1, and counting up to (REFRESH_INTERVAL / TIMER_INTERVAL_MICRO) = (20000 / 10) = 2000
    // then reset to 1. Use this to calculate when to turn ON / OFF pulse to servo
    // For example, servo1 uses pulse width 1000us => turned ON when timerCount = 1, turned OFF when timerCount = 1000 / TIMER_INTERVAL_MICRO = 100
    volatile unsigned long timerCount;

    // ESP32 is a multi core / multi processing chip. It is mandatory to disable task switches during ISR
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

    // For ESP32 timer
    uint8_t _timerNo;
    ESP32FastTimer* ESP32_ITimer;
};

extern ESP32_ISR_Servo ESP32_ISR_Servos;  // create servo object to control up to 16 servos


#endif

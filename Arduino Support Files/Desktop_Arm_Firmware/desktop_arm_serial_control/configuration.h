/*
* Configuration.h
*
* Basic settings for defining hardware, configuring the SerLCD, endstops, and features
*
*/

#pragma once

// Show custom "Desktop Arm" bootscreen on setup. Uncommend to enable
//#define SHOW_BOOTSCREEN

// Choose the name from boards.h that matches your setup
#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_RAMPS_14_EFB
#endif

/**
 * Select the serial port on the board to use for communication with the host.
 * This allows the connection of wireless adapters (for instance) to non-default port pins.
 * Serial port -1 is the USB emulated serial port, if available.
 * Note: The first serial port (-1 or 0) will always be used by the Arduino bootloader.
 *
 * :[-1, 0, 1, 2, 3, 4, 5, 6, 7]
 */
#define SERIAL_PORT 0

/**
 * Serial Port Baud Rate
 * This is the default communication speed for all serial ports.
 * Set the baud rate defaults for additional serial ports below.
 *
 * 250000 works in most cases, but you might try a lower speed if
 * you commonly experience drop-outs during host printing.
 * You may try up to 1000000 to speed up SD file transfer.
 *
 * :[2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000]
 */
#define BAUDRATE 250000


//===========================================================================
//============================= PID Settings ================================
//===========================================================================
// PID Tuning Guide here: https://reprap.org/wiki/PID_Tuning

// Comment the following line to disable PID and enable bang-bang.
//#define PID
//#define BANG_MAX 255     // Limits current to nozzle while in bang-bang mode; 255=full current
//#define PID_MAX BANG_MAX // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
//#define PID_K1 0.95      // Smoothing factor within any PID loop

//#if ENABLED(PID)
//    #define DEFAULT_Kp  22.20
//    #define DEFAULT_Ki   1.08
//    #define DEFAULT_Kd 114.00
//#endif // PID



// Define endstops that the robot will use here. Usually want to use at least one per joint to 
// protect the robot from collisions. 
//
// The last joint is assumed to be a servo right now and should know its relative position. 
#define USE_JOINT0_PLUG
#define USE_JOINT1_PLUG
#define USE_JOINT2_PLUG
#define USE_JOINT3_PLUG
#define USE_JOINT4_PLUG
#define USE_JOINT5_PLUG

/**
 * Stepper Drivers
 *
 * These settings allow Marlin to tune stepper driver timing and enable advanced options for
 * stepper drivers that support them. You may also override timing options in Configuration_adv.h.
 *
 * A4988 is assumed for unspecified drivers.
 *
 * Use TMC2208/TMC2208_STANDALONE for TMC2225 drivers and TMC2209/TMC2209_STANDALONE for TMC2226 drivers.
 *
 * Options: A4988, A5984, DRV8825, LV8729, L6470, L6474, POWERSTEP01,
 *          TB6560, TB6600, TMC2100,
 *          TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE,
 *          TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE,
 *          TMC26X,  TMC26X_STANDALONE,  TMC2660, TMC2660_STANDALONE,
 *          TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE
 * :['A4988', 'A5984', 'DRV8825', 'LV8729', 'L6470', 'L6474', 'POWERSTEP01', 'TB6560', 'TB6600', 'TMC2100', 'TMC2130', 'TMC2130_STANDALONE', 'TMC2160', 'TMC2160_STANDALONE', 'TMC2208', 'TMC2208_STANDALONE', 'TMC2209', 'TMC2209_STANDALONE', 'TMC26X', 'TMC26X_STANDALONE', 'TMC2660', 'TMC2660_STANDALONE', 'TMC5130', 'TMC5130_STANDALONE', 'TMC5160', 'TMC5160_STANDALONE']
 */
#define JOINT0_DRIVER_TYPE  TMC2208 // For base
#define JOINT1_DRIVER_TYPE  TMC2208 // For both shoulder stepper motors
#define JOINT2_DRIVER_TYPE  TMC2208 // For elbow
#define JOINT3_DRIVER_TYPE  TMC2208 // For wrist roll
#define JOINT4_DRIVER_TYPE  TMC2208 // For wrist bend

#define HAS_TMC_UART false
#define HAS_WIRED_LCD false

#define motorInterfaceType 1

// =====================================================================
// ====================== Movement Settings ============================
// =====================================================================
// Default axis steps per unit (steps/mm)

#define DEFAULT_AXIS_STEPS_PER_UNIT {80, 60, 70, 80, 80}

// Movement acceleration 
#define DEFAULT_ACCELERATION          3000 

// Travel limits (mm) after homing, corresponding to endstop positions.
#define JOINT0_MIN_POS 0
#define JOINT1_MIN_POS 0
#define JOINT2_MIN_POS 0
#define JOINT3_MIN_POS 0
#define JOINT4_MIN_POS 0
#define JOINT0_MAX_POS 200
#define JOINT1_MAX_POS 200
#define JOINT2_MAX_POS 300
#define JOINT3_MAX_POS 2000
#define JOINT4_MAX_POS 4000
#define JOINT0_MAX_SPEED 200
#define JOINT1_MAX_SPEED 200
#define JOINT2_MAX_SPEED 300
#define JOINT3_MAX_SPEED 250
#define JOINT4_MAX_SPEED 250


#define GRIPPER_OPEN_POS 90
#define GRIPPER_CLOSE_POS 190

/**
 * Set these to true to diable certain joints, especially for debugging
 */
#define JOINT0_DISABLED false
#define JOINT1_DISABLED false
#define JOINT2_DISABLED false
#define JOINT3_DISABLED false
#define JOINT4_DISABLED false

//Debugging output on monitor 
# define enable_verbose true

#define PRESSED_STATE LOW // Defining the state of the limit switches depending on wiring
/**
 * Number of servos
 *
 * For some servo-related options NUM_SERVOS will be set automatically.
 * Set this manually if there are extra servos needing manual control.
 * Set to 0 to turn off servo support.
 */
//#define NUM_SERVOS 3 // Note: Servo index starts with 0 for M280-M282 commands

// (ms) Delay before the next move will start, to give the servo time to reach its target angle.
// 300ms is a good value but you can try less delay.
// If the servo can't reach the requested position, increase it.
#define SERVO_DELAY { 300 }



/**
 * RGB LED / LED Strip Control
 *
 * Enable support for an RGB LED connected to 5V digital pins, or
 * an RGB Strip connected to MOSFETs controlled by digital pins.
 *
 * Adds the M150 command to set the LED (or LED strip) color.
 * If pins are PWM capable (e.g., 4, 5, 6, 11) then a range of
 * luminance values can be set from 0 to 255.
 * For NeoPixel LED an overall brightness parameter is also available.
 *
 * *** CAUTION ***
 *  LED Strips require a MOSFET Chip between PWM lines and LEDs,
 *  as the Arduino cannot handle the current the LEDs will require.
 *  Failure to follow this precaution can destroy your Arduino!
 *  NOTE: A separate 5V power supply is required! The NeoPixel LED needs
 *  more current than the Arduino 5V linear regulator can produce.
 * *** CAUTION ***
 *
 * LED Type. Enable only one of the following two options.
 */
//#define RGB_LED
//#define RGBW_LED

//#if EITHER(RGB_LED, RGBW_LED)
  //#define RGB_LED_R_PIN 34
  //#define RGB_LED_G_PIN 43
  //#define RGB_LED_B_PIN 35
  //#define RGB_LED_W_PIN -1
//#endif

// Support for Adafruit NeoPixel LED driver
//#define NEOPIXEL_LED
//#if ENABLED(NEOPIXEL_LED)
//  #define NEOPIXEL_TYPE          NEO_GRBW // NEO_GRBW, NEO_RGBW, NEO_GRB, NEO_RBG, etc.
                                          // See https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.h
//  //#define NEOPIXEL_PIN                4 // LED driving pin
//  //#define NEOPIXEL2_TYPE  NEOPIXEL_TYPE
//  //#define NEOPIXEL2_PIN               5
//  #define NEOPIXEL_PIXELS              30 // Number of LEDs in the strip. (Longest strip when NEOPIXEL2_SEPARATE is disabled.)
//  #define NEOPIXEL_IS_SEQUENTIAL          // Sequential display for temperature change - LED by LED. Disable to change all LEDs at once.
// #define NEOPIXEL_BRIGHTNESS         127 // Initial brightness (0-255)
  //#define NEOPIXEL_STARTUP_TEST         // Cycle through colors at startup

  // Support for second Adafruit NeoPixel LED driver
  //#define NEOPIXEL2_SEPARATE
//  #if ENABLED(NEOPIXEL2_SEPARATE)
//    #define NEOPIXEL2_PIXELS           15 // Number of LEDs in the second strip
//    #define NEOPIXEL2_BRIGHTNESS      127 // Initial brightness (0-255)
//    #define NEOPIXEL2_STARTUP_TEST        // Cycle through colors at startup
//  #else
    //#define NEOPIXEL2_INSERIES          // Default behavior is NeoPixel 2 in parallel
//  #endif

  // Use some of the NeoPixel LEDs for static (background) lighting
  //#define NEOPIXEL_BKGD_INDEX_FIRST   0 // Index of the first background LED
  //#define NEOPIXEL_BKGD_INDEX_LAST    5 // Index of the last background LED
  //#define NEOPIXEL_BKGD_COLOR { 255, 255, 255, 0 }  // R, G, B, W
  //#define NEOPIXEL_BKGD_ALWAYS_ON       // Keep the backlight on when other NeoPixels are off
//#endif
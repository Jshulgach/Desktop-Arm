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
#define BAUDRATE 115200
#define CONNECT_LED_PIN_OUT 16 // Also AUX4_18_PIN on pin config

//===========================================================================
//============================= PID Settings ================================
//===========================================================================
// PID Tuning Guide here: https://reprap.org/wiki/PID_Tuning
#define USE_PID false // Set to true to use PID tuning for joint positons
#define Kp  22.20
#define Ki   1.08
#define Kd 114.00
//#define PID_K1 0.95      // Smoothing factor within any PID loop

// Define endstops that the robot will use here. Usually want to use at least one per joint to 
// protect the robot from collisions. 
//
// The last joint (5) is assumed to be a servo right now and should know its relative position. 
#define USE_JOINT0_PLUG
#define USE_JOINT1_PLUG
#define USE_JOINT2_PLUG
#define USE_JOINT3_PLUG
#define USE_JOINT4_PLUG
#define USE_JOINT5_PLUG


/* Fan Control */
#define ENABLE_FAN true
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

// Travel limits (mm) after homing, corresponding to endstop positions.
#define JOINT0_MIN_POS 0
#define JOINT1_MIN_POS 0
#define JOINT2_MIN_POS 0
#define JOINT3_MIN_POS 0
#define JOINT4_MIN_POS 0
#define JOINT0_MAX_POS 14000
#define JOINT1_MAX_POS 10000
#define JOINT2_MAX_POS 3000
#define JOINT3_MAX_POS 2000
#define JOINT4_MAX_POS 4000

// Define the speed and acceleration limits for each joint in steps per revolution
#define JOINT0_MAX_SPEED 40000
#define JOINT1_MAX_SPEED 40000
#define JOINT2_MAX_SPEED 40000
#define JOINT3_MAX_SPEED 40000
#define JOINT4_MAX_SPEED 40000
#define JOINT0_MAX_ACCELERATION 40000
#define JOINT1_MAX_ACCELERATION 40000
#define JOINT2_MAX_ACCELERATION 40000
#define JOINT3_MAX_ACCELERATION 40000
#define JOINT4_MAX_ACCELERATION 40000

// Define the homing buffer for each joint (how much to move forward after triggering homing/limit switch)
#define JOINT0_HOMING_BUFFER 5.0 // units: deg. 
#define JOINT1_HOMING_BUFFER 5.0
#define JOINT2_HOMING_BUFFER 5.0
#define JOINT3_HOMING_BUFFER 5.0
#define JOINT4_HOMING_BUFFER 5.0

// Define the number of steps each stepper motor needs to take to make a full joint revolution 
// This should also match the pin configuration on the RAMPS1.4 board for the motor drivers
// This can also be calibrated 
#define JOINT0_REV_STEPS 16000 // 1/8 microstepping
#define JOINT1_REV_STEPS 16000 // 2 jumpers, 8 degrees 
#define JOINT2_REV_STEPS 16000
#define JOINT3_REV_STEPS 16000 // 300 with 2-jumper-pin setup (what is the microstepping resolution?)
#define JOINT4_REV_STEPS 16000

// Define the servo positions for the open and close states
#define GRIPPER_OPEN_POS 90
#define GRIPPER_CLOSE_POS 300

//Set these to true to diable certain joints, especially for debugging
#define JOINT0_DISABLED false
#define JOINT1_DISABLED true // Disabled until tested on 2 stepper motors not connected to the robot
#define JOINT2_DISABLED true
#define JOINT3_DISABLED false
#define JOINT4_DISABLED false

#define PRESSED_STATE LOW // Defining the state of the limit switches depending on wiring

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

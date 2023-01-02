/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
//#pragma once

/**
 * Arduino Mega with RAMPS v1.4 (or v1.3) pin assignments
 *
 * Applies to the following boards:
 *
 *  RAMPS_14_EFB (Hotend, Fan, Bed)
 *  RAMPS_14_EEB (Hotend0, Hotend1, Bed)
 *  RAMPS_14_EFF (Hotend, Fan0, Fan1)
 *  RAMPS_14_EEF (Hotend0, Hotend1, Fan)
 *  RAMPS_14_SF  (Spindle, Controller Fan)
 *
 *  RAMPS_13_EFB (Hotend, Fan, Bed)
 *  RAMPS_13_EEB (Hotend0, Hotend1, Bed)
 *  RAMPS_13_EFF (Hotend, Fan0, Fan1)
 *  RAMPS_13_EEF (Hotend0, Hotend1, Fan)
 *  RAMPS_13_SF  (Spindle, Controller Fan)
 *
 *  Other pins_MYBOARD.h files may override these defaults
 *
 *  Differences between
 *  RAMPS_13 | RAMPS_14
 *         7 | 11
 */

//#if ENABLED(AZSMZ_12864) && DISABLED(ALLOW_SAM3X8E)
//  #error "No pins defined for RAMPS with AZSMZ_12864."
//#endif

//#include "env_validate.h"

// Custom flags and defines for the build
//#define BOARD_CUSTOM_BUILD_FLAGS -D__FOO__

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "RAMPS 1.4"
#endif

//
// Servos
//
#ifndef SERVO0_PIN
  #ifdef IS_RAMPS_13
    #define SERVO0_PIN                         7
  #else
    #define SERVO0_PIN                        11
  #endif
#endif
#ifndef SERVO1_PIN
  #define SERVO1_PIN                           6
#endif
#ifndef SERVO2_PIN
  #define SERVO2_PIN                           5
#endif
#ifndef SERVO3_PIN
  #define SERVO3_PIN                           4
#endif

//
// Limit Switches
// X
#ifndef JOINT0_MIN_PIN
  #define JOINT0_MIN_PIN                          3
#endif
#ifndef JOINT1_MIN_PIN
  #define JOINT1_MIN_PIN                          2
#endif
// Y
#ifndef JOINT2_MIN_PIN
  #define JOINT2_MIN_PIN                         14
#endif
#ifndef JOINT3_MIN_PIN
  #define JOINT3_MIN_PIN                         15
#endif
// Z
#ifndef JOINT4_MIN_PIN
  #define JOINT4_MIN_PIN                         18
#endif
#ifndef EXTRA_MIN_PIN
  #define EXTRA_MIN_PIN                         19
#endif


//
// Steppers
//
#define JOINT2_STEP_PIN                            54 //X
#define JOINT2_DIR_PIN                             55
#define JOINT2_ENABLE_PIN                          38
#define JOINT2_CS_PIN                              53

#define JOINT0_STEP_PIN                            60 // Y
#define JOINT0_DIR_PIN                             61
#define JOINT0_ENABLE_PIN                          56
#define JOINT0_CS_PIN                              49

#define JOINT1_STEP_PIN                            46 //Z
#define JOINT1_DIR_PIN                             48
#define JOINT1_ENABLE_PIN                          62
#define JOINT1_CS_PIN                              40

#define JOINT3_STEP_PIN                            26 //E0
#define JOINT3_DIR_PIN                             28
#define JOINT3_ENABLE_PIN                          24
#define JOINT3_CS_PIN                              42

#define JOINT4_STEP_PIN                            36 //E1
#define JOINT4_DIR_PIN                             34
#define JOINT4_ENABLE_PIN                          30
#define JOINT4_CS_PIN                              44

//
// Temperature Sensors
//
#ifndef TEMP_0_PIN
  #define TEMP_0_PIN                          13  // Analog Input
#endif
#ifndef TEMP_1_PIN
  #define TEMP_1_PIN                          15  // Analog Input
#endif
#ifndef TEMP_BED_PIN
  #define TEMP_BED_PIN                        14  // Analog Input
#endif

//
// Fans
//
#define FAN_0_PIN 9 // Also #define AUX4_17_PIN


#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   *
   * Serial2 -- AUX-4 Pin 18 (D16 TX2) and AUX-4 Pin 17 (D17 RX2)
   * Serial1 -- Pins D18 and D19 are used for Z-MIN and Z-MAX
   */
  //#define X_HARDWARE_SERIAL Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1

  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                   40
  #endif
  #ifndef X_SERIAL_RX_PIN
    #define X_SERIAL_RX_PIN                   63
  #endif
  #ifndef X2_SERIAL_TX_PIN
    #define X2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef X2_SERIAL_RX_PIN
    #define X2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                   59
  #endif
  #ifndef Y_SERIAL_RX_PIN
    #define Y_SERIAL_RX_PIN                   64
  #endif
  #ifndef Y2_SERIAL_TX_PIN
    #define Y2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef Y2_SERIAL_RX_PIN
    #define Y2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                   42
  #endif
  #ifndef Z_SERIAL_RX_PIN
    #define Z_SERIAL_RX_PIN                   65
  #endif
  #ifndef Z2_SERIAL_TX_PIN
    #define Z2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef Z2_SERIAL_RX_PIN
    #define Z2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                  44
  #endif
  #ifndef E0_SERIAL_RX_PIN
    #define E0_SERIAL_RX_PIN                  66
  #endif
  #ifndef E1_SERIAL_TX_PIN
    #define E1_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E1_SERIAL_RX_PIN
    #define E1_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E2_SERIAL_TX_PIN
    #define E2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E2_SERIAL_RX_PIN
    #define E2_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E3_SERIAL_TX_PIN
    #define E3_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E3_SERIAL_RX_PIN
    #define E3_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E4_SERIAL_TX_PIN
    #define E4_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E4_SERIAL_RX_PIN
    #define E4_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E5_SERIAL_TX_PIN
    #define E5_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E5_SERIAL_RX_PIN
    #define E5_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E6_SERIAL_TX_PIN
    #define E6_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E6_SERIAL_RX_PIN
    #define E6_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E7_SERIAL_TX_PIN
    #define E7_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E7_SERIAL_RX_PIN
    #define E7_SERIAL_RX_PIN                  -1
  #endif
#endif



//
// AUX1    5V  GND D2  D1
//          2   4   6   8
//          1   3   5   7
//         5V  GND A3  A4
//
#define AUX1_05_PIN                           57  // (A3)
#define AUX1_06_PIN                            2
#define AUX1_07_PIN                           58  // (A4)
#define AUX1_08_PIN                            1

//
// AUX2    GND A9 D40 D42 A11
//          2   4   6   8  10
//          1   3   5   7   9
//         VCC A5 A10 D44 A12
//
#define AUX2_03_PIN                           59  // (A5)
#define AUX2_04_PIN                           63  // (A9)
#define AUX2_05_PIN                           64  // (A10)
#define AUX2_06_PIN                           40
#define AUX2_07_PIN                           44
#define AUX2_08_PIN                           42
#define AUX2_09_PIN                           66  // (A12)
#define AUX2_10_PIN                           65  // (A11)

//
// AUX3    GND D52 D50 5V
//          7   5   3   1
//          8   6   4   2
//         NC  D53 D51 D49
//
#define AUX3_02_PIN                           49
#define AUX3_03_PIN                           50
#define AUX3_04_PIN                           51
#define AUX3_05_PIN                           52
#define AUX3_06_PIN                           53

//
// AUX4    5V GND D32 D47 D45 D43 D41 D39 D37 D35 D33 D31 D29 D27 D25 D23 D17 D16
//
#define AUX4_03_PIN                           32
#define AUX4_04_PIN                           47
#define AUX4_05_PIN                           45
#define AUX4_06_PIN                           43
#define AUX4_07_PIN                           41
#define AUX4_08_PIN                           39
#define AUX4_09_PIN                           37
#define AUX4_10_PIN                           35
#define AUX4_11_PIN                           33
#define AUX4_12_PIN                           31
#define AUX4_13_PIN                           29
#define AUX4_14_PIN                           27
#define AUX4_15_PIN                           25
#define AUX4_16_PIN                           23
#define AUX4_17_PIN                           17
#define AUX4_18_PIN                           16



//////////////////////////
// LCDs and Controllers //
//////////////////////////

#if HAS_WIRED_LCD

  //#define LCD_SCREEN_ROTATE                180  // 0, 90, 180, 270

  //
  // LCD Display output pins
  //
  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

    #define LCD_PINS_RS              EXP2_04_PIN  // CS chip select /SS chip slave select
    #define LCD_PINS_ENABLE          EXP2_05_PIN  // SID (MOSI)
    #define LCD_PINS_D4              EXP2_09_PIN  // SCK (CLK) clock

  #elif BOTH(IS_NEWPANEL, PANEL_ONE)

    #define LCD_PINS_RS              AUX2_06_PIN
    #define LCD_PINS_ENABLE          AUX2_08_PIN
    #define LCD_PINS_D4              AUX2_10_PIN
    #define LCD_PINS_D5              AUX2_09_PIN
    #define LCD_PINS_D6              AUX2_07_PIN
    #define LCD_PINS_D7              AUX2_05_PIN

  #elif ENABLED(TFTGLCD_PANEL_SPI)

    #define TFTGLCD_CS               EXP2_06_PIN

  #else

    #if ENABLED(CR10_STOCKDISPLAY)

      #define LCD_PINS_RS            EXP1_04_PIN
      #define LCD_PINS_ENABLE        EXP1_03_PIN
      #define LCD_PINS_D4            EXP1_05_PIN

      #if !IS_NEWPANEL
        #define BEEPER_PIN           EXP1_10_PIN
      #endif

    #elif ENABLED(ZONESTAR_LCD)

      #error "CAUTION! ZONESTAR_LCD on RAMPS requires wiring modifications. It plugs into AUX2 but GND and 5V need to be swapped. Comment out this line to continue."
      #define LCD_PINS_RS            AUX2_05_PIN
      #define LCD_PINS_ENABLE        AUX2_07_PIN
      #define LCD_PINS_D4            AUX2_04_PIN
      #define LCD_PINS_D5            AUX2_06_PIN
      #define LCD_PINS_D6            AUX2_08_PIN
      #define LCD_PINS_D7            AUX2_10_PIN

    #elif ENABLED(AZSMZ_12864)

      // Pins only defined for RAMPS_SMART currently

    #else

      #if EITHER(MKS_12864OLED, MKS_12864OLED_SSD1306)
        #define LCD_PINS_DC          EXP1_05_PIN  // Set as output on init
        #define LCD_PINS_RS          EXP1_04_PIN  // Pull low for 1s to init
        // DOGM SPI LCD Support
        #define DOGLCD_A0            LCD_PINS_DC
        #define DOGLCD_CS            EXP1_07_PIN
        #define DOGLCD_MOSI          EXP1_08_PIN
        #define DOGLCD_SCK           EXP1_06_PIN
      #else
        #define LCD_PINS_RS          EXP1_07_PIN
        #define LCD_PINS_ENABLE      EXP1_08_PIN
        #define LCD_PINS_D4          EXP1_06_PIN
        #define LCD_PINS_D5          EXP1_05_PIN
        #define LCD_PINS_D6          EXP1_04_PIN
      #endif

      #define LCD_PINS_D7            EXP1_03_PIN

      #if !IS_NEWPANEL
        #define BEEPER_PIN           EXP2_06_PIN
      #endif

    #endif

    #if !IS_NEWPANEL
      // Buttons attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK_PIN                 38
      //#define SHIFT_LD_PIN         AUX2_08_PIN
      //#define SHIFT_OUT_PIN        AUX2_06_PIN
      //#define SHIFT_EN_PIN         EXP1_08_PIN
    #endif

  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

  //
  // LCD Display input pins
  //
  #if IS_NEWPANEL

    #if IS_RRD_SC

      #define BEEPER_PIN             EXP1_10_PIN

      #if ENABLED(CR10_STOCKDISPLAY)
        #define BTN_EN1              EXP1_08_PIN
        #define BTN_EN2              EXP1_06_PIN
      #else
        #define BTN_EN1              EXP2_08_PIN
        #define BTN_EN2              EXP2_06_PIN
      #endif

      #define BTN_ENC                EXP1_09_PIN
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_04_PIN
      #endif
      #ifndef KILL_PIN
        #define KILL_PIN             EXP2_03_PIN
      #endif

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN    AUX4_08_PIN
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      #define BTN_EN1                AUX2_05_PIN
      #define BTN_EN2                AUX2_03_PIN
      #define BTN_ENC                AUX2_04_PIN
      #define SD_DETECT_PIN          AUX2_08_PIN

    #elif ENABLED(LCD_I2C_PANELOLU2)

      #define BTN_EN1                AUX4_04_PIN
      #define BTN_EN2                AUX4_06_PIN
      #define BTN_ENC                AUX4_03_PIN
      #define LCD_SDSS                      SDSS
      #define KILL_PIN               EXP2_03_PIN

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1                AUX2_06_PIN  // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains AUX2-06 and AUX2-08.
      #define BTN_EN2                AUX2_08_PIN
      #define BTN_ENC                         -1

      #define LCD_SDSS                      SDSS
      #define SD_DETECT_PIN          EXP2_04_PIN

    #elif ANY(VIKI2, miniVIKI)

      #define DOGLCD_CS              AUX4_05_PIN
      #define DOGLCD_A0              AUX2_07_PIN
      #define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270

      #define BEEPER_PIN             EXP2_06_PIN
      #define STAT_LED_RED_PIN       AUX4_03_PIN
      #define STAT_LED_BLUE_PIN      EXP1_09_PIN

      #define BTN_EN1                         22
      #define BTN_EN2                          7
      #define BTN_ENC                AUX4_08_PIN

      #define SD_DETECT_PIN                   -1  // Pin 49 for display SD interface, 72 for easy adapter board
      #define KILL_PIN               EXP2_08_PIN

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN

      #define BEEPER_PIN             EXP1_06_PIN
      #define LCD_BACKLIGHT_PIN      EXP2_06_PIN

      #define BTN_EN1                EXP1_09_PIN
      #define BTN_EN2                EXP1_10_PIN
      #define BTN_ENC                EXP2_08_PIN

      #define LCD_SDSS                      SDSS
      #define SD_DETECT_PIN          EXP2_04_PIN
      #define KILL_PIN               EXP2_03_PIN

    #elif EITHER(MKS_MINI_12864, FYSETC_MINI_12864)

      #define BEEPER_PIN             EXP1_10_PIN
      #define BTN_ENC                EXP1_09_PIN
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_04_PIN
      #endif

      #ifndef KILL_PIN
        #define KILL_PIN             EXP2_03_PIN
      #endif

      #if ENABLED(MKS_MINI_12864)

        #define DOGLCD_A0            EXP1_04_PIN
        #define DOGLCD_CS            EXP1_05_PIN

        // not connected to a pin
        #define LCD_BACKLIGHT_PIN             -1  // 65 (MKS mini12864 can't adjust backlight by software!)

        #define BTN_EN1              EXP2_08_PIN
        #define BTN_EN2              EXP2_06_PIN

      #elif ENABLED(FYSETC_MINI_12864)

        // From https://wiki.fysetc.com/Mini12864_Panel/

        #define DOGLCD_A0            EXP1_07_PIN
        #define DOGLCD_CS            EXP1_08_PIN

        #define BTN_EN1              EXP2_06_PIN
        #define BTN_EN2              EXP2_08_PIN

        //#define FORCE_SOFT_SPI                  // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

        #define LCD_RESET_PIN        EXP1_06_PIN  // Must be high or open for LCD to operate normally.

        #if EITHER(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
          #ifndef RGB_LED_R_PIN
            #define RGB_LED_R_PIN    EXP1_05_PIN
          #endif
          #ifndef RGB_LED_G_PIN
            #define RGB_LED_G_PIN    EXP1_04_PIN
          #endif
          #ifndef RGB_LED_B_PIN
            #define RGB_LED_B_PIN    EXP1_03_PIN
          #endif
        #elif ENABLED(FYSETC_MINI_12864_2_1)
          #define NEOPIXEL_PIN       EXP1_05_PIN
        #endif

      #endif

    #elif ENABLED(MINIPANEL)

      #define BEEPER_PIN             AUX2_08_PIN
      #define LCD_BACKLIGHT_PIN      AUX2_10_PIN

      #define DOGLCD_A0              AUX2_07_PIN
      #define DOGLCD_CS              AUX2_09_PIN

      #define BTN_EN1                AUX2_06_PIN
      #define BTN_EN2                AUX2_04_PIN
      #define BTN_ENC                AUX2_03_PIN

      #define SD_DETECT_PIN          AUX3_02_PIN
      #define KILL_PIN               AUX2_05_PIN

    #elif ENABLED(ZONESTAR_LCD)

      #define ADC_KEYPAD_PIN                  12

    #elif ENABLED(AZSMZ_12864)

      // Pins only defined for RAMPS_SMART currently

    #elif ENABLED(G3D_PANEL)

      #define BEEPER_PIN             EXP1_10_PIN

      #define SD_DETECT_PIN          EXP2_04_PIN
      #define KILL_PIN               EXP2_03_PIN

      #define BTN_EN1                EXP2_06_PIN
      #define BTN_EN2                EXP2_08_PIN
      #define BTN_ENC                EXP1_09_PIN

    #elif IS_TFTGLCD_PANEL

      #define SD_DETECT_PIN          EXP2_04_PIN

    #else

      #define BEEPER_PIN             EXP2_06_PIN

      #if ENABLED(PANEL_ONE)                       // Buttons connect directly to AUX-2
        #define BTN_EN1              AUX2_03_PIN
        #define BTN_EN2              AUX2_04_PIN
        #define BTN_ENC              AUX3_02_PIN
      #else
        #define BTN_EN1              EXP1_10_PIN
        #define BTN_EN2              EXP1_09_PIN
        #define BTN_ENC              EXP2_08_PIN
      #endif

    #endif
  #endif // IS_NEWPANEL

#endif // HAS_WIRED_LCD

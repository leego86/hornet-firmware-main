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
#pragma once

/**
 * Configuration.h
 *
 * Basic settings such as:
 *
 * - Type of electronics  |电子设备类型
 * - Type of temperature sensor  |温度传感器类型
 * - Printer geometry  |打印机几何形状
 * - Endstop configuration  |终点停止配置
 * - LCD controller  |液晶控制器
 * - Extra features  |额外功能
 *
 * Advanced settings can be found in Configuration_adv.h  |高级设置可以在Configuration_adv.h中找到
 */
#define CONFIGURATION_H_VERSION 020007

//===========================================================================
//============================= Getting Started =============================
//===========================================================================

/**
 * Here are some standard links for getting your machine calibrated:
 * 以下是一些用于校准机器的标准链接
 *
 * https://reprap.org/wiki/Calibration
 * https://youtu.be/wAL9d7FgInk
 * http://calculator.josefprusa.cz
 * https://reprap.org/wiki/Triffid_Hunter%27s_Calibration_Guide
 * https://www.thingiverse.com/thing:5573
 * https://sites.google.com/site/repraplogphase/calibration-of-your-reprap
 * https://www.thingiverse.com/thing:298812
 */

//===========================================================================
//============================= DELTA Printer ===============================
//===========================================================================
// For a Delta printer start with one of the configuration files in the  |对于 Delta 打印机，从以下配置文件之一开始：
// config/examples/delta directory and customize for your machine.   |config/examples/delta 目录并针对您的计算机进行自定义。
//

//===========================================================================
//============================= SCARA Printer ===============================
//===========================================================================
// For a SCARA printer start with the configuration files in
// config/examples/SCARA and customize for your machine.
//

// @section info

// Author info of this build printed to the host during boot and M115  |此版本的作者信息在启动和 M115 期间打印到主机
#define STRING_CONFIG_H_AUTHOR "Artillery 3D" // Who made the changes.
//#define CUSTOM_VERSION_FILE Version.h // Path from the root directory (no quotes) |根目录的路径（无引号）

/**
 * *** VENDORS PLEASE READ ***
 *       供应商请阅读 
 *
 * Marlin allows you to add a custom boot image for Graphical LCDs. |Marlin 允许您为图形 LCD 添加自定义启动映像。
 * With this option Marlin will first show your custom screen followed |使用此选项 Marlin 将首先显示您的自定义屏幕，然后显示
 * by the standard Marlin logo with version number and web URL.  |采用标准 Marlin 徽标以及版本号和网址。
 *
 * We encourage you to take advantage of this new feature and we also  |我们鼓励您利用这个新功能，我们也
 * respectfully request that you retain the unmodified Marlin boot screen.  |恳请您保留未经修改的 Marlin 启动屏幕。
 */

// Show the Marlin bootscreen on startup. ** ENABLE FOR PRODUCTION **  |启动时显示 Marlin 引导屏幕。 **投入生产 **
#define SHOW_BOOTSCREEN

// Show the bitmap in Marlin/_Bootscreen.h on startup.    |启动时在 Marlin/_Bootscreen.h 中显示位图。
#define SHOW_CUSTOM_BOOTSCREEN

// Show the bitmap in Marlin/_Statusscreen.h on the status screen.    |在状态屏幕上显示 Marlin/_Statusscreen.h 中的位图。
//#define CUSTOM_STATUS_SCREEN_IMAGE

// @section machine

/**
 * Select the serial port on the board to use for communication with the host.    |选择板上用于与主机通信的串口。
 * This allows the connection of wireless adapters (for instance) to non-default port pins.    |这允许将无线适配器（例如）连接到非默认端口引脚。
 * Serial port -1 is the USB emulated serial port, if available.    |串行端口 -1 是 USB 模拟串行端口（如果有）。 
 * Note: The first serial port (-1 or 0) will always be used by the Arduino bootloader.    |注意：第一个串行端口（-1 或 0）将始终由 Arduino 引导加载程序使用。  
 *
 * :[-1, 0, 1, 2, 3, 4, 5, 6, 7]
 */
#define SERIAL_PORT -1

/**
 * Select a secondary serial port on the board to use for communication with the host.    |选择板上的辅助串行端口用于与主机通信。
 * :[-1, 0, 1, 2, 3, 4, 5, 6, 7]
 */
//#define SERIAL_PORT_2 -1

/**
 * This setting determines the communication speed of the printer.    |串口波特率   此设置决定打印机的通信速度。
 BAUDRATE 设置串口通信的波特率，一般默认是250000，如果使用Mac或者Linux系统，需要改成115200，因为系统原因波特率上不到250000。对应的切片或联机打印软件也需要给波特率改为设置的值，否则无法连接。
 * 250000 works in most cases, but you might try a lower speed if  |大多数情况下 250000 有效，但如果出现以下情况，您可以尝试较低的速度
 * you commonly experience drop-outs during host printing.  |在主机打印过程中，您经常会遇到掉线的情况。
 * You may try up to 1000000 to speed up SD file transfer.  |您可以尝试高达 1000000 来加快 SD 文件传输速度。
 *
 * :[2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000]
 */
#define BAUDRATE 250000

// Enable the Bluetooth serial interface on AT90USB devices  |在 AT90USB 设备上启用蓝牙串行接口
//#define BLUETOOTH
//主板类型
//现在最新版本的Marlin固件固件已经默认设置为Makeboard主板主板类型，无需再更改。根据自己情况修改。
// Choose the name from boards.h that matches your setup  |从boards.h中选择与您的设置相匹配的名称
#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_ARTILLERY_RUBY
#endif

// Name displayed in the LCD "Ready" message and Info menu    |LCD“就绪”消息和信息菜单中显示的名称
#define CUSTOM_MACHINE_NAME "Artillery Hornet"

// Printer's unique ID, used by some programs to differentiate between machines.    |打印机的唯一 ID，某些程序使用它来区分机器
// Choose your own or use a service like https://www.uuidgenerator.net/version4    |选择您自己的或使用类似 https://www.uuidgenerator.net/version4的服务
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// @section extruder

// This defines the number of extruders    |定义挤出机数量
// :[0, 1, 2, 3, 4, 5, 6, 7, 8]
#define EXTRUDERS 1

// Generally expected filament diameter (1.75, 2.85, 3.0, ...). Used for Volumetric, Filament Width Sensor, etc.    
//|一般预期的线材直径（1.75、2.85、3.0，...）。用于体积、细丝宽度传感器等
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75

//混色打印
//去掉 SINGLENOZZLE 行的注释，可以开启多挤出机混色打印，就是有多个挤出机挤出不同的颜色，但只有一个打印头，可靠切片软件安排不同挤出机颜色的挤出量达到混色的彩色效果。
// For Cyclops or any "multi-extruder" that shares a single nozzle.    |适用于 Cyclops 或任何共享单个喷嘴的“多挤出机”。
//#define SINGLENOZZLE

// Save and restore temperature and fan speed on tool-change.    |更换工具时保存和恢复温度和风扇速度。
// Set standby for the unselected tool with M104/106/109 T...    |使用 M104/106/109 T... 为未选择的刀具设置待机状态
#if ENABLED(SINGLENOZZLE)
  //#define SINGLENOZZLE_STANDBY_TEMP
  //#define SINGLENOZZLE_STANDBY_FAN
#endif

/**
 * Průša MK2 Single Nozzle Multi-Material Multiplexer, and variants.    |Průša MK2 单喷嘴多材料复用器及其变体。
 *
 * This device allows one stepper driver on a control board to drive    |该设备允许控制板上的一个步进驱动器来驱动
 * two to eight stepper motors, one at a time, in a manner suitable    |两到八个步进电机，一次一个，以合适的方式
 * for extruders.    |用于挤出机。
 *
 * This option only allows the multiplexer to switch on tool-change.    |此选项仅允许多路复用器开启换刀。
 * Additional options to configure custom E moves are pending.    |配置自定义 E 移动的其他选项正在等待确定。
 */
//#define MK2_MULTIPLEXER
#if ENABLED(MK2_MULTIPLEXER)
  // Override the default DIO selector pins here, if needed.    |如果需要，可以在此处覆盖默认的 DIO 选择器引脚。
  // Some pins files may provide defaults for these pins.    |某些引脚文件可能会提供这些引脚的默认值。
  //#define E_MUX0_PIN 40  // Always Required    |始终需要
  //#define E_MUX1_PIN 42  // Needed for 3 to 8 inputs    |3 至 8 个输入需要
  //#define E_MUX2_PIN 44  // Needed for 5 to 8 inputs    |5 到 8 个输入需要
#endif

/**
 * Průša Multi-Material Unit v2    |Průša 多材料单元 v2 
 *
 * Requires NOZZLE_PARK_FEATURE to park print head in case MMU unit fails.    |需要 NOZZLE_PARK_FEATURE 来停放打印头，以防 MMU 单元出现故障。
 * Requires EXTRUDERS = 5    |需要挤出机 = 5
 *
 * For additional configuration see Configuration_adv.h    |有关其他配置，请参阅 Configuration_adv.h
 */
//#define PRUSA_MMU2

// A dual extruder that uses a single stepper motor    |使用单个步进电机的双挤出机
//#define SWITCHING_EXTRUDER
#if ENABLED(SWITCHING_EXTRUDER)
  #define SWITCHING_EXTRUDER_SERVO_NR 0
  #define SWITCHING_EXTRUDER_SERVO_ANGLES { 0, 90 } // Angles for E0, E1[, E2, E3]    |E0, E1[, E2, E3]的角度
  #if EXTRUDERS > 3
    #define SWITCHING_EXTRUDER_E23_SERVO_NR 1
  #endif
#endif

// A dual-nozzle that uses a servomotor to raise/lower one (or both) of the nozzles    |一种双喷嘴，使用伺服电机升高/降低一个（或两个）喷嘴 
//#define SWITCHING_NOZZLE
#if ENABLED(SWITCHING_NOZZLE)
  #define SWITCHING_NOZZLE_SERVO_NR 0
  //#define SWITCHING_NOZZLE_E1_SERVO_NR 1          // If two servos are used, the index of the second    |如果使用两个舵机，则第二个舵机的索引
  #define SWITCHING_NOZZLE_SERVO_ANGLES { 0, 90 }   // Angles for E0, E1 (single servo) or lowered/raised (dual servo)    |E0、E1（单伺服）或降低/升高（双伺服）的角度
#endif

/**
 * Two separate X-carriages with extruders that connect to a moving part    |两个独立的 X 型托架，带有连接到移动部件的挤出机
 * via a solenoid docking mechanism. Requires SOL1_PIN and SOL2_PIN.    |通过电磁阀对接机构。需要 SOL1_PIN 和 SOL2_PIN。
 */
//#define PARKING_EXTRUDER

/**
 * Two separate X-carriages with extruders that connect to a moving part    |两个独立的 X 型托架，带有连接到移动部件的挤出机
 * via a magnetic docking mechanism using movements and no solenoid    |通过磁性对接机制使用移动且无螺线管
 *
 * project   : https://www.thingiverse.com/thing:3080893
 * movements : https://youtu.be/0xCEiG9VS3k
 *             https://youtu.be/Bqbcs0CU2FE
 */
//#define MAGNETIC_PARKING_EXTRUDER

#if EITHER(PARKING_EXTRUDER, MAGNETIC_PARKING_EXTRUDER)

  #define PARKING_EXTRUDER_PARKING_X { -78, 184 }     // X positions for parking the extruders    |用于停放挤出机的 X 位置
  #define PARKING_EXTRUDER_GRAB_DISTANCE 1            // (mm) Distance to move beyond the parking point to grab the extruder    |(mm) 超出停车点以抓取挤出机的距离
  //#define MANUAL_SOLENOID_CONTROL                   // Manual control of docking solenoids with M380 S / M381    |使用 M380 S /M381 手动控制对接电磁阀

  #if ENABLED(PARKING_EXTRUDER)

    #define PARKING_EXTRUDER_SOLENOIDS_INVERT           // If enabled, the solenoid is NOT magnetized with applied voltage    | 如果启用，螺线管不会因施加电压而磁化 
    #define PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE LOW  // LOW or HIGH pin signal energizes the coil    |低或高引脚信号为线圈供电
    #define PARKING_EXTRUDER_SOLENOIDS_DELAY 250        // (ms) Delay for magnetic field. No delay if 0 or not defined.    |(ms) 磁场延迟。如果为 0 或未定义，则无延迟。
    //#define MANUAL_SOLENOID_CONTROL                   // Manual control of docking solenoids with M380 S / M381    |使用 M380 S /M381 手动控制对接电磁阀

  #elif ENABLED(MAGNETIC_PARKING_EXTRUDER)

    #define MPE_FAST_SPEED      9000      // (mm/min) Speed for travel before last distance point    | (mm/min) 最后一个距离点之前的行进速度
    #define MPE_SLOW_SPEED      4500      // (mm/min) Speed for last distance travel to park and couple    |(毫米/分钟) 到公园和情侣的最后距离行驶速度
    #define MPE_TRAVEL_DISTANCE   10      // (mm) Last distance point    |(mm) 最后距离点
    #define MPE_COMPENSATION       0      // Offset Compensation -1 , 0 , 1 (multiplier) only for coupling    |偏移补偿 -1 , 0 , 1（乘数）仅适用于耦合

  #endif

#endif

/**
 * Switching Toolhead    |切换工具头
 *
 * Support for swappable and dockable toolheads, such as    |支持可交换和可对接工具头，例如
 * the E3D Tool Changer. Toolheads are locked with a servo.    |E3D 换刀装置。工具头通过伺服系统锁定。
 */
//#define SWITCHING_TOOLHEAD

/**
 * Magnetic Switching Toolhead    |磁力开关工具头
 *
 * Support swappable and dockable toolheads with a magnetic    |支持带有磁性的可交换和可对接工具头
 * docking mechanism using movement and no servo.    |对接机构采用运动且无伺服。
 */
//#define MAGNETIC_SWITCHING_TOOLHEAD

/**
 * Electromagnetic Switching Toolhead    |电磁开关工具头
 *
 * Parking for CoreXY / HBot kinematics.    |CoreXY /HBot 运动学的停车。
 * Toolheads are parked at one edge and held with an electromagnet.    |工具头停放在一侧边缘并用电磁铁固定。
 * Supports more than 2 Toolheads. See https://youtu.be/JolbsAKTKf4    |支持2个以上工具头。请参阅 https://youtu.be/JolbsAKTKf4
 */
//#define ELECTROMAGNETIC_SWITCHING_TOOLHEAD

#if ANY(SWITCHING_TOOLHEAD, MAGNETIC_SWITCHING_TOOLHEAD, ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
  #define SWITCHING_TOOLHEAD_Y_POS          235         // (mm) Y position of the toolhead dock    |(mm) 工具头底座的 Y 位置
  #define SWITCHING_TOOLHEAD_Y_SECURITY      10         // (mm) Security distance Y axis    |(mm) 安全距离Y轴
  #define SWITCHING_TOOLHEAD_Y_CLEAR         60         // (mm) Minimum distance from dock for unobstructed X axis    |(mm) X 轴不受阻碍时距底座的最小距离
  #define SWITCHING_TOOLHEAD_X_POS          { 215, 0 }  // (mm) X positions for parking the extruders    |(mm) 用于停放挤出机的 X 位置
  #if ENABLED(SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_SERVO_NR       2         // Index of the servo connector    |伺服连接器索引
    #define SWITCHING_TOOLHEAD_SERVO_ANGLES { 0, 180 }  // (degrees) Angles for Lock, Unlock    |（度）锁定、解锁角度
  #elif ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_Y_RELEASE      5         // (mm) Security distance Y axis    |(mm) 安全距离Y轴
    #define SWITCHING_TOOLHEAD_X_SECURITY   { 90, 150 } // (mm) Security distance X axis (T0,T1)    |(mm) X轴安全距离(T0,T1)
    //#define PRIME_BEFORE_REMOVE                       // Prime the nozzle before release from the dock    |在从底座释放之前先灌注喷嘴
    #if ENABLED(PRIME_BEFORE_REMOVE)
      #define SWITCHING_TOOLHEAD_PRIME_MM           20  // (mm)   Extruder prime length    |(mm) 挤出机主要长度
      #define SWITCHING_TOOLHEAD_RETRACT_MM         10  // (mm)   Retract after priming length    |(mm) 启动后缩回长度
      #define SWITCHING_TOOLHEAD_PRIME_FEEDRATE    300  // (mm/min) Extruder prime feedrate    | (mm/min) 挤出机主进给速度
      #define SWITCHING_TOOLHEAD_RETRACT_FEEDRATE 2400  // (mm/min) Extruder retract feedrate    |(mm/min) 挤出机缩回进给率
    #endif
  #elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_Z_HOP          2         // (mm) Z raise for switching    |(mm) 用于切换的 Z 升程
  #endif
#endif

/**
 * "Mixing Extruder"    |“混合挤出机”
 *   - Adds G-codes M163 and M164 to set and "commit" the current mix factors.    |添加 G 代码 M163 和 M164 以设置和“提交”当前混合因子。
 *   - Extends the stepping routines to move multiple steppers in proportion to the mix.    |扩展步进例程以按混音比例移动多个步进器。
 *   - Optional support for Repetier Firmware's 'M164 S<index>' supporting virtual tools.    |可选支持 Repetier Firmware 的“M164 S<index>”支持虚拟工具。
 *   - This implementation supports up to two mixing extruders.    |此实施支持最多两个混合挤出机。
 *   - Enable DIRECT_MIXING_IN_G1 for M165 and mixing in G1 (from Pia Taubert's reference implementation).    |为 M165 启用 DIRECT_MIXING_IN_G1 并在 G1 中混合（来自 Pia Taubert 的参考实现）。
 */
//#define MIXING_EXTRUDER
#if ENABLED(MIXING_EXTRUDER)
  #define MIXING_STEPPERS 2        // Number of steppers in your mixing extruder    |混合挤出机中的步进器数量
  #define MIXING_VIRTUAL_TOOLS 16  // Use the Virtual Tool method with M163 and M164    |使用虚拟工具方法与 M163 和 M164
  //#define DIRECT_MIXING_IN_G1    // Allow ABCDHI mix factors in G1 movement commands    |允许 G1 移动命令中的 ABCDHI 混合因子
  //#define GRADIENT_MIX           // Support for gradient mixing with M166 and LCD    |支持M166和LCD的渐变混合
  #if ENABLED(GRADIENT_MIX)
    //#define GRADIENT_VTOOL       // Add M166 T to use a V-tool index as a Gradient alias    |添加 M166 T 以使用 V 工具索引作为渐变别名
  #endif
#endif

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).    |挤出机的偏移（如果使用多个挤出机并在更改时依靠固件定位，请取消注释）。 
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).    |对于挤出机 0 热端（默认挤出机），偏移量必须为 X=0、Y=0。
// For the other hotends it is their distance from the extruder 0 hotend.    |对于其他热端，它是它们距挤出机 0 热端的距离。
//#define HOTEND_OFFSET_X { 0.0, 20.00 } // (mm) relative X-offset for each nozzle    |(mm) 每个喷嘴的相对 X 偏移
//#define HOTEND_OFFSET_Y { 0.0, 5.00 }  // (mm) relative Y-offset for each nozzle    |(mm) 每个喷嘴的相对 Y 偏移
//#define HOTEND_OFFSET_Z { 0.0, 0.00 }  // (mm) relative Z-offset for each nozzle    |(mm) 每个喷嘴的相对 Z 偏移

// @section machine

/**
 *电源控制|
 *
 *启用电源并将其连接到 PS_ON_PIN。    |
 *指定电源是高电平有效还是低电平有效。    |
 */
//#define PSU_CONTROL
//#define PSU_NAME "Power Supply"

#if ENABLED(PSU_CONTROL)
  #define PSU_ACTIVE_STATE LOW      // ATX 设置为“低”，X-Box 设置为“高”|


  //#define PSU_DEFAULT_OFF //保持电源关闭状态，直到直接使用 M80 启用 |
  //#define PSU_POWERUP_DELAY 250 //(ms) PSU 预热到全功率的延迟 |


  //#define AUTO_POWER_CONTROL //启用 PS_ON 引脚的自动控制 |

  #if ENABLED(AUTO_POWER_CONTROL)
    #define AUTO_POWER_FANS         // 如果风扇需要电源，请打开 PSU |

    #define AUTO_POWER_E_FANS
    #define AUTO_POWER_CONTROLLERFAN
    #define AUTO_POWER_CHAMBER_FAN
    //#define AUTO_POWER_E_TEMP        50 // (°C) Turn on PSU over this temperature    |(°C) 在此温度下打开 PSU 
    //#define AUTO_POWER_CHAMBER_TEMP  30 // (°C) Turn on PSU over this temperature    |(°C) 在此温度下打开 PSU 
    #define POWER_TIMEOUT 30
  #endif
#endif

//===========================================================================
//============================= Thermal Settings ============================     |散热设置 
//===========================================================================
// @section temperature

/**
 * --NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
 *温度传感器
 TEMP_SENSOR_0 表示一号打印头的感温类型，TEMP_SENSOR_1 以此类推，设置为0表示不启用，其他设置为对应感温类型编号，具体查看下面代码的列表。现在的打印机一般使用的是100k感温电阻测温，所以类型编号选择1。
TEMP_SENSOR_BED 表示热床的感温类型，只需要在这里把热床的感温类型设置好，就可以开启热床功能，一般热床感温类型编号也是1，和打印头一样。
 * Temperature sensors available:
 *
 *    -5 : PT100 / PT1000 with MAX31865 (only for sensors 0-1)
 *    -3 : thermocouple with MAX31855 (only for sensors 0-1)  |带 MAX31855 的热电偶（仅适用于传感器 0-1）
 *    -2 : thermocouple with MAX6675 (only for sensors 0-1)
 *    -4 : thermocouple with AD8495
 *    -1 : thermocouple with AD595
 *     0 : not used
 *     1 : 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
 *   331 : (3.3V scaled thermistor 1 table for MEGA)
 *   332 : (3.3V scaled thermistor 1 table for DUE)
 *     2 : 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
 *   202 : 200k thermistor - Copymaster 3D
 *     3 : Mendel-parts thermistor (4.7k pullup)
 *     4 : 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!  |10K热敏电阻！！请勿将其用于热端。它在高温下分辨率很差。 !!
 *     5 : 100K thermistor - ATC Semitec 104GT-2/104NT-4-R025H42G (Used in ParCan, J-Head, and E3D) (4.7k pullup)
 *   501 : 100K Zonestar (Tronxy X3A) Thermistor
 *   502 : 100K Zonestar Thermistor used by hot bed in Zonestar Průša P802M
 *   512 : 100k RPW-Ultra hotend thermistor (4.7k pullup)
 *     6 : 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
 *     7 : 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
 *    71 : 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
 *     8 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
 *     9 : 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
 *    10 : 100k RS thermistor 198-961 (4.7k pullup)
 *    11 : 100k beta 3950 1% thermistor (Used in Keenovo AC silicone mats and most Wanhao i3 machines) (4.7k pullup)
 *    12 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
 *    13 : 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
 *    15 : 100k thermistor calibration for JGAurora A5 hotend
 *    18 : ATC Semitec 204GT-2 (4.7k pullup) Dagoma.Fr - MKS_Base_DKU001327
 *    20 : Pt100 with circuit in the Ultimainboard V2.x with 5v excitation (AVR)
 *    21 : Pt100 with circuit in the Ultimainboard V2.x with 3.3v excitation (STM32 \ LPC176x....)
 *    22 : 100k (hotend) with 4.7k pullup to 3.3V and 220R to analog input (as in GTM32 Pro vB)
 *    23 : 100k (bed) with 4.7k pullup to 3.3v and 220R to analog input (as in GTM32 Pro vB)
 *    30 : Kis3d Silicone heating mat 200W/300W with 6mm precision cast plate (EN AW 5083) NTC100K / B3950 (4.7k pullup)
 *   201 : Pt100 with circuit in Overlord, similar to Ultimainboard V2.x
 *    60 : 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
 *    61 : 100k Formbot / Vivedino 3950 350C thermistor 4.7k pullup
 *    66 : 4.7M High Temperature thermistor from Dyze Design
 *    67 : 450C thermistor from SliceEngineering
 *    70 : the 100K thermistor found in the bq Hephestos 2
 *    75 : 100k Generic Silicon Heat Pad with NTC 100K MGB18-104F39050L32 thermistor
 *    99 : 100k thermistor with a 10K pull-up resistor (found on some Wanhao i3 machines)
 *
 *       1k ohm pullup tables - This is atypical, and requires changing out the 4.7k pullup for 1k.
 *                              (but gives greater accuracy and more stable PID)
 *    51 : 100k thermistor - EPCOS (1k pullup)
 *    52 : 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
 *    55 : 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
 *
 *  1047 : Pt1000 with 4k7 pullup (E3D)
 *  1010 : Pt1000 with 1k pullup (non standard)
 *   147 : Pt100 with 4k7 pullup
 *   110 : Pt100 with 1k pullup (non standard)
 *
 *  1000 : Custom - Specify parameters in Configuration_adv.h
 *
 *         Use these for Testing or Development purposes. NEVER for production machine.    |将它们用于测试或开发目的。切勿用于生产机器
 *   998 : Dummy Table that ALWAYS reads 25°C or the temperature defined below.    |虚拟表始终显示 25°C 或下面定义的温度。
 *   999 : Dummy Table that ALWAYS reads 100°C or the temperature defined below.    |虚拟表始终显示 100°C 或下面定义的温度。
 */
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_4 0
#define TEMP_SENSOR_5 0
#define TEMP_SENSOR_6 0
#define TEMP_SENSOR_7 0
#define TEMP_SENSOR_BED 1
#define TEMP_SENSOR_PROBE 0
#define TEMP_SENSOR_CHAMBER 0

// Dummy thermistor constant temperature readings, for use with 998 and 999 ||虚拟热敏电阻恒温读数，与 998 和 999 配合使用
#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 100

// Resistor values when using a MAX31865 (sensor -5)  |使用 MAX31865 时的电阻值（传感器 -5）
// Sensor value is typically 100 (PT100) or 1000 (PT1000)    |传感器值通常为 100 (PT100) 或 1000 (PT1000)
// Calibration value is typically 430 ohm for AdaFruit PT100 modules and 4300 ohm for AdaFruit PT1000 modules.    |AdaFruit PT100 模块的校准值通常为 430 欧姆，AdaFruit PT1000 模块的校准值为 4300 欧姆。
//#define MAX31865_SENSOR_OHMS      100
//#define MAX31865_CALIBRATION_OHMS 430

// Use temp sensor 1 as a redundant sensor with sensor 0. If the readings    |使用温度传感器 1 作为传感器 0 的冗余传感器。如果读数
// from the two sensors differ too much the print will be aborted.    |两个传感器相差太大，打印将被中止。
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

#define TEMP_RESIDENCY_TIME     10  // (seconds) Time to wait for hotend to "settle" in M109    |（秒）等待热端在 M109 中“稳定”的时间
#define TEMP_WINDOW              1  // (°C) Temperature proximity for the "temperature reached" timer    |(°C) “温度达到”计时器的温度接近值
#define TEMP_HYSTERESIS          3  // (°C) Temperature proximity considered "close enough" to the target    |(°C) 温度接近度被视为与目标“足够接近”

#define TEMP_BED_RESIDENCY_TIME 10  // (seconds) Time to wait for bed to "settle" in M190    |（秒）等待床在 M190 中“安定”的时间
#define TEMP_BED_WINDOW          1  // (°C) Temperature proximity for the "temperature reached" timer    |(°C) “温度达到”计时器的温度接近值
#define TEMP_BED_HYSTERESIS      3  // (°C) Temperature proximity considered "close enough" to the target    |(°C) 温度接近度被视为与目标“足够接近”

// Below this temperature the heater will be switched off    |低于此温度，加热器将关闭
// because it probably indicates a broken thermistor wire.    | 因为它可能表明热敏电阻线损坏。
#define HEATER_0_MINTEMP   5
#define HEATER_1_MINTEMP   5
#define HEATER_2_MINTEMP   5
#define HEATER_3_MINTEMP   5
#define HEATER_4_MINTEMP   5
#define HEATER_5_MINTEMP   5
#define HEATER_6_MINTEMP   5
#define HEATER_7_MINTEMP   5
#define BED_MINTEMP        5

//最大温度
//HEATER_0_MAXTEMP 表示一号打印头的最大温度，HEATER_1_MAXTEMP 以此类推。BED_MAXTEMP 表示热床的最大温度。
//温度数值建议根据实际情况保守设置，以免设置过大损坏硬件，或造成安全隐患，推荐打印头250，热床100，足以使用。
// Above this temperature the heater will be switched off.    |高于此温度，加热器将关闭。
// This can protect components from overheating, but NOT from shorts and failures.    |这可以防止组件过热，但不能防止短路和故障。
// (Use MINTEMP for thermistor short/failure protection.)    |（使用 MITEMP 进行热敏电阻短路/故障保护。）
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define HEATER_4_MAXTEMP 275
#define HEATER_5_MAXTEMP 275
#define HEATER_6_MAXTEMP 275
#define HEATER_7_MAXTEMP 275
#define BED_MAXTEMP      150

//===========================================================================
//============================= PID Settings ================================    |
//===========================================================================
// PID Tuning Guide here: https://reprap.org/wiki/PID_Tuning   | PID 调节指南

// Comment the following line to disable PID and enable bang-bang.    |注释以下行以禁用 PID 并启用 bang-bang。 
#define PIDTEMP
#define BANG_MAX 255     // Limits current to nozzle while in bang-bang mode; 255=full current    |在 Bang-Bang 模式下限制流向喷嘴的电流； 255=全电流
#define PID_MAX BANG_MAX // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current    |当 PID 处于活动状态时限制喷嘴电流（参见下面的 PID_FUNCTIONAL_RANGE）； 255=全电流
#define PID_K1 0.95      // Smoothing factor within any PID loop    |任何 PID 环路中的平滑因子

#if ENABLED(PIDTEMP)
  //#define PID_EDIT_MENU         // Add PID editing to the "Advanced Settings" menu. (~700 bytes of PROGMEM)    |将PID编辑添加到“高级设置”菜单。 （~700 字节的 PROGMEM）
  //#define PID_AUTOTUNE_MENU     // Add PID auto-tuning to the "Advanced Settings" menu. (~250 bytes of PROGMEM)    |在“高级设置”菜单中添加PID自动整定。 （~250 字节的 PROGMEM）
  //#define PID_PARAMS_PER_HOTEND // Uses separate PID parameters for each extruder (useful for mismatched extruders)    |对每个挤出机使用单独的 PID 参数（对于不匹配的挤出机很有用）
                                  // Set/get with gcode: M301 E[extruder number, 0-2]    |使用 gcode 设置/获取：M301 E[挤出机编号，0-2] 

  #if ENABLED(PID_PARAMS_PER_HOTEND)
    // Specify between 1 and HOTENDS values per array.    |每个数组指定 1 到 HOTENDS 之间的值。
    // If fewer than EXTRUDER values are provided, the last element will be repeated.    |如果提供的值少于 EXTRUDER，则将重复最后一个元素。
    #define DEFAULT_Kp_LIST {  22.20,  22.20 }
    #define DEFAULT_Ki_LIST {   1.08,   1.08 }
    #define DEFAULT_Kd_LIST { 114.00, 114.00 }
  #else
    #define DEFAULT_Kp 15.25
    #define DEFAULT_Ki  0.97
    #define DEFAULT_Kd 59.78
  #endif
#endif // PIDTEMP    |

//===========================================================================
//====================== PID > Bed Temperature Control ======================    |PID > 热床温度控制
//===========================================================================

/**
 * PID Bed Heating    |PID 床加热
 *
 * If this option is enabled set PID constants below.    |如果启用此选项，请设置以下 PID 常数。 
 * If this option is disabled, bang-bang will be used and BED_LIMIT_SWITCHING will enable hysteresis.    |如果禁用此选项，将使用 bang-bang 并且 BED_LIMIT_SWITCHING 将启用迟滞。  
 *
 * The PID frequency will be the same as the extruder PWM.    |PID 频率与挤出机 PWM 频率相同。
 * If PID_dT is the default, and correct for the hardware/configuration, that means 7.689Hz,    |如果 PID_dT 是默认值，并且硬件/配置正确，则意味着 7.689Hz，
 * which is fine for driving a square wave into a resistive load and does not significantly    |这对于将方波驱动到电阻负载来说很好，但不会显着 
 * impact FET heating. This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W    |影响 FET 加热。这也适用于 Fotek SSR-10DA 固态继电器至 250W 
 * heater. If your configuration is significantly different than this and you don't understand    |加热器。如果您的配置与此明显不同并且您不明白
 * the issues involved, don't use bed PID until someone else verifies that your hardware works.    |所涉及的问题，在其他人验证您的硬件正常工作之前，不要使用床PID。 
 */
#define PIDTEMPBED

//#define BED_LIMIT_SWITCHING

/**
 * Max Bed Power    |最大床功率
 * Applies to all forms of bed control (PID, bang-bang, and bang-bang with hysteresis).    |适用于所有形式的床控制（PID、bang-bang 和带迟滞的 bang-bang）。
 * When set to any value below 255, enables a form of PWM to the bed that acts like a divider    |当设置为低于 255 的任何值时，将为床启用某种形式的 PWM，其作用类似于分频器
 * so don't use it unless you are OK with PWM on your bed. (See the comment on enabling PIDTEMPBED)    |因此，除非您同意在床上使用 PWM，否则请勿使用它。 （请参阅有关启用 PIDTEMPBED 的评论）
 */
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current    |将工作循环限制在床上； 255=全电流

#if ENABLED(PIDTEMPBED)
  //#define MIN_BED_POWER 0
  //#define PID_BED_DEBUG // Sends debug data to the serial port.    |将调试数据发送到串行端口。

  // Artillery Hornet    |
  #define DEFAULT_bedKp  92.75
  #define DEFAULT_bedKi  15.67
  #define DEFAULT_bedKd 366.04

  // FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.    |找到您自己的：“M303 E-1 C8 S90”在床上以 90 摄氏度运行自动调谐 8 个周期。
#endif // PIDTEMPBED

#if EITHER(PIDTEMP, PIDTEMPBED)
  //#define PID_DEBUG             // Sends debug data to the serial port. Use 'M303 D' to toggle activation.    |将调试数据发送到串行端口。使用“M303 D”切换激活。
  //#define PID_OPENLOOP          // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX    |将 PID 置于开环状态。 M104/M140 设置输出功率从 0 到 PID_MAX
  //#define SLOW_PWM_HEATERS      // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay    |具有非常低频率（大约 0.125Hz=8s）和大约 1s 的最小状态时间的 
  #define PID_FUNCTIONAL_RANGE 25 // If the temperature difference between the target temperature and the actual temperature    |PWM，对于继电器驱动的加热器很有用 
                                  // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.    |超过 PID_FUNCTIONAL_RANGE，则 PID 将关闭，加热器将设置为最小/最大。
#endif

// @section extruder

/**
 * 挤出机保护
 * EXTRUDE_MINTEMP 设置挤出机工作的最小温度，只有达到指定温度，挤出机电机才会转动，以此保护送丝轮挤不动造成磨损。所以如果发现挤出机不工作，请先查看打印头是否加热到指定温度。触摸屏不受此控制。
EXTRUDE_MAXLENGTH 设置挤出机挤出耗材的最大长度，防止误操作造成损失。
 * Prevent extrusion if the temperature is below EXTRUDE_MINTEMP.    |如果温度低于 EXTRUDE_MINTEMP，则防止挤出
 * Add M302 to set the minimum extrusion temperature and/or turn    |添加 M302 以设置最低挤出温度和/或转数
 * cold extrusion prevention on and off.    |冷挤压预防打开和关闭
 *
 * *** IT IS HIGHLY RECOMMENDED TO LEAVE THIS OPTION ENABLED! ***    |强烈建议启用此选项！
 */
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170

/**
 * Prevent a single extrusion longer than EXTRUDE_MAXLENGTH.    |防止单次挤出时间超过 EXTRUDE_MAXLENGTH。
 * Note: For Bowden Extruders make this large enough to allow load/unload.    |注意：对于 Bowden 挤出机，请使其足够大以允许装载/卸载。 
 */
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 650

//===========================================================================
//======================== Thermal Runaway Protection =======================    |热失控保护
//===========================================================================

/**
 * Thermal Protection provides additional protection to your printer from damage    |热保护为您的打印机提供额外保护，防止损坏
 * and fire. Marlin always includes safe min and max temperature ranges which    |和火。 Marlin 始终包含安全的最低和最高温度范围，其中 
 * protect against a broken or disconnected thermistor wire.    |防止热敏电阻线断裂或断开。  
 *
 * The issue: If a thermistor falls out, it will report the much lower    |问题：如果热敏电阻脱落，它将报告低得多的值
 * temperature of the air in the room, and the the firmware will keep    |房间内的空气温度，固件会保持|
 * the heater on.    |加热器打开。
 *
 * If you get "Thermal Runaway" or "Heating failed" errors the    |如果您收到“热失控”或“加热失败”错误，则 |
 * details can be tuned in Configuration_adv.h    |详细信息可以在Configuration_adv.h中调整|
 */

#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders    |为所有挤出机启用热保护 
#define THERMAL_PROTECTION_BED     // Enable thermal protection for the heated bed    |为加热床启用热保护
#define THERMAL_PROTECTION_CHAMBER // Enable thermal protection for the heated chamber    |为加热室启用热保护

//===========================================================================
//============================= Mechanical Settings =========================    | 机械设置 
//===========================================================================

// @section machine
//双轴联动结构
//如使用两个电机协同带两个轴的机型，如COREXY，COREXZ，COREYZ，去掉对应机型前面的注释即可。
// Enable one of the options below for CoreXY, CoreXZ, or CoreYZ kinematics,    | 为 CoreXY、CoreXZ 或 CoreYZ 运动学启用以下选项之一，
// either in the usual order or reversed    | 要么按照通常的顺序，要么相反
//#define COREXY
//#define COREXZ
//#define COREYZ
//#define COREYX
//#define COREZX
//#define COREZY
//#define MARKFORGED_XY  // MarkForged. See https://reprap.org/forum/read.php?152,504042 |马克锻造

//===========================================================================
//============================== Endstop Settings ===========================    |限位设置
//===========================================================================

// @section homing
//软限位
// Specify here all the endstop connectors that are connected to any endstop or probe.    |此处指定连接到任何终点挡块或探头的所有终点挡块连接器。
// Almost all printers will be using one per axis. Probes will use one or more of the    |几乎所有打印机每轴都使用一个。探针将使用一个或多个
// extra connectors. Leave undefined any used for non-endstop and non-probe purposes.    |额外的连接器。保留未定义的任何用于非终点和非探测目的。 
//#define USE_XMIN_PLUG
#define USE_YMIN_PLUG
#define USE_ZMIN_PLUG
#define USE_XMAX_PLUG
//#define USE_YMAX_PLUG
//#define USE_ZMAX_PLUG


//限位开关上拉
//ENDSTOPPULLUPS 去掉注释的话表示所有限位开关上拉，上拉表示对应引脚悬空的情况下默认是高电平，即限位开关开路状态下是H电平状态。
//Makeboard系列主板必须开启此项。如注释掉此项的话，可在下面代码单独配置XYZ轴MAX和MIN限位开关上拉状态。
//如去掉 ENDSTOPPULLUP_XMAX 注释可单独开启X-MAX限位开关上拉。
// Enable pullup for all endstops to prevent a floating state    | 为所有终点启用上拉以防止浮动状态
#define ENDSTOPPULLUPS
#if DISABLED(ENDSTOPPULLUPS)
  // Disable ENDSTOPPULLUPS to set pullups individually    |禁用 ENDSTOPPULLUPS 以单独设置上拉电阻
  //#define ENDSTOPPULLUP_XMAX
  //#define ENDSTOPPULLUP_YMAX
  //#define ENDSTOPPULLUP_ZMAX
  //#define ENDSTOPPULLUP_XMIN
  //#define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
  //#define ENDSTOPPULLUP_ZMIN_PROBE
#endif

// Enable pulldown for all endstops to prevent a floating state    |为所有终点启用下拉功能以防止浮动状态
//#define ENDSTOPPULLDOWNS
#if DISABLED(ENDSTOPPULLDOWNS)
  // Disable ENDSTOPPULLDOWNS to set pulldowns individually    |禁用 ENDSTOPPULLDOWNS 以单独设置下拉
  //#define ENDSTOPPULLDOWN_XMAX
  //#define ENDSTOPPULLDOWN_YMAX
  //#define ENDSTOPPULLDOWN_ZMAX
  //#define ENDSTOPPULLDOWN_XMIN
  //#define ENDSTOPPULLDOWN_YMIN
  //#define ENDSTOPPULLDOWN_ZMIN
  //#define ENDSTOPPULLDOWN_ZMIN_PROBE
#endif
//限位开关信号
//X_MIN_ENDSTOP_INVERTING 等系列参数设置为 true 表示将限位开关的信号反转，针对限位开关的常开和长闭状态，如触发状态不符合预期，可在此处修正。
//Z_MIN_PROBE_ENDSTOP_INVERTING 表示自动调平使用的探针电平状态，如不时触底时才触发，可在此反转。
// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).    |COM 接地且 NC 信号连接的机械限位器在此使用“假”（最常见的设置）。 
#define X_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.    |设置为 true 可反转终点挡板的逻辑。 
#define Y_MIN_ENDSTOP_INVERTING true  // Set to true to invert the logic of the endstop.    |
#define Z_MIN_ENDSTOP_INVERTING true  // Set to true to invert the logic of the endstop.    |
#define X_MAX_ENDSTOP_INVERTING true  // Set to true to invert the logic of the endstop.    |
#define Y_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.    |
#define Z_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.    |
#define Z_MIN_PROBE_ENDSTOP_INVERTING true // Set to true to invert the logic of the probe.    |设置为 true 可反转探针的逻辑。

/**
 * Stepper Drivers    |步进驱动器
 *
 * These settings allow Marlin to tune stepper driver timing and enable advanced options for    |这些设置允许 Marlin 调整步进驱动器时序并启用高级选项
 * stepper drivers that support them. You may also override timing options in Configuration_adv.h.    |支持它们的步进驱动器。您还可以覆盖 Configuration_adv.h 中的计时选项
 *
 * A4988 is assumed for unspecified drivers.    |假定 A4988 用于未指定的驱动程序。
 *
 * Options: A4988, A5984, DRV8825, LV8729, L6470, L6474, POWERSTEP01,
 *          TB6560, TB6600, TMC2100,
 *          TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE,
 *          TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE,
 *          TMC26X,  TMC26X_STANDALONE,  TMC2660, TMC2660_STANDALONE,
 *          TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE
 * :['A4988', 'A5984', 'DRV8825', 'LV8729', 'L6470', 'L6474', 'POWERSTEP01', 'TB6560', 'TB6600', 'TMC2100', 'TMC2130', 'TMC2130_STANDALONE', 'TMC2160', 'TMC2160_STANDALONE', 'TMC2208', 'TMC2208_STANDALONE', 'TMC2209', 'TMC2209_STANDALONE', 'TMC26X', 'TMC26X_STANDALONE', 'TMC2660', 'TMC2660_STANDALONE', 'TMC5130', 'TMC5130_STANDALONE', 'TMC5160', 'TMC5160_STANDALONE']
 */
#define X_DRIVER_TYPE  TMC2100
#define Y_DRIVER_TYPE  TMC2100
#define Z_DRIVER_TYPE  TMC2100
//#define X2_DRIVER_TYPE A4988
//#define Y2_DRIVER_TYPE A4988
//#define Z2_DRIVER_TYPE A4988
//#define Z3_DRIVER_TYPE A4988
//#define Z4_DRIVER_TYPE A4988
#define E0_DRIVER_TYPE TMC2100
//#define E1_DRIVER_TYPE A4988
//#define E2_DRIVER_TYPE A4988
//#define E3_DRIVER_TYPE A4988
//#define E4_DRIVER_TYPE A4988
//#define E5_DRIVER_TYPE A4988
//#define E6_DRIVER_TYPE A4988
//#define E7_DRIVER_TYPE A4988

// Enable this feature if all enabled endstop pins are interrupt-capable.    |如果所有启用的终点停止引脚都具有中断功能，则启用此功能。
// This will remove the need to poll the interrupt pins, saving many CPU cycles.    |这将消除轮询中断引脚的需要，从而节省许多 CPU 周期。
//#define ENDSTOP_INTERRUPTS_FEATURE

/**
 * Endstop Noise Threshold    |终点停止噪声阈值
 *
 * Enable if your probe or endstops falsely trigger due to noise.    |如果您的探头或限位器由于噪音而错误触发，请启用。
 *
 * - Higher values may affect repeatability or accuracy of some bed probes.    |较高的值可能会影响某些床探头的重复性或准确性。
 * - To fix noise install a 100nF ceramic capacitor in parallel with the switch.    |为了消除噪声，请与开关并联安装一个 100nF 陶瓷电容器。 
 * - This feature is not required for common micro-switches mounted on PCBs    |安装在 PCB 上的常见微动开关不需​​要此功能
 *   based on the Makerbot design, which already have the 100nF capacitor.    |基于 Makerbot 设计，已配备 100nF 电容器。
 *
 * :[2,3,4,5,6,7]
 */
//#define ENDSTOP_NOISE_THRESHOLD 2

// Check for stuck or disconnected endstops during homing moves.    |在复位移动期间检查终端止动件是否卡住或断开。
//#define DETECT_BROKEN_ENDSTOP

//=============================================================================
//============================== Movement Settings ============================    |运动设置
//=============================================================================
// @section motion

/**
 * Default Settings    |默认设置
 *
 * These settings can be reset by M502    |这些设置可以通过M502重置
 *
 * Note that if EEPROM is enabled, saved values will override these.    |请注意，如果启用了 EEPROM，则保存的值将覆盖这些值。
 */

/**
 * With this option each E stepper can have its own factors for the    |使用此选项，每个 E 步进器都可以有自己的 因子。
 * following movement settings. If fewer factors are given than the    |以下运动设置。如果给出的因素少于
 * total number of extruders, the last value applies to the rest.    |挤出机总数，最后一个值适用于其余挤出机。  
 */
//#define DISTINCT_E_FACTORS

/**
 * 电机步进数
 * DEFAULT_AXIS_STEPS_PER_UNIT 后面的四个数字 {80,80,3200,100} ，分别表示XYZ和挤出机电机的步进数。
 * XYZ电机步进公式为：

(360 / 电机步距角 * 细分数 ) / (同步带齿距 * 齿数)
一般现在3D打印机使用的42步进电机步距角为1.8度，细分数根据主板上跳帽的设置，一般设置成16，同步带齿距一般使用的是2mm的，齿数一般为16或者20齿，可以自己数数。如果Z轴使丝杠的话，直接除以丝杆的导程即可。这样默认情况下可以算得：

(360 / 1.8 * 16) / (2 * 20) = 80
挤出机步进数没有很精确的计算公式，目前通常使用的近程挤出机设置为100，远程挤出机设置为150，然后根据实际打印的出丝多少微调，要出多点步进数就调大，出少点就调小。
 * Default Axis Steps Per Unit (steps/mm)  |每单位默认轴步数（步数/毫米）
 * Override with M92    |用 M92 覆盖
 *                                      X, Y, Z, E0 [, E1[, E2...]]
 */
#define DEFAULT_AXIS_STEPS_PER_UNIT   { 80.121, 80.121, 400, 445 }

/**
 * 电机最大加速度
 * DEFAULT_MAX_ACCELERATION 后面的四个数字 {3000,3000,100,10000} ，分别表示XYZ和挤出机电机的最大加速度。
 * 三角洲机型的加速度可以设置的大些，其他机型小些，原则也使实际测试，并不是越大越好，大的话会增加机器的抖动。
 * Default Max Feed Rate (mm/s)    |默认最大速率（毫米/秒）
 * Override with M203    |用M203覆盖
 *                                      X, Y, Z, E0 [, E1[, E2...]]
 */
#define DEFAULT_MAX_FEEDRATE          { 300, 300, 40, 50 }

//#define LIMITED_MAX_FR_EDITING        // Limit edit via M203 or LCD to DEFAULT_MAX_FEEDRATE * 2    |通过 M203 或 LCD 将编辑限制为 DEFAULT_MAX_FEEDRATE *2
#if ENABLED(LIMITED_MAX_FR_EDITING)
  #define MAX_FEEDRATE_EDIT_VALUES    { 600, 600, 10, 50 } // ...or, set your own edit limits    |或者，设置您自己的编辑限制
#endif

/**
 * Default Max Acceleration (change/s) change = mm/s    |默认最大加速度（变化/秒）变化 = mm/s
 * (Maximum start speed for accelerated moves)    |（加速移动的最大启动速度）
 * Override with M201    |用 M201 覆盖 
 *                                      X, Y, Z, E0 [, E1[, E2...]]
 */
#define DEFAULT_MAX_ACCELERATION      { 2000, 2000, 100, 10000 }

//#define LIMITED_MAX_ACCEL_EDITING     // Limit edit via M201 or LCD to DEFAULT_MAX_ACCELERATION * 2    |通过 M201 或 LCD 将编辑限制为 DEFAULT_MAX_ACCELERATION *2
#if ENABLED(LIMITED_MAX_ACCEL_EDITING)
  #define MAX_ACCEL_EDIT_VALUES       { 6000, 6000, 200, 20000 } // ...or, set your own edit limits
#endif

/**
 * 电机默认加速度
 * DEFAULT_ACCELERATION 设置X，Y，Z和E轴电机的默认移动加速度。
 * DEFAULT_RETRACT_ACCELERATION 设置E轴电机回抽时候的默认加速度。
 * DEFAULT_TRAVEL_ACCELERATION* 设置E轴电机挤出时候的默认加速度。
 * Default Acceleration (change/s) change = mm/s
 * Override with M204
 *
 *   M204 P    Acceleration 加速
 *   M204 R    Retract Acceleration  回抽加速
 *   M204 T    Travel Acceleration  行驶加速度
 */
#define DEFAULT_ACCELERATION          800    // X, Y, Z and E acceleration for printing moves    |打印移动的 X、Y、Z 和 E 加速度
#define DEFAULT_RETRACT_ACCELERATION  10000    // E acceleration for retracts    |E 缩回加速度
#define DEFAULT_TRAVEL_ACCELERATION   2000    // X, Y, Z acceleration for travel (non printing) moves    |行进（非打印）移动的 X、Y、Z 加速度

/**
 * Default Jerk limits (mm/s)    |默认加加速度限制 (mm/s)
 * Override with M205 X Y Z E    |使用 M205 X Y Z E 覆盖
 *
 * "Jerk" specifies the minimum speed change that requires acceleration.    |“Jerk”指定需要加速的最小速度变化。
 * When changing speed and direction, if the difference is less than the    |当改变速度和方向时，如果差值小于
 * value set here, it may happen instantaneously.    |此处设置的值，可能会瞬间发生。
 */
//#define CLASSIC_JERK
#if ENABLED(CLASSIC_JERK)
  #define DEFAULT_XJERK 10.0
  #define DEFAULT_YJERK 10.0
  #define DEFAULT_ZJERK  0.3

  //#define TRAVEL_EXTRA_XYJERK 0.0     // Additional jerk allowance for all travel moves |所有行程移动的附加加加速度津贴

  //#define LIMITED_JERK_EDITING        // Limit edit via M205 or LCD to DEFAULT_aJERK * 2  |通过 M205 或 LCD 限制编辑为 DEFAULT_aJERK *2
  #if ENABLED(LIMITED_JERK_EDITING)
    #define MAX_JERK_EDIT_VALUES { 20, 20, 0.6, 10 } // ...or, set your own edit limits
  #endif
#endif

#define DEFAULT_EJERK    5.0  // May be used by Linear Advance  |可由 Linear Advance 使用

/**
 * Junction Deviation Factor    |结点偏差系数
 *
 * See:
 *   https://reprap.org/forum/read.php?1,739819
 *   https://blog.kyneticcnc.com/2018/10/computing-junction-deviation-for-marlin.html
 */
#if DISABLED(CLASSIC_JERK)
  #define JUNCTION_DEVIATION_MM 0.032 // (mm) Distance from real junction edge    | (mm) 距实际交界边缘的距离 
  #define JD_HANDLE_SMALL_SEGMENTS    // Use curvature estimation instead of just the junction angle    | 使用曲率估计而不仅仅是连接角
                                      // for small segments (< 1mm) with large junction angles (> 135°).    |适用于具有大接合角 (> 135°) 的小段 (< 1mm)。
#endif

/**
 * S-Curve Acceleration    |S 曲线加速
 *
 * This option eliminates vibration during printing by fitting a Bézier    |此选项通过安装贝塞尔曲线消除打印过程中的振动
 * curve to move acceleration, producing much smoother direction changes.    |曲线移动加速度，产生更平滑的方向变化。
 *
 * See https://github.com/synthetos/TinyG/wiki/Jerk-Controlled-Motion-Explained
 */
#define S_CURVE_ACCELERATION

//===========================================================================
//============================= Z Probe Options =============================    |Z 探头选项
//===========================================================================
// @section probes

//
// See https://marlinfw.org/docs/configuration/probes.html
//

/**
 * Enable this option for a probe connected to the Z-MIN pin.    |为连接到 Z-MIN 引脚的探头启用此选项。 
 * The probe replaces the Z-MIN endstop and is used for Z homing.    |探头取代了 Z-MIN 限位器并用于 Z 复位。
 * (Automatically enables USE_PROBE_FOR_Z_HOMING.)    |（自动启用 USE_PROBE_FOR_Z_HOMING。） 
 */
//#define Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN

// Force the use of the probe for Z-axis homing    |强制使用探针进行Z轴复位
#define USE_PROBE_FOR_Z_HOMING

/**
 * Z_MIN_PROBE_PIN    |
 *
 * Define this pin if the probe is not connected to Z_MIN_PIN.    |如果探头未连接到 Z_MIN_PIN，请定义此引脚
 * If not defined the default pin for the selected MOTHERBOARD    |如果未定义所选主板的默认引脚
 * will be used. Most of the time the default is what you want.    |将被使用。大多数时候，默认值就是您想要的。
 *
 *  - The simplest option is to use a free endstop connector.    |最简单的选择是使用免费的终端止动连接器。
 *  - Use 5V for powered (usually inductive) sensors.    |对供电（通常是电感）传感器使用 5V。
 *
 *  - RAMPS 1.3/1.4 boards may use the 5V, GND, and Aux4->D32 pin:    |RAMPS 1.3/1.4 板可以使用 5V、GND 和 Aux4->D32 引脚：
 *    - For simple switches connect...    |对于简单的开关连接...
 *      - normally-closed switches to GND and D32.    |常闭开关连接至 GND 和 D32。
 *      - normally-open switches to 5V and D32.    |常开开关为 5V 和 D32。
 */
//#define Z_MIN_PROBE_PIN 32 // Pin 32 is the RAMPS default    |引脚 32 是 RAMPS 默认值
#define Z_MIN_PROBE_PIN PC2
/**
 * Probe Type    |探头类型
 *
 * Allen Key Probes, Servo Probes, Z-Sled Probes, FIX_MOUNTED_PROBE, etc.    |六角扳手探针、伺服探针、Z-Sled 探针、FIX_MOUNTED_PROBE 等 
 * Activate one of these to use Auto Bed Leveling below.    |激活其中一项以使用下面的自动床调平功能。
 */

/**
 * The "Manual Probe" provides a means to do "Auto" Bed Leveling without a probe.    |“手动探头”提供了一种无需探头即可“自动”床调平的方法。 
 * Use G29 repeatedly, adjusting the Z height at each point with movement commands    |重复使用G29，用移动指令调整各点的Z高度
 * or (with LCD_BED_LEVELING) the LCD controller.    |或（使用 LCD_BED_LEVELING）LCD 控制器。
 */
//#define PROBE_MANUALLY
//#define MANUAL_PROBE_START_Z 0.2

/**
 * A Fix-Mounted Probe either doesn't deploy or needs manual deployment.    |固定安装的探头要么不部署，要么需要手动部署。
 *   (e.g., an inductive probe or a nozzle-based probe-switch.)    |（例如，感应探头或基于喷嘴的探头开关。）
 */
//#define FIX_MOUNTED_PROBE

/**
 * Use the nozzle as the probe, as with a conductive    |使用喷嘴作为探针，就像使用导电
 * nozzle system or a piezo-electric smart effector.    |喷嘴系统或压电智能执行器。 
 */
//#define NOZZLE_AS_PROBE

/**
 * Z Servo Probe, such as an endstop switch on a rotating arm.    |Z 伺服探头，例如旋转臂上的限位开关。
 */
//#define Z_PROBE_SERVO_NR 0       // Defaults to SERVO 0 connector.    |默认为 SERVO 0 连接器。
//#define Z_SERVO_ANGLES { 70, 0 } // Z Servo Deploy and Stow angles    |Z 伺服部署和收起角度

/**
 * The BLTouch probe uses a Hall effect sensor and emulates a servo.    |BLTouch 探头使用霍尔效应传感器并模拟伺服系统。 
 */
#define BLTOUCH

/**
 * Pressure sensor with a BLTouch-like interface    |具有类似 BLTouch 接口的压力传感器
 */
//#define CREALITY_TOUCH

/**
 * Touch-MI Probe by hotends.fr    |Touch-MI 探针，作者：hotends.fr
 *
 * This probe is deployed and activated by moving the X-axis to a magnet at the edge of the bed.    |通过将 X 轴移动到床边缘的磁铁来部署和激活该探头。 
 * By default, the magnet is assumed to be on the left and activated by a home. If the magnet is    |默认情况下，假定磁铁位于左侧并由家庭激活。如果磁铁是 
 * on the right, enable and set TOUCH_MI_DEPLOY_XPOS to the deploy position.    |在右侧，启用 TOUCH_MI_DEPLOY_XPOS 并将其设置为部署位置。
 *
 * Also requires: BABYSTEPPING, BABYSTEP_ZPROBE_OFFSET, Z_SAFE_HOMING,    |还需要：BABYSTEPPING、BABYSTEP_ZPROBE_OFFSET、Z_SAFE_HOMING、
 *                and a minimum Z_HOMING_HEIGHT of 10.    |且最小 Z_HOMING_HEIGHT 为 10。
 */
//#define TOUCH_MI_PROBE
#if ENABLED(TOUCH_MI_PROBE)
  #define TOUCH_MI_RETRACT_Z 0.5                  // Height at which the probe retracts    |探头缩回高度
  //#define TOUCH_MI_DEPLOY_XPOS (X_MAX_BED + 2)  // For a magnet on the right side of the bed    |用于床右侧的磁铁
  //#define TOUCH_MI_MANUAL_DEPLOY                // For manual deploy (LCD menu)    |用于手动部署（LCD 菜单）
#endif

// A probe that is deployed and stowed with a solenoid pin (SOL1_PIN)    |使用电磁阀销 (SOL1_PIN) 展开和收起的探头
//#define SOLENOID_PROBE

// A sled-mounted probe like those designed by Charles Bell.    |类似于查尔斯·贝尔设计的雪橇式探头。
//#define Z_PROBE_SLED
//#define SLED_DOCKING_OFFSET 5  // The extra distance the X axis must travel to pickup the sled. 0 should be fine but you can push it further if you'd like.    |X 轴必须移动以拾取雪橇的额外距离。 0 应该没问题，但如果您愿意，您可以进一步推动它。

// A probe deployed by moving the x-axis, such as the Wilson II's rack-and-pinion probe designed by Marty Rice.    |通过移动 x 轴来部署的探头，例如 Marty Rice 设计的 Wilson II 齿轮齿条探头。
//#define RACK_AND_PINION_PROBE
#if ENABLED(RACK_AND_PINION_PROBE)
  #define Z_PROBE_DEPLOY_X  X_MIN_POS
  #define Z_PROBE_RETRACT_X X_MAX_POS
#endif

// Duet Smart Effector (for delta printers) - https://bit.ly/2ul5U7J    |Duet 智能效应器
// When the pin is defined you can use M672 to set/reset the probe sensivity.    | 定义引脚后，您可以使用 M672 设置/重置探头灵敏度。 
//#define DUET_SMART_EFFECTOR
#if ENABLED(DUET_SMART_EFFECTOR)
  #define SMART_EFFECTOR_MOD_PIN  -1  // Connect a GPIO pin to the Smart Effector MOD pin    |将 GPIO 引脚连接到智能效应器 MOD 引脚
#endif

/**
 * Use StallGuard2 to probe the bed with the nozzle.    |使用 StallGuard2 用喷嘴探测床。 
 * Requires stallGuard-capable Trinamic stepper drivers.    |需要具有stallGuard 功能的Trinamic 步进驱动器。
 * CAUTION: This can damage machines with Z lead screws.    |注意：这可能会损坏带有 Z 丝杠的机器。
 *          Take extreme care when setting up this feature.    |设置此功能时要格外小心。
 */
//#define SENSORLESS_PROBING

//
// For Z_PROBE_ALLEN_KEY see the Delta example configurations.    |对于 Z_PROBE_ALLEN_KEY，请参阅 Delta 示例配置。
//

/**
 * Nozzle-to-Probe offsets { X, Y, Z }    |喷嘴到探头的偏移
 *
 * - Use a caliper or ruler to measure the distance from the tip of    |使用卡尺或直尺测量距尖端的距离
 *   the Nozzle to the center-point of the Probe in the X and Y axes.    |喷嘴到 X 轴和 Y 轴上探针的中心点
 * - For the Z offset use your best known value and adjust at runtime.    |对于 Z 偏移，请使用您最熟悉的值并在运行时进行调整。
 * - Probe Offsets can be tuned at runtime with 'M851', LCD menus, babystepping, etc.    |探头偏移可以在运行时使用“M851”、LCD 菜单、babystepping 等进行调整。
 *
 * Assuming the typical work area orientation:    |假设典型的工作区域方向：
 *  - Probe to RIGHT of the Nozzle has a Positive X offset    |喷嘴右侧的探头具有正 X 偏移
 *  - Probe to LEFT  of the Nozzle has a Negative X offset    |
 *  - Probe in BACK  of the Nozzle has a Positive Y offset    |
 *  - Probe in FRONT of the Nozzle has a Negative Y offset    |
 *
 * Some examples:
 *   #define NOZZLE_TO_PROBE_OFFSET { 10, 10, -1 }   // Example "1"
 *   #define NOZZLE_TO_PROBE_OFFSET {-10,  5, -1 }   // Example "2"
 *   #define NOZZLE_TO_PROBE_OFFSET {  5, -5, -1 }   // Example "3"
 *   #define NOZZLE_TO_PROBE_OFFSET {-15,-10, -1 }   // Example "4"
 *
 *     +-- BACK ---+
 *     |    [+]    |
 *   L |        1  | R <-- Example "1" (right+,  back+)
 *   E |  2        | I <-- Example "2" ( left-,  back+)
 *   F |[-]  N  [+]| G <-- Nozzle
 *   T |       3   | H <-- Example "3" (right+, front-)
 *     | 4         | T <-- Example "4" ( left-, front-)
 *     |    [-]    |
 *     O-- FRONT --+
 */
#define NOZZLE_TO_PROBE_OFFSET { 0, -34, 0 }

// Most probes should stay away from the edges of the bed, but    |大多数探头应远离床的边缘，但是
// with NOZZLE_AS_PROBE this can be negative for a wider probing area.    |对于 NOZZLE_AS_PROBE，这对于更广泛的探测区域可能是负值。
#define PROBING_MARGIN 10

// X and Y axis travel speed (mm/min) between probes    |探头之间的 X 和 Y 轴移动速度 (mm/min) 
#define XY_PROBE_SPEED (133*60)

// Feedrate (mm/min) for the first approach when double-probing (MULTIPLE_PROBING == 2)    |双探测时第一种方法的进给率 (mm/min) (MULTIPLE_PROBING == 2)
#define Z_PROBE_SPEED_FAST HOMING_FEEDRATE_Z

// Feedrate (mm/min) for the "accurate" probe of each point    | 每个点“精确”探头的进给率（毫米/分钟）
#define Z_PROBE_SPEED_SLOW (Z_PROBE_SPEED_FAST / 2)

/**
 * Multiple Probing    |多重探测
 *
 * You may get improved results by probing 2 or more times.    |探测 2 次或更多次可能会得到更好的结果。
 * With EXTRA_PROBING the more atypical reading(s) will be disregarded.    |使用 EXTRA_PROBING 时，更非典型的读数将被忽略。
 *
 * A total of 2 does fast/slow probes with a weighted average.    |总共 2 个带加权平均值的快/慢探测。
 * A total of 3 or more adds more slow probes, taking the average.    |总共 3 个或更多添加更慢的探针，取平均值。
 */
//#define MULTIPLE_PROBING 2
//#define EXTRA_PROBING    1

/**
 * Z probes require clearance when deploying, stowing, and moving between    |Z 探头在部署、收起和在 之间移动时需要间隙。
 * probe points to avoid hitting the bed and other hardware.    |探针点避免撞击床和其他硬件。
 * Servo-mounted probes require extra space for the arm to rotate.    |伺服安装探头需要额外的空间供臂旋转。 
 * Inductive probes need space to keep from triggering early.    |感应式探头需要空间以防止过早触发。
 *
 * Use these settings to specify the distance (mm) to raise the probe (or    |使用这些设置指定升高探头的距离 (mm)（或 
 * lower the bed). The values set here apply over and above any (negative)    |降低床）。此处设置的值适用于任何（负）
 * probe Z Offset set with NOZZLE_TO_PROBE_OFFSET, M851, or the LCD.    |使用 NOZZLE_TO_PROBE_OFFSET、M851 或 LCD 设置探头 Z 偏移。
 * Only integer values >= 1 are valid here.    |此处仅整数值 >= 1 有效。 
 *
 * Example: `M851 Z-5` with a CLEARANCE of 4  =>  9mm from bed to nozzle.    |示例：“M851 Z-5”，从床到喷嘴的间隙为 4 => 9mm。
 *     But: `M851 Z+1` with a CLEARANCE of 2  =>  2mm from bed to nozzle.    |但是：“M851 Z+1”，从床到喷嘴的间隙为 2 => 2mm。
 */
#define Z_CLEARANCE_DEPLOY_PROBE   10 // Z Clearance for Deploy/Stow    |展开/收起的 Z 间隙
#define Z_CLEARANCE_BETWEEN_PROBES  5 // Z Clearance between probe points    |Z 探头点之间的间隙 
#define Z_CLEARANCE_MULTI_PROBE     5 // Z Clearance between multiple probes    |多个探头之间的 Z 间隙
//#define Z_AFTER_PROBING           5 // Z position after probing is done    |探测完成后的 Z 位置

#define Z_PROBE_LOW_POINT          -2 // Farthest distance below the trigger-point to go before stopping    |停止前低于触发点的最远距离

// For M851 give a range for adjusting the Z probe offset    |对于M851，给出调整Z探针偏移的范围
#define Z_PROBE_OFFSET_RANGE_MIN -20
#define Z_PROBE_OFFSET_RANGE_MAX 20

// Enable the M48 repeatability test to test probe accuracy    |启用 M48 重复性测试来测试探头精度
//#define Z_MIN_PROBE_REPEATABILITY_TEST

// Before deploy/stow pause for user confirmation    |在部署/存放之前暂停以供用户确认
//#define PAUSE_BEFORE_DEPLOY_STOW
#if ENABLED(PAUSE_BEFORE_DEPLOY_STOW)
  //#define PAUSE_PROBE_DEPLOY_WHEN_TRIGGERED // For Manual Deploy Allenkey Probe    |用于手动部署 Allenkey 探针
#endif

/**
 * Enable one or more of the following if probing seems unreliable.    |如果探测看起来不可靠，请启用以下一项或多项。  
 * Heaters and/or fans can be disabled during probing to minimize electrical    |在探测过程中可以禁用加热器和/或风扇，以最大限度地减少电气
 * noise. A delay can also be added to allow noise and vibration to settle.    |噪音。还可以添加延迟以使噪音和振动稳定下来。 
 * These options are most useful for the BLTouch probe, but may also improve    |这些选项对于 BLTouch 探头最有用，但也可能会改进
 * readings with inductive probes and piezo sensors.    |使用电感式探头和压电传感器进行读数。
 */
//#define PROBING_HEATERS_OFF       // Turn heaters off when probing    |探测时关闭加热器
#if ENABLED(PROBING_HEATERS_OFF)
  //#define WAIT_FOR_BED_HEATER     // Wait for bed to heat back up between probes (to improve accuracy)    |等待床在探头之间重新加热（以提高准确性）
#endif
//#define PROBING_FANS_OFF          // Turn fans off when probing    |探测时关闭风扇
//#define PROBING_STEPPERS_OFF      // Turn steppers off (unless needed to hold position) when probing    |探测时关闭步进器（除非需要保持位置）
//#define DELAY_BEFORE_PROBING 200  // (ms) To prevent vibrations from triggering piezo sensors    |(ms) 防止振动触发压电传感器

//电机使能信号
//X_ENABLE_ON 等系列参数设置为 0 表示电机是低电平使能，1 表示高电平使能。Makeboard配套驱动芯片均为低电平使能，默认设置即可。如外接驱动器，如果电机不工作，不锁死，可将此参数设为 1 尝试。
//
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1    |对于反相步进器使能引脚（低电平有效）使用 0，非反相（高电平有效）使用 1
// :{ 0:'Low', 1:'High' }
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders    |

//禁用电机
//如特殊用途，如激光切割机一类，将指定轴，如 DISABLE_Z 设置为 ture 即可禁用Z轴电机。
// Disable axis steppers immediately when they're not being stepped.    |当轴步进器不被步进时立即禁用它们。
// WARNING: When motors turn off there is a chance of losing position accuracy!    |警告：当电机关闭时，可能会失去位置精度！
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false

// Turn off the display blinking that warns about possible accuracy reduction    |关闭警告可能会降低精度的显示屏闪烁
//#define DISABLE_REDUCED_ACCURACY_WARNING    |

// @section extruder

#define DISABLE_E false             // Disable the extruder when not stepping    |不步进时禁用挤出机
#define DISABLE_INACTIVE_EXTRUDER   // Keep only the active extruder enabled    |仅保持活动挤出机启用

// @section machine
//电机运动方向
//如果复位时候，打印头不是朝限位开关方向移动，可将对应轴，如 INVERT_X_DIR 设置为 true 即可反转X轴电机运动方向。
//如果挤出机电机挤出和回抽动作是反，将对应挤出机，如 INVERT_E0_DIR 设置为 true 即可反转一号挤出机运动方向。
// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.    |反转步进器方向。如果轴方向错误，请更换（或反转电机连接器）。
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false

// @section extruder

// For direct drive extruder v9 set to true, for geared extruder set to false.    |对于直接驱动挤出机 v9 设置为 true，对于齿轮传动挤出机设置为 false。
#define INVERT_E0_DIR true
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
#define INVERT_E6_DIR false
#define INVERT_E7_DIR false

// @section homing

//#define NO_MOTION_BEFORE_HOMING // Inhibit movement until all axes have been homed    |禁止移动，直到所有轴都已复位

//#define UNKNOWN_Z_NO_RAISE      // Don't raise Z (lower the bed) if Z is "unknown." For beds that fall when Z is powered off.    |如果 Z 为“未知”，请勿升高 Z（降低床）。适用于 Z 断电时掉落的床。

//#define Z_HOMING_HEIGHT  4      // (mm) Minimal Z height before homing (G28) for Z clearance above the bed, clamps, ...    |(mm) 回原点 (G28) 之前的最小 Z 高度，用于床身、夹具等上方的 Z 间隙
                                  // Be sure to have this much clearance over your Z_MAX_POS to prevent grinding.    |请务必在 Z_MAX_POS 上留出足够的间隙，以防止磨损。

//#define Z_AFTER_HOMING  10      // (mm) Height to move to after homing Z    |(mm) 复位后移动到的高度 Z

//复位方向
//设置各轴复位时触发的限位开关，三角洲机型设置为 1，复位时最大值，三轴限位开关插在MAX接口。非三角洲机型一般设置为 -1，复位时为最小值，复位后坐标为 0,0,0，三轴限位开关插在MIN接口。
// Direction of endstops when homing; 1=MAX, -1=MIN    |复位时终点挡块的方向； 1=最大值，-1=最小值
// :[-1,1]
#define X_HOME_DIR 1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

// @section machine
// The size of the print bed    |热床尺寸
#define X_BED_SIZE 220
#define Y_BED_SIZE 220
//复位坐标
//设置打印机的打印范围，X_MIN_POS，Y_MIN_POS，Z_MIN_POS，为打印机最小值方向复位的坐标，一般默认设置为 0 即可。
//X_MAX_POS，Y_MAX_POS，Z_MAX_POS 为打印机复位时的坐标值，在开启MAX软复位后，为打印机的最大打印范围。一般打印机在调试完后，通过G1指令移动打印头，M114查看当前坐标测得打印机最大打印范围。
//三角洲机型需特别注意 Z_MAX_POS 参数，为复位后打印头到平台之间的距离，可将此值设置大些，G28复位后，通过G1指令，移动Z轴，使打印头接触平台后，通过M114查看当前坐标，即可知道距离是多少了。
//
// Travel limits (mm) after homing, corresponding to endstop positions.    |复位后的行程限制 (mm)，对应于终点挡块位置。
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS X_BED_SIZE
#define Y_MAX_POS Y_BED_SIZE
#define Z_MAX_POS 250

/**
 * Software Endstops    |软件终点站
 *
 * - Prevent moves outside the set machine bounds.    |防止移动到设定的机器范围之外。
 * - Individual axes can be disabled, if desired.    |如果需要，可以禁用各个轴。
 * - X and Y only apply to Cartesian robots.    |X 和 Y 仅适用于笛卡尔机器人。
 * - Use 'M211' to set software endstops on/off or report current state    |使用“M211”设置软件终端止动装置开/关或报告当前状态
 */

// Min software endstops constrain movement within minimum coordinate bounds    |最小软件终点限位器将运动限制在最小坐标范围内
#define MIN_SOFTWARE_ENDSTOPS
#if ENABLED(MIN_SOFTWARE_ENDSTOPS)
  #define MIN_SOFTWARE_ENDSTOP_X
  #define MIN_SOFTWARE_ENDSTOP_Y
  //#define MIN_SOFTWARE_ENDSTOP_Z
#endif

// Max software endstops constrain movement within maximum coordinate bounds    |最小软件终点限位器将运动限制在最小坐标范围内
#define MAX_SOFTWARE_ENDSTOPS
#if ENABLED(MAX_SOFTWARE_ENDSTOPS)
  #define MAX_SOFTWARE_ENDSTOP_X
  #define MAX_SOFTWARE_ENDSTOP_Y
  #define MAX_SOFTWARE_ENDSTOP_Z
#endif

#if EITHER(MIN_SOFTWARE_ENDSTOPS, MAX_SOFTWARE_ENDSTOPS)
  //#define SOFT_ENDSTOPS_MENU_ITEM  // Enable/Disable software endstops from the LCD    |从 LCD 启用/禁用软件终点挡块
#endif

/**
 * 缺料检查
 * 如果有触摸屏。一般触摸屏带这个功能，无需理会
FILAMENT_RUNOUT_SENSOR 去掉注释，即可开启缺料检测功能。一般使用光电限位开关装在送丝机耗材入口处，高电平表示正常送丝，低电平表示缺料。
FIL_RUNOUT_INVERTING 设置为 true 可反转限位开关信号。
ENDSTOPPULLUP_FIL_RUNOUT 去掉注释，表示缺料检测引脚默认上拉，一般默认开启。
FILAMENT_RUNOUT_SCRIPT 设置缺料检测激活时运行的脚本，一般为 M600，使打印机暂时打印，换好耗材后，可继续打印。
 * 
 * Filament Runout Sensors    |线材跳动传感器
 * Mechanical or opto endstops are used to check for the presence of filament.    |机械或光电限位器用于检查线材是否存在。
 *
 * RAMPS-based boards use SERVO3_PIN for the first runout sensor.    |基于 RAMPS 的板使用 SERVO3_PIN 作为第一个跳动传感器。
 * For other boards you may need to define FIL_RUNOUT_PIN, FIL_RUNOUT2_PIN, etc.    |对于其他板，您可能需要定义 FIL_RUNOUT_PIN、FIL_RUNOUT2_PIN 等。
 * 
 * 
 * 针对Makeboard主板，可以打开 pins_RAMPS.h 文件，搜索 FIL_RUNOUT_PIN，看到如下代码：
 * // define digital pin 4 for the filament runout sensor. Use the RAMPS 1.4 digital input 4 on the servos connector
   #define FIL_RUNOUT_PIN 4
 * 将缺料检测引脚设置为任意空闲的限制开关，如 X_MAX_PIN，设置值为 2，将缺料检测光电限位开关插入 X-MAX 插座即可。
 */
//#define FILAMENT_RUNOUT_SENSOR
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT_ENABLED_DEFAULT true // Enable the sensor on startup. Override with M412 followed by M500.    |在启动时启用传感器。先用 M412 覆盖，然后用 M500 覆盖。
  #define NUM_RUNOUT_SENSORS   1          // Number of sensors, up to one per extruder. Define a FIL_RUNOUT#_PIN for each.    |传感器数量，每台挤出机最多一个。为每个定义一个 FIL_RUNOUT#_PIN。
  #define FIL_RUNOUT_STATE     LOW        // Pin state indicating that filament is NOT present.    |引脚状态指示线材不存在。
  #define FIL_RUNOUT_PULLUP               // Use internal pullup for filament runout pins.    |对线材跳动引脚使用内部上拉电阻。
  //#define FIL_RUNOUT_PULLDOWN           // Use internal pulldown for filament runout pins.    |对线材跳动引脚使用内部下拉。

  // Set one or more commands to execute on filament runout.    |设置一个或多个命令以在线材耗尽时执行
  // (After 'M412 H' Marlin will ask the host to handle the process.)    |（“M412 H”之后，马林会要求主持人处理该过程。）
  #define FILAMENT_RUNOUT_SCRIPT "M600"

  // After a runout is detected, continue printing this length of filament    |检测到跳动后，继续打印该长度的耗材
  // before executing the runout script. Useful for a sensor at the end of    |在执行运行脚本之前。对于 | 末尾的传感器很有用
  // a feed tube. Requires 4 bytes SRAM per sensor, plus 4 bytes overhead.    |进料管。每个传感器需要 4 字节 SRAM，外加 4 字节开销。 
  //#define FILAMENT_RUNOUT_DISTANCE_MM 25

  #ifdef FILAMENT_RUNOUT_DISTANCE_MM
    // Enable this option to use an encoder disc that toggles the runout pin    |启用此选项可使用可切换跳动销的编码器盘
    // as the filament moves. (Be sure to set FILAMENT_RUNOUT_DISTANCE_MM    |当线材移动时。 （请务必设置 FILAMENT_RUNOUT_DISTANCE_MM
    // large enough to avoid false positives.)    |足够大以避免误报。）
    //#define FILAMENT_MOTION_SENSOR
  #endif
#endif

//===========================================================================
//=============================== Bed Leveling ==============================    |热床调平
//===========================================================================
// @section calibrate



/**
 * Choose one of the options below to enable G29 Bed Leveling. The parameters    |选择以下选项之一以启用 G29 床调平。参数
 * and behavior of G29 will change depending on your selection.    |G29 的行为将根据您的选择而改变。
 *
 *  If using a Probe for Z Homing, enable Z_SAFE_HOMING also!    |如果使用探针进行 Z 复位，请同时启用 Z_SAFE_HOMING！
 *
 * - AUTO_BED_LEVELING_3POINT    |
 *   Probe 3 arbitrary points on the bed (that aren't collinear)    |探测床上的 3 个任意点（不共线）
 *   You specify the XY coordinates of all 3 points.    |您指定所有 3 个点的 XY 坐标。
 *   The result is a single tilted plane. Best for a flat bed.    |结果是单个倾斜平面。最适合平床。 
 *
 * - AUTO_BED_LEVELING_LINEAR    |
 *   Probe several points in a grid.    |探测网格中的几个点。 
 *   You specify the rectangle and the density of sample points.    |您指定矩形和样本点的密度。
 *   The result is a single tilted plane. Best for a flat bed.    |结果是单个倾斜平面。最适合平床。
 *
 * - AUTO_BED_LEVELING_BILINEAR    |
 *   Probe several points in a grid.    |探测网格中的几个点。 
 *   You specify the rectangle and the density of sample points.    |您指定矩形和样本点的密度。  
 *   The result is a mesh, best for large or uneven beds.    |结果是网状，最适合大或不平坦的床。
 *
 * - AUTO_BED_LEVELING_UBL (Unified Bed Leveling)    |（统一床调平）
 *   A comprehensive bed leveling system combining the features and benefits    |综合性的床调平系统，结合了功能和优点
 *   of other systems. UBL also includes integrated Mesh Generation, Mesh    |其他系统。 UBL 还包括集成的网格生成、Mesh
 *   Validation and Mesh Editing systems.    |验证和网格编辑系统。
 *
 * - MESH_BED_LEVELING    |
 *   Probe a grid manually    |手动探测网格
 *   The result is a mesh, suitable for large or uneven beds. (See BILINEAR.)    |结果是网状，适用于大或不平坦的床。 （参见双线性。）
 *   For machines without a probe, Mesh Bed Leveling provides a method to perform    |对于没有探头的机器，网床调平提供了一种执行 | 的方法。
 *   leveling in steps so you can manually adjust the Z height at each grid-point.    |逐步调平，以便您可以手动调整每个网格点的 Z 高度。
 *   With an LCD controller the process is guided step-by-step.    |使用 LCD 控制器可逐步引导该过程。 
 */
//#define AUTO_BED_LEVELING_3POINT
//#define AUTO_BED_LEVELING_LINEAR
#define AUTO_BED_LEVELING_BILINEAR
//#define AUTO_BED_LEVELING_UBL
//#define MESH_BED_LEVELING

/**
 * Normally G28 leaves leveling disabled on completion. Enable    |通常，G28 在完成后会禁用调平。启用
 * this option to have G28 restore the prior leveling state.    |此选项让G28恢复之前的调平状态
 */
#define RESTORE_LEVELING_AFTER_G28

/**
 * Enable detailed logging of G28, G29, M48, etc.    |启用G28、G29、M48等的详细记录
 * Turn on with the command 'M111 S32'.    |使用命令“M111 S32”打开。
 * NOTE: Requires a lot of PROGMEM!    |注意：需要大量的程序！
 */
//#define DEBUG_LEVELING_FEATURE

#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_UBL)
  // Gradually reduce leveling correction until a set height is reached,    |逐渐减少调平校正，直到达到设定高度，
  // at which point movement will be level to the machine's XY plane.    |此时运动将与机器的 XY 平面保​​持水平。 
  // The height can be set with M420 Z<height>    |高度可通过 M420 Z<height> 设置
  #define ENABLE_LEVELING_FADE_HEIGHT

  // For Cartesian machines, instead of dividing moves on mesh boundaries,    |对于笛卡尔机器，不是在网格边界上划分移动，
  // split up moves into short segments like a Delta. This follows the    |像三角洲一样分成短的部分。这遵循
  // contours of the bed more closely than edge-to-edge straight moves.    | 床的轮廓比边缘到边缘的直线移动更紧密。 
  #define SEGMENT_LEVELED_MOVES
  #define LEVELED_SEGMENT_LENGTH 5.0 // (mm) Length of all segments (except the last one)    |(mm) 所有段的长度（最后一段除外）

  /**
   * Enable the G26 Mesh Validation Pattern tool.    |启用 G26 网格验证图案工具。
   */
  //#define G26_MESH_VALIDATION
  #if ENABLED(G26_MESH_VALIDATION)
    #define MESH_TEST_NOZZLE_SIZE    0.4  // (mm) Diameter of primary nozzle.    |(mm) 主喷嘴直径。
    #define MESH_TEST_LAYER_HEIGHT   0.2  // (mm) Default layer height for the G26 Mesh Validation Tool.    |(mm) G26 网格验证工具的默认层高度。
    #define MESH_TEST_HOTEND_TEMP  205    // (°C) Default nozzle temperature for the G26 Mesh Validation Tool.    | (°C) G26 网格验证工具的默认喷嘴温度。
    #define MESH_TEST_BED_TEMP      60    // (°C) Default bed temperature for the G26 Mesh Validation Tool.    | (°C) G26 网格验证工具的默认床温度。
    #define G26_XY_FEEDRATE         20    // (mm/s) Feedrate for XY Moves for the G26 Mesh Validation Tool.    |(mm/s) G26 网格验证工具的 XY 移动进给率。
    #define G26_RETRACT_MULTIPLIER   1.0  // G26 Q (retraction) used by default between mesh test elements.    |默认情况下，网格测试元素之间使用 G26 Q（缩回）。  
  #endif

#endif

#if EITHER(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR)

  // Set the number of grid points per dimension.    |设置每个维度的网格点数。
  #define GRID_MAX_POINTS_X 3
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  // Probe along the Y axis, advancing X after each column    |沿 Y 轴探测，在每列后推进 X 
  //#define PROBE_Y_FIRST

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

    // Beyond the probed grid, continue the implied tilt?    |在探测到的网格之外，继续隐含的倾斜？ 
    // Default is to maintain the height of the nearest edge.    |默认值是保持最近边缘的高度。 
    //#define EXTRAPOLATE_BEYOND_GRID

    //
    // Experimental Subdivision of the grid by Catmull-Rom method.    | 通过 Catmull-Rom 方法对网格进行实验细分。
    // Synthesizes intermediate points to produce a more detailed mesh.    |合成中间点以生成更详细的网格。
    //
    //#define ABL_BILINEAR_SUBDIVISION
    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      // Number of subdivisions between probe points  |探测点之间的细分数
      #define BILINEAR_SUBDIVISIONS 3
    #endif

  #endif

#elif ENABLED(AUTO_BED_LEVELING_UBL)

  //===========================================================================
  //========================= Unified Bed Leveling ============================    |统一床层调平
  //===========================================================================

  //#define MESH_EDIT_GFX_OVERLAY   // Display a graphics overlay while editing the mesh    |在编辑网格时显示图形叠加

  #define MESH_INSET 1              // Set Mesh bounds as an inset region of the bed    | 将网格边界设置为床的插入区域
  #define GRID_MAX_POINTS_X 10      // Don't use more than 15 points per axis, implementation limited.    |每个轴不要使用超过 15 个点，实施受到限制。
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  #define UBL_MESH_EDIT_MOVES_Z     // Sophisticated users prefer no movement of nozzle    |经验丰富的用户更喜欢喷嘴不移动
  #define UBL_SAVE_ACTIVE_ON_M500   // Save the currently active mesh in the current slot on M500    |将当前活动的网格保存在 M500 上的当前槽中

  //#define UBL_Z_RAISE_WHEN_OFF_MESH 2.5 // When the nozzle is off the mesh, this value is used    |当喷嘴离开网格时，使用该值
                                          // as the Z-Height correction value.    |作为 Z 高度校正值。

#elif ENABLED(MESH_BED_LEVELING)

  //===========================================================================
  //=================================== Mesh ==================================    |网格
  //===========================================================================

  #define MESH_INSET 10          // Set Mesh bounds as an inset region of the bed    |将网格边界设置为床的插入区域
  #define GRID_MAX_POINTS_X 3    // Don't use more than 7 points per axis, implementation limited.    | 每个轴不要使用超过 7 个点，实施受到限制。
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  //#define MESH_G28_REST_ORIGIN // After homing all axes ('G28' or 'G28 XYZ') rest Z at Z_MIN_POS    |复位后所有轴（'G28' 或 'G28 XYZ'）在 Z_MIN_POS 处静止 Z

#endif // BED_LEVELING

/**
 * Add a bed leveling sub-menu for ABL or MBL.    |为 ABL 或 MBL 添加床平整子菜单。
 * Include a guided procedure if manual probing is enabled.    |如果启用了手动探测，请包括指导程序。
 */
#define LCD_BED_LEVELING

#if ENABLED(LCD_BED_LEVELING)
  #define MESH_EDIT_Z_STEP  0.025 // (mm) Step size while manually probing Z axis.    | (mm) 手动探测Z轴时的步长。 
  #define LCD_PROBE_Z_RANGE 4     // (mm) Z Range centered on Z_MIN_POS for LCD Z adjustment    | (mm) 以 Z_MIN_POS 为中心的 Z 范围，用于 LCD Z 调整 
  //#define MESH_EDIT_MENU        // Add a menu to edit mesh points    |添加菜单来编辑网格点
#endif

// Add a menu item to move between bed corners for manual bed adjustment    |添加一个菜单项以在床角之间移动以进行手动床调整
#define LEVEL_BED_CORNERS

#if ENABLED(LEVEL_BED_CORNERS)
  #define LEVEL_CORNERS_INSET_LFRB { 40, 40, 40, 40 } // (mm) Left, Front, Right, Back insets    |(mm) 左、前、右、后插图
  #define LEVEL_CORNERS_HEIGHT      0.0   // (mm) Z height of nozzle at leveling points    | (mm) 调平点喷嘴 Z 高度
  #define LEVEL_CORNERS_Z_HOP       4.0   // (mm) Z height of nozzle between leveling points    |(mm) 调平点之间喷嘴的 Z 高度
  #define LEVEL_CENTER_TOO              // Move to the center after the last corner    |最后一个角后移动到中心
#endif

/**
 * Commands to execute at the end of G29 probing.    |G29 探测结束时执行的命令。    
 * Useful to retract or move the Z probe out of the way.    |用于缩回或移开 Z 探针。 
 */
//#define Z_PROBE_END_SCRIPT "G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10"    |

// @section homing

// The center of the bed is at (X=0, Y=0)    |床的中心位于 (X=0, Y=0)
//#define BED_CENTER_AT_0_0

// Manually set the home position. Leave these undefined for automatic settings.    |手动设置起始位置。对于自动设置，请保留这些未定义。
// For DELTA this is the top-center of the Cartesian print volume.    |对于 DELTA，这是笛卡尔打印体积的顶部中心。
//#define MANUAL_X_HOME_POS 0
//#define MANUAL_Y_HOME_POS 0
//#define MANUAL_Z_HOME_POS 0

// Use "Z Safe Homing" to avoid homing with a Z probe outside the bed area.    |使用“Z 安全复位”可避免在床区域外使用 Z 探头复位。
//
// With this feature enabled:    |启用此功能后：
//
// - Allow Z homing only after X and Y homing AND stepper drivers still enabled.    |仅在 X 和 Y 复位且步进驱动器仍启用后才允许 Z 复位。
// - If stepper drivers time out, it will need X and Y homing again before Z homing.    |如果步进驱动器超时，则在 Z 回原点之前需要再次 X 和 Y 回原点。
// - Move the Z probe (or nozzle) to a defined XY point before Z Homing.    |在 Z 复位之前将 Z 探头（或喷嘴）移动到定义的 XY 点。
// - Prevent Z homing when the Z probe is outside bed area.    |当 Z 探头位于床区域之外时，防止 Z 复位。
//
#define Z_SAFE_HOMING

#if ENABLED(Z_SAFE_HOMING)
  #define Z_SAFE_HOMING_X_POINT X_CENTER  // X point for Z homing    |Z 复位的 X 点
  #define Z_SAFE_HOMING_Y_POINT Y_CENTER  // Y point for Z homing    |Z 复位的 Y 点
#endif

//复位速度
//HOMING_FEEDRATE_XY 设置复位时XY轴的移动速度
//HOMING_FEEDRATE_Z 设置复位时Z轴的移动速度
// Homing speeds (mm/min)
#define HOMING_FEEDRATE_XY (80*60)
#define HOMING_FEEDRATE_Z  (20*60)

// Validate that endstops are triggered on homing moves    |验证复位移动时是否会触发终点挡块
#define VALIDATE_HOMING_ENDSTOPS

// @section calibrate

/**
 * Bed Skew Compensation    |床身倾斜补偿 
 *
 * This feature corrects for misalignment in the XYZ axes.    |此功能可纠正 XYZ 轴的错位。
 *
 * Take the following steps to get the bed skew in the XY plane:    |按照以下步骤获取 XY 平面上的床身倾斜：
 *  1. Print a test square (e.g., https://www.thingiverse.com/thing:2563185)    |打印一个测试方块
 *  2. For XY_DIAG_AC measure the diagonal A to C    |对于 XY_DIAG_AC，测量 A 到 C 的对角线
 *  3. For XY_DIAG_BD measure the diagonal B to D    |对于 XY_DIAG_BD 测量对角线 B 到 D
 *  4. For XY_SIDE_AD measure the edge A to D    |对于 XY_SIDE_AD，测量边缘 A 到 D
 *
 * Marlin automatically computes skew factors from these measurements.    |Marlin 根据这些测量结果自动计算偏斜系数。
 * Skew factors may also be computed and set manually:    |倾斜因子也可以手动计算和设置：
 *
 *  - Compute AB     : SQRT(2*AC*AC+2*BD*BD-4*AD*AD)/2    |计算AB：SQRT(2*AC*AC+2*BD*BD-4*AD*AD)/2
 *  - XY_SKEW_FACTOR : TAN(PI/2-ACOS((AC*AC-AB*AB-AD*AD)/(2*AB*AD))) 
 *
 * If desired, follow the same procedure for XZ and YZ.    |如果需要，请对 XZ 和 YZ 执行相同的步骤。
 * Use these diagrams for reference:    |使用这些图表作为参考
 *
 *    Y                     Z                     Z
 *    ^     B-------C       ^     B-------C       ^     B-------C
 *    |    /       /        |    /       /        |    /       /
 *    |   /       /         |   /       /         |   /       /
 *    |  A-------D          |  A-------D          |  A-------D
 *    +-------------->X     +-------------->X     +-------------->Y
 *     XY_SKEW_FACTOR        XZ_SKEW_FACTOR        YZ_SKEW_FACTOR
 */
//#define SKEW_CORRECTION

#if ENABLED(SKEW_CORRECTION)
  // Input all length measurements here:    |在此输入所有长度测量值：
  #define XY_DIAG_AC 282.8427124746
  #define XY_DIAG_BD 282.8427124746
  #define XY_SIDE_AD 200

  // Or, set the default skew factors directly here    |或者，直接在此处设置默认倾斜系数 
  // to override the above measurements:    |覆盖上述测量：
  #define XY_SKEW_FACTOR 0.0

  //#define SKEW_CORRECTION_FOR_Z
  #if ENABLED(SKEW_CORRECTION_FOR_Z)
    #define XZ_DIAG_AC 282.8427124746
    #define XZ_DIAG_BD 282.8427124746
    #define YZ_DIAG_AC 282.8427124746
    #define YZ_DIAG_BD 282.8427124746
    #define YZ_SIDE_AD 200
    #define XZ_SKEW_FACTOR 0.0
    #define YZ_SKEW_FACTOR 0.0
  #endif

  // Enable this option for M852 to set skew at runtime    |为 M852 启用此选项以在运行时设置偏斜
  //#define SKEW_CORRECTION_GCODE
#endif

//=============================================================================
//============================= Additional Features ===========================    |附加功能
//=============================================================================

// @section extras

/**
 * EEPROM
 *EEPROM_SETTINGS 去掉注释，可开启EEPROM功能，打印机部分配置参数将保存在打印机中，可通过液晶屏实时调节，无需重刷固件。
 配置固件阶段推荐关闭，调试好机器后再开启，要不很可能有参数给代码中调节后不会生效，造成误判断。如果清除eeprom,参数默认值是固件中修改的数值。
 * Persistent storage to preserve configurable settings across reboots.
 *
 *   M500 - Store settings to EEPROM.    |将设置存储到 EEPROM
 *   M501 - Read settings from EEPROM. (i.e., Throw away unsaved changes)    |从 EEPROM 读取设置。 （即，丢弃未保存的更改）
 *   M502 - Revert settings to "factory" defaults. (Follow with M500 to init the EEPROM.)    |将设置恢复为“出厂”默认值。 （接着用M500来初始化EEPROM。）
 */
#define EEPROM_SETTINGS       // Persistent storage with M500 and M501    |使用 M500 和 M501 进行持久存储
//#define DISABLE_M503        // Saves ~2700 bytes of PROGMEM. Disable for release!    |节省约 2700 字节的 PROGMEM。禁用以释放！
#define EEPROM_CHITCHAT       // Give feedback on EEPROM commands. Disable to save PROGMEM.    |提供有关 EEPROM 命令的反馈。禁止保存PROGMEM
#define EEPROM_BOOT_SILENT    // Keep M503 quiet and only give errors during first load    |保持 M503 安静，仅在首次加载期间给出错误
#if ENABLED(EEPROM_SETTINGS)
  #define EEPROM_AUTO_INIT  // Init EEPROM automatically on any errors.    |出现任何错误时自动初始化 EEPROM。
#endif

//
// Host Keepalive    |主机保活
//
// When enabled Marlin will send a busy status message to the host    |启用后，Marlin 将向主机发送繁忙状态消息
// every couple of seconds when it can't accept commands.    |当它无法接受命令时每隔几秒钟。   
//
#define HOST_KEEPALIVE_FEATURE        // Disable this if your host doesn't like keepalive messages    |如果您的主机不喜欢 keepalive 消息，请禁用此功能
#define DEFAULT_KEEPALIVE_INTERVAL 2  // Number of seconds between "busy" messages. Set with M113.    |“忙”消息之间的秒数。用M113设置。
#define BUSY_WHILE_HEATING            // Some hosts require "busy" messages even during heating    |有些主机即使在加热过程中也需要“忙”消息 

//
// G20/G21 Inch mode support    |G20/G21 英制模式支持
//
//#define INCH_MODE_SUPPORT    |

//
// M149 Set temperature units support    |M149 设置温度单位支持
//
//#define TEMPERATURE_UNITS_SUPPORT    |

// @section temperature

//预加热
//默认即可，开启eeprom可在菜单中调整
//分别设置液晶屏菜单选择预加热PLA和ABS的打印头，热床温度，及冷却风扇转速。
// Preheat Constants     |预热常数
#define PREHEAT_1_LABEL       "PLA"
#define PREHEAT_1_TEMP_HOTEND 200
#define PREHEAT_1_TEMP_BED     60
#define PREHEAT_1_FAN_SPEED     0 // Value from 0 to 255

#define PREHEAT_2_LABEL       "PETG"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED     80
#define PREHEAT_2_FAN_SPEED     0 // Value from 0 to 255

/**
 * Nozzle Park    |喷嘴公园
 *
 * Park the nozzle at the given XYZ position on idle or G27.    |将喷嘴停放在空闲或 G27 上的给定 XYZ 位置。
 *
 * The "P" parameter controls the action applied to the Z axis:    |“P”参数控制应用于 Z 轴的动作：
 *
 *    P0  (Default) If Z is below park Z raise the nozzle.    |P0（默认）如果 Z 低于停放 Z，则升起喷嘴。 
 *    P1  Raise the nozzle always to Z-park height.    |P1 始终将喷嘴提升至 Z 停放高度。 
 *    P2  Raise the nozzle by Z-park amount, limited to Z_MAX_POS.    |P2 将喷嘴提升Z-park 量，限制为Z_MAX_POS。
 */
#define NOZZLE_PARK_FEATURE

#if ENABLED(NOZZLE_PARK_FEATURE)
  // Specify a park position as { X, Y, Z_raise }    |将停放位置指定为 { X, Y, Z_raise } 
  #define NOZZLE_PARK_POINT { (X_MIN_POS + 10), (Y_MAX_POS - 10), 20 }
  //#define NOZZLE_PARK_X_ONLY          // X move only is required to park    |仅需要 X 移动即可停车
  //#define NOZZLE_PARK_Y_ONLY          // Y move only is required to park    |仅需要 Y 移动即可停车
  #define NOZZLE_PARK_Z_RAISE_MIN   2   // (mm) Always raise Z by at least this distance    |(mm) 始终将 Z 升高至少此距离
  #define NOZZLE_PARK_XY_FEEDRATE 100   // (mm/s) X and Y axes feedrate (also used for delta Z axis)    |(mm/s) X 轴和 Y 轴进给率（也用于 Delta Z 轴） 
  #define NOZZLE_PARK_Z_FEEDRATE    5   // (mm/s) Z axis feedrate (not used for delta printers)    |(mm/s) Z 轴进给率（不用于 Delta 打印机）
#endif

/**
 * Clean Nozzle Feature -- EXPERIMENTAL    |清洁喷嘴功能——实验
 *
 * Adds the G12 command to perform a nozzle cleaning process.    |添加G12指令来执行喷嘴清洁过程。
 *
 * Parameters:
 *   P  Pattern    |P 图案
 *   S  Strokes / Repetitions    |S 笔画/重复
 *   T  Triangles (P1 only)    |T 形三角形（仅限 P1）
 *
 * Patterns:  |图案
 *   P0  Straight line (default). This process requires a sponge type material    |P0 直线（默认）。这个过程需要海绵类材料 
 *       at a fixed bed location. "S" specifies strokes (i.e. back-forth motions)    |在固定床位。 “S”指定笔画（即前后运动）
 *       between the start / end points.    |起点/终点之间。
 *
 *   P1  Zig-zag pattern between (X0, Y0) and (X1, Y1), "T" specifies the    |P1 (X0, Y0) 和 (X1, Y1) 之间的锯齿形图案，“T”指定
 *       number of zig-zag triangles to do. "S" defines the number of strokes.    |要做的之字形三角形的数量。 “S”定义了笔划数。
 *       Zig-zags are done in whichever is the narrower dimension.    |之字形以较窄的尺寸进行。
 *       For example, "G12 P1 S1 T3" will execute:    |例如，“G12 P1 S1 T3”将执行：
 *
 *          --
 *         |  (X0, Y1) |     /\        /\        /\     | (X1, Y1)
 *         |           |    /  \      /  \      /  \    |
 *       A |           |   /    \    /    \    /    \   |
 *         |           |  /      \  /      \  /      \  |
 *         |  (X0, Y0) | /        \/        \/        \ | (X1, Y0)
 *          --         +--------------------------------+
 *                       |________|_________|_________|
 *                           T1        T2        T3
 *
 *   P2  Circular pattern with middle at NOZZLE_CLEAN_CIRCLE_MIDDLE.    |P2 圆形图案，中间位于 NOZZLE_CLEAN_CIRCLE_MIDDLE。
 *       "R" specifies the radius. "S" specifies the stroke count.    |“R”指定半径。 “S”指定笔划数。
 *       Before starting, the nozzle moves to NOZZLE_CLEAN_START_POINT.    |启动前，喷嘴移动至 NOZZLE_CLEAN_START_POINT。
 *
 *   Caveats: The ending Z should be the same as starting Z.    |注意事项：结尾 Z 应该与起始 Z 相同。
 * Attention: EXPERIMENTAL. G-code arguments may change.    |注意：实验性的。 G 代码参数可能会改变。
 */
//#define NOZZLE_CLEAN_FEATURE

#if ENABLED(NOZZLE_CLEAN_FEATURE)
  // Default number of pattern repetitions    |默认模式重复次数
  #define NOZZLE_CLEAN_STROKES  12

  // Default number of triangles    |默认三角形数量
  #define NOZZLE_CLEAN_TRIANGLES  3

  // Specify positions for each tool as { { X, Y, Z }, { X, Y, Z } }    |将每个工具的位置指定为 { { X, Y, Z }, { X, Y, Z } }
  // Dual hotend system may use { {  -20, (Y_BED_SIZE / 2), (Z_MIN_POS + 1) },  {  420, (Y_BED_SIZE / 2), (Z_MIN_POS + 1) }}    | 双热端系统可以使用 { { -20, (Y_BED_SIZE /2), (Z_MIN_POS + 1) }, { 420, (Y_BED_SIZE /2), (Z_MIN_POS + 1) }} 
  #define NOZZLE_CLEAN_START_POINT { {  30, 30, (Z_MIN_POS + 1) } }
  #define NOZZLE_CLEAN_END_POINT   { { 100, 60, (Z_MIN_POS + 1) } }

  // Circular pattern radius    | 圆形图案半径
  #define NOZZLE_CLEAN_CIRCLE_RADIUS 6.5
  // Circular pattern circle fragments number    |圆形图案圆形碎片数
  #define NOZZLE_CLEAN_CIRCLE_FN 10
  // Middle point of circle    |圆的中点
  #define NOZZLE_CLEAN_CIRCLE_MIDDLE NOZZLE_CLEAN_START_POINT

  // Move the nozzle to the initial position after cleaning    |清洗后将喷嘴移至初始位置
  #define NOZZLE_CLEAN_GOBACK

  // For a purge/clean station that's always at the gantry height (thus no Z move)    |对于始终处于龙门高度的吹扫/清洁站（因此没有 Z 轴移动）
  //#define NOZZLE_CLEAN_NO_Z

  // For a purge/clean station mounted on the X axis    |对于安装在 X 轴上的吹扫/清洁站
  //#define NOZZLE_CLEAN_NO_Y

  // Explicit wipe G-code script applies to a G12 with no arguments.    |显式擦除 G 代码脚本适用于不带参数的 G12。
  //#define WIPE_SEQUENCE_COMMANDS "G1 X-17 Y25 Z10 F4000\nG1 Z1\nM114\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 Z15\nM400\nG0 X-10.0 Y-9.0"    |

#endif

/**
 * Print Job Timer    |打印作业定时器
 *
 * Automatically start and stop the print job timer on M104/M109/M190.    |自动启动和停止M104/M109/M190 上的打印作业定时器。
 *
 *   M104 (hotend, no wait) - high temp = none,        low temp = stop timer    |M104（热端，无等待）-高温=无，低温=停止定时器
 *   M109 (hotend, wait)    - high temp = start timer, low temp = stop timer    |M109（热端，等待）-高温=启动定时器，低温=停止定时器
 *   M190 (bed, wait)       - high temp = start timer, low temp = none    |M190（睡觉，等待）-高温=启动计时器，低温=无
 *
 * The timer can also be controlled with the following commands:    |定时器也可以通过以下命令控制：
 *
 *   M75 - Start the print job timer    |M75 -启动打印作业计时器
 *   M76 - Pause the print job timer    |M76 -暂停打印作业计时器
 *   M77 - Stop the print job timer    |M77 -停止打印作业计时器
 */
#define PRINTJOB_TIMER_AUTOSTART

/**
 * Print Counter    |打印柜台
 *
 * Track statistical data such as:    |跟踪统计数据，例如：
 *
 *  - Total print jobs    |打印作业总数
 *  - Total successful print jobs    |成功打印作业总数
 *  - Total failed print jobs    |失败的打印作业总数 
 *  - Total time printing    |打印总时间
 *
 * View the current statistics with M78.    |查看M78的当前统计数据。
 */
//#define PRINTCOUNTER

/**
 * Password    |密码 
 *
 * Set a numerical password for the printer which can be requested:    |为打印机设置可请求的数字密码：
 *
 *  - When the printer boots up    |打印机启动时
 *  - Upon opening the 'Print from Media' Menu    |打开“从媒体打印”菜单时
 *  - When SD printing is completed or aborted    |当 SD 打印完成或中止时
 *
 * The following G-codes can be used:    |可以使用以下 G 代码：
 *
 *  M510 - Lock Printer. Blocks all commands except M511.    |M510 -锁定打印机。阻止除 M511 之外的所有命令。
 *  M511 - Unlock Printer.    |M511 -解锁打印机。 
 *  M512 - Set, Change and Remove Password.    |M512 -设置、更改和删除密码。
 *
 * If you forget the password and get locked out you'll need to re-flash    |如果您忘记密码并被锁定，则需要重新刷新
 * the firmware with the feature disabled, reset EEPROM, and (optionally)    |禁用该功能的固件，重置 EEPROM，以及（可选）
 * re-flash the firmware again with this feature enabled.    |启用此功能后再次重新刷新固件。 
 */
//#define PASSWORD_FEATURE
#if ENABLED(PASSWORD_FEATURE)
  #define PASSWORD_LENGTH 4                 // (#) Number of digits (1-9). 3 or 4 is recommended    |(#) 位数 (1-9)。推荐3或4
  #define PASSWORD_ON_STARTUP
  #define PASSWORD_UNLOCK_GCODE             // Unlock with the M511 P<password> command. Disable to prevent brute-force attack.    |使用 M511 P<密码> 命令解锁。禁用以防止暴力攻击。
  #define PASSWORD_CHANGE_GCODE             // Change the password with M512 P<old> S<new>.    |使用 M512 P<旧> S<新> 更改密码。
  //#define PASSWORD_ON_SD_PRINT_MENU       // This does not prevent gcodes from running    |这不会阻止 gcode 运行
  //#define PASSWORD_AFTER_SD_PRINT_END
  //#define PASSWORD_AFTER_SD_PRINT_ABORT
  //#include "Configuration_Secure.h"       // External file with PASSWORD_DEFAULT_VALUE    |带有 PASSWORD_DEFAULT_VALUE 的外部文件
#endif

//=============================================================================
//============================= LCD and SD support ============================    |
//=============================================================================

// @section lcd

/**
 * LCD LANGUAGE
 * 液晶屏语言
 * Marlin固件现在的最新版本直接官方支持中文，LCD_LANGUAGE 设置为 cn 即可，前提使需要使用12864液晶屏。注意，新版的不是cn而是zh_Cn大小写请参考注释
 *
 * Select the language to display on the LCD. These languages are available:    |选择 LCD 上显示的语言。这些语言可用
 *
 *   en, an, bg, ca, cz, da, de, el, el_gr, es, eu, fi, fr, gl, hr, hu, it,
 *   jp_kana, ko_KR, nl, pl, pt, pt_br, ro, ru, sk, tr, uk, vi, zh_CN, zh_TW, test
 *
 * :{ 'en':'English', 'an':'Aragonese', 'bg':'Bulgarian', 'ca':'Catalan', 'cz':'Czech', 'da':'Danish', 'de':'German', 'el':'Greek', 'el_gr':'Greek (Greece)', 'es':'Spanish', 'eu':'Basque-Euskera', 'fi':'Finnish', 'fr':'French', 'gl':'Galician', 'hr':'Croatian', 'hu':'Hungarian', 'it':'Italian', 'jp_kana':'Japanese', 'ko_KR':'Korean (South Korea)', 'nl':'Dutch', 'pl':'Polish', 'pt':'Portuguese', 'pt_br':'Portuguese (Brazilian)', 'ro':'Romanian', 'ru':'Russian', 'sk':'Slovak', 'tr':'Turkish', 'uk':'Ukrainian', 'vi':'Vietnamese', 'zh_CN':'Chinese (Simplified)', 'zh_TW':'Chinese (Traditional)', 'test':'TEST' }
 */
#define LCD_LANGUAGE zh_CN

/**
 * LCD Character Set    |LCD字符集
 *
 * Note: This option is NOT applicable to Graphical Displays.    |注意：此选项不适用于图形显示。
 *
 * All character-based LCDs provide ASCII plus one of these    |所有基于字符的 LCD 均提供 ASCII 以及其中之一
 * language extensions:    |语言扩展:
 *
 *  - JAPANESE ... the most common    |日语...最常见
 *  - WESTERN  ... with more accented characters    |西方...带有更多重音字符
 *  - CYRILLIC ... for the Russian language    |西里尔字母 ... 俄语
 *
 * To determine the language extension installed on your controller:    |要确定控制器上安装的语言扩展：
 *
 *  - Compile and upload with LCD_LANGUAGE set to 'test'    |将 LCD_LANGUAGE 设置为“测试”进行编译和上传
 *  - Click the controller to view the LCD menu    |单击控制器可查看 LCD 菜单
 *  - The LCD will display Japanese, Western, or Cyrillic text    |LCD 将显示日语、西方或西里尔文字
 *
 * See https://marlinfw.org/docs/development/lcd_language.html
 *
 * :['JAPANESE', 'WESTERN', 'CYRILLIC']
 */
#define DISPLAY_CHARSET_HD44780 JAPANESE

/**
 * Info Screen Style (0:Classic, 1:Průša)    |信息屏幕样式（0：经典，1：Průša）
 *
 * :[0:'Classic', 1:'Průša']
 */
#define LCD_INFO_SCREEN_STYLE 0

/**
 * SD CARD
 *
 * SD Card support is disabled by default. If your controller has an SD slot,    |默认情况下禁用 SD 卡支持。如果您的控制器有 SD 插槽，
 * you must uncomment the following option or it won't work.    |您必须取消注释以下选项，否则它将不起作用。 
 */
#define SDSUPPORT

/**
 * SD CARD: SPI SPEED
 *
 * Enable one of the following items for a slower SPI transfer speed.    |启用以下项目之一可降低 SPI 传输速度。
 * This may be required to resolve "volume init" errors.    |这可能需要解决“卷初始化”错误。
 */
//#define SPI_SPEED SPI_HALF_SPEED
//#define SPI_SPEED SPI_QUARTER_SPEED
//#define SPI_SPEED SPI_EIGHTH_SPEED

/**
 * SD CARD: ENABLE CRC
 *
 * Use CRC checks and retries on the SD communication.    |对 SD 通信使用 CRC 检查和重试。
 */
#define SD_CHECK_AND_RETRY

/**
 * LCD Menu Items   |LCD 菜单项
 *
 * Disable all menus and only display the Status Screen, or    |禁用所有菜单并仅显示状态屏幕，或 
 * just remove some extraneous menu items to recover space.    |只需删除一些无关的菜单项即可恢复空间。
 */
//#define NO_LCD_MENUS
//#define SLIM_LCD_MENUS

//
// ENCODER SETTINGS    |编码器设置 |
//
// This option overrides the default number of encoder pulses needed to    | 此选项会覆盖默认的编码器脉冲数。
// produce one step. Should be increased for high-resolution encoders.    |产生一步。对于高分辨率编码器应增加。
//
//#define ENCODER_PULSES_PER_STEP 4

//
// Use this option to override the number of step signals required to    |使用此选项可以覆盖 | 所需的步进信号数量。
// move between next/prev menu items.    |在下一个/上一个菜单项之间移动。 
//
//#define ENCODER_STEPS_PER_MENU_ITEM 1

/**
 * Encoder Direction Options    |
 * 旋转编码器方向
 * REVERSE_ENCODER_DIRECTION 去掉注释，反转液晶屏上调节数值旋转编码器方向。
REVERSE_MENU_DIRECTION 去掉注释，反转液晶屏上选择菜单时旋转编码器方向。
有些液晶屏旋转编码器方向做反了，需要软件修正，
 *
 * Test your encoder's behavior first with both options disabled.    |首先在禁用这两个选项的情况下测试编码器的行为。
 *
 *  Reversed Value Edit and Menu Nav? Enable REVERSE_ENCODER_DIRECTION.    |反转值编辑和菜单导航？启用 REVERSE_ENCODER_DIRECTION。
 *  Reversed Menu Navigation only?    Enable REVERSE_MENU_DIRECTION.    |仅反向菜单导航？    启用 REVERSE_MENU_DIRECTION。
 *  Reversed Value Editing only?      Enable BOTH options.    |仅限反转值编辑？      启用这两个选项。
 */

//
// This option reverses the encoder direction everywhere.    |此选项会在各处反转编码器方向。 
//
//  Set this option if CLOCKWISE causes values to DECREASE    |如果“顺时针”导致值减小，请设置此选项
//
//#define REVERSE_ENCODER_DIRECTION

//
// This option reverses the encoder direction for navigating LCD menus.    |此选项反转编码器方向以导航 LCD 菜单。
//
//  If CLOCKWISE normally moves DOWN this makes it go UP.    | 如果“顺时针”通常向下移动，则它会向上移动。
//  If CLOCKWISE normally moves UP this makes it go DOWN.    |如果顺时针通常向上移动，则它会向下移动。  
//
//#define REVERSE_MENU_DIRECTION

//
// This option reverses the encoder direction for Select Screen.    |此选项反转选择屏幕的编码器方向。 
//
//  If CLOCKWISE normally moves LEFT this makes it go RIGHT.    |如果顺时针通常向左移动，则它会向右移动。
//  If CLOCKWISE normally moves RIGHT this makes it go LEFT.    |如果顺时针通常向右移动，则它会向左移动。  
//
//#define REVERSE_SELECT_DIRECTION

//
// Individual Axis Homing    |
//独立轴复位菜单
//INDIVIDUAL_AXIS_HOMING_MENU 去掉注释，可在液晶屏上增加单独的复位X，Y，Z轴的菜单，方便调试。
// Add individual axis homing items (Home X, Home Y, and Home Z) to the LCD menu.
//
//#define INDIVIDUAL_AXIS_HOMING_MENU

//
// SPEAKER/BUZZER
//液晶屏蜂鸣器
//SPEAKER 去掉注释，可开始液晶屏上的蜂鸣器，旋转编码旋转或者按下时蜂鸣器会发声。
// If you have a speaker that can produce tones, enable it here.    |如果您有可以发出声音的扬声器，请在此处启用它。
// By default Marlin assumes you have a buzzer with a fixed frequency.    |默认情况下，Marlin 假定您有一个固定频率的蜂鸣器。
//
//#define SPEAKER

//
// The duration and frequency for the UI feedback sound.    | UI 反馈声音的持续时间和频率。
//  
// Set these to 0 to disable audio feedback in the LCD menus.    |将这些设置为 0 可禁用 LCD 菜单中的音频反馈。
//
// Note: Test audio output with the G-Code:    |注意：使用 G 代码测试音频输出：
//  M300 S<frequency Hz> P<duration ms>    |M300 S<频率 Hz> P<持续时间 ms> 
//
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
//#define LCD_FEEDBACK_FREQUENCY_HZ 5000

//=============================================================================
//======================== LCD / Controller Selection =========================    |LCD/控制器选择
//========================   (Character-based LCDs)   =========================    |基于字符的 LCD）
//=============================================================================

//
//液晶屏2004
//去掉 REPRAP_DISCOUNT_SMART_CONTROLLER 行的注释，可开启2004液晶屏功能。
//特别强调，如果使用非配套Makeboard液晶屏，液晶屏无法使用的话，可能时液晶屏牛角插座缺口方向是反的，EXP1和EXP2线的接头凸起处换各方向强行插入即可。
// RepRapDiscount Smart Controller.
// https://reprap.org/wiki/RepRapDiscount_Smart_Controller
//
// Note: Usually sold with a white PCB.
//
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

//
// Original RADDS LCD Display+Encoder+SDCardReader    |原装RADDS液晶显示屏+编码器+SD卡读卡器
// http://doku.radds.org/dokumentation/lcd-display/
//
//#define RADDS_DISPLAY

//
// ULTIMAKER Controller.    |
//
//#define ULTIMAKERCONTROLLER

//
// ULTIPANEL as seen on Thingiverse.    |
//
//#define ULTIPANEL

//
// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)    |
// https://reprap.org/wiki/PanelOne
//
//#define PANEL_ONE

//
// GADGETS3D G3D LCD/SD Controller    |
// https://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//
// Note: Usually sold with a blue PCB.    |注意：通常与蓝色 PCB 一起出售。
//
//#define G3D_PANEL

//
// RigidBot Panel V1.0    |
// http://www.inventapart.com/
//
//#define RIGIDBOT_PANEL

//
//Makeboard 3D Printer Parts 3D Printer Mini Display 1602 Mini Controller    |Makeboard 3D 打印机零件 3D 打印机迷你显示器 1602 迷你控制器 
// https://www.aliexpress.com/item/32765887917.html
//
//#define MAKEBOARD_MINI_2_LINE_DISPLAY_1602

//
// ANET and Tronxy 20x4 Controller    |ANET 和 Tronxy 20x4 控制器
//
//#define ZONESTAR_LCD            // Requires ADC_KEYPAD_PIN to be assigned to an analog pin.    |需要将 ADC_KEYPAD_PIN 分配给模拟引脚。
                                  // This LCD is known to be susceptible to electrical interference    |众所周知，这种 LCD 容易受到电气干扰
                                  // which scrambles the display.  Pressing any button clears it up.    |这会扰乱显示。  按任意按钮即可将其清除。
                                  // This is a LCD2004 display with 5 analog buttons.    |这是一个 LCD2004 显示屏，带有 5 个模拟按钮。

//
// Generic 16x2, 16x4, 20x2, or 20x4 character-based LCD.    |通用 16x2、16x4、20x2 或 20x4 基于字符的 LCD。
//
//#define ULTRA_LCD

//=============================================================================
//======================== LCD / Controller Selection =========================    |LCD/控制器选择
//=====================   (I2C and Shift-Register LCDs)   =====================    |I2C 和移位寄存器 LCD）
//=============================================================================

//
// CONTROLLER TYPE: I2C    |控制器类型：I2C |
//
// Note: These controllers require the installation of Arduino's LiquidCrystal_I2C    |注意：这些控制器需要安装Arduino的LiquidCrystal_I2C 
// library. For more info: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//

//
// Elefu RA Board Control Panel    |Elefu RA板控制面板
// http://www.elefu.com/index.php?route=product/product&product_id=53
//
//#define RA_CONTROL_PANEL

//
// Sainsmart (YwRobot) LCD Displays    |Sainsmart（YwRobot）液晶显示器
//
// These require F.Malpartida's LiquidCrystal_I2C library    |这些需要 F.Malpartida 的 LiquidCrystal_I2C 库
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
//
//#define LCD_SAINSMART_I2C_1602
//#define LCD_SAINSMART_I2C_2004

//
// Generic LCM1602 LCD adapter    |通用 LCM1602 LCD 适配器
//
//#define LCM1602

//
// PANELOLU2 LCD with status LEDs,    | PANELOLU2 LCD 带状态 LED，
// separate encoder and click inputs.    |单独的编码器和点击输入。
//
// Note: This controller requires Arduino's LiquidTWI2 library v1.2.3 or later.    |注意：该控制器需要 Arduino 的 LiquidTWI2 库 v1.2.3 或更高版本。
// For more info: https://github.com/lincomatic/LiquidTWI2
//
// Note: The PANELOLU2 encoder click input can either be directly connected to    |注意：PANELOLU2编码器点击输入可以直接连接到
// a pin (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).    |引脚（如果 BTN_ENC 定义为！= -1）或通过 I2C 读取（当 BTN_ENC == -1 时）。
//
//#define LCD_I2C_PANELOLU2

//
// Panucatt VIKI LCD with status LEDs,    |带状态 LED 的 Panucatt VIKI LCD，
// integrated click & L/R/U/D buttons, separate encoder inputs.    |集成点击和 L/R/U/D 按钮，独立的编码器输入。
//
//#define LCD_I2C_VIKI

//
// CONTROLLER TYPE: Shift register panels    |控制器类型：移位寄存器面板
//

//
// 2-wire Non-latching LCD SR from https://goo.gl/aJJ4sH    |
// LCD configuration: https://reprap.org/wiki/SAV_3D_LCD    |
//
//#define SAV_3DLCD

//
// 3-wire SR LCD with strobe using 74HC4094    |使用 74HC4094 带频闪的 3 线 SR LCD 
// https://github.com/mikeshub/SailfishLCD
// Uses the code directly from Sailfish    |直接使用来自 Sailfish 的代码
//
//#define FF_INTERFACEBOARD

//
// TFT GLCD Panel with Marlin UI    |带有 Marlin UI 的 TFT GLCD 面板
// Panel connected to main board by SPI or I2C interface.    |面板通过SPI或I2C接口与主板连接。 
// See https://github.com/Serhiy-K/TFTGLCDAdapter
//
//#define TFTGLCD_PANEL_SPI
//#define TFTGLCD_PANEL_I2C

//=============================================================================
//=======================   LCD / Controller Selection  =======================
//=========================      (Graphical LCDs)      ========================    |（图形 LCD）
//=============================================================================

//
// CONTROLLER TYPE: Graphical 128x64 (DOGM)    |控制器类型：图形 128x64 (DOGM) 
//
// IMPORTANT: The U8glib library is required for Graphical Display!    |重要提示：图形显示需要 U8glib 库！  
//            https://github.com/olikraus/U8glib_Arduino
//
// NOTE: If the LCD is unresponsive you may need to reverse the plugs.    |注意：如果 LCD 无响应，您可能需要颠倒插头。
//

//液晶屏12864
//去掉 REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER 行的注释，可开启12864液晶屏功能。需要u8glib库文件，否则编译无法通过，
// RepRapDiscount FULL GRAPHIC Smart Controller    |全图形智能控制器
// https://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

//
// ReprapWorld Graphical LCD    |ReprapWorld 图形 LCD
// https://reprapworld.com/?products_details&products_id/1218
//
//#define REPRAPWORLD_GRAPHICAL_LCD

//
// Activate one of these if you have a Panucatt Devices    |如果您有 Panucatt 设备，请激活其中一项
// Viki 2.0 or mini Viki with Graphic LCD    |Viki 2.0 或带有图形 LCD 的迷你 Viki
// https://www.panucatt.com
//
//#define VIKI2
//#define miniVIKI

//
// MakerLab Mini Panel with graphic    |带图形的 MakerLab 迷你面板 
// controller and SD support - https://reprap.org/wiki/Mini_panel
//
//#define MINIPANEL

//
// MaKr3d Makr-Panel with graphic controller and SD support.    |MaKr3d Makr-Panel 具有图形控制器和 SD 支持。
// https://reprap.org/wiki/MaKr3d_MaKrPanel
//
//#define MAKRPANEL

//
// Adafruit ST7565 Full Graphic Controller.    |Adafruit ST7565 全图形控制器。
// https://github.com/eboston/Adafruit-ST7565-Full-Graphic-Controller/
//
//#define ELB_FULL_GRAPHIC_CONTROLLER

//
// BQ LCD Smart Controller shipped by    |BQ液晶智能控制器发货|
// default with the BQ Hephestos 2 and Witbox 2.    |默认使用 BQ Hephestos 2 和 Witbox 2。
//
//#define BQ_LCD_SMART_CONTROLLER

//
// Cartesio UI    |Cartesio 用户界面
// http://mauk.cc/webshop/cartesio-shop/electronics/user-interface
//
//#define CARTESIO_UI

//
// LCD for Melzi Card with Graphical LCD    |带图形 LCD 的 Melzi 卡 LCD
//
//#define LCD_FOR_MELZI

//
// Original Ulticontroller from Ultimaker 2 printer with SSD1309 I2C display and encoder    |Ultimaker 2 打印机的原装 Ulticontroller，带 SSD1309 I2C 显示屏和编码器
// https://github.com/Ultimaker/Ultimaker2/tree/master/1249_Ulticontroller_Board_(x1)
//
//#define ULTI_CONTROLLER

//
// MKS MINI12864 with graphic controller and SD support    |具有图形控制器和 SD 支持的 MKS MINI12864
// https://reprap.org/wiki/MKS_MINI_12864
//
//#define MKS_MINI_12864

//
// MKS LCD12864A/B with graphic controller and SD support. Follows MKS_MINI_12864 pinout.    |MKS LCD12864A/B 具有图形控制器和 SD 支持。遵循 MKS_MINI_12864 引脚排列。
// https://www.aliexpress.com/item/33018110072.html
//
//#define MKS_LCD12864

//
// FYSETC variant of the MINI12864 graphic controller with SD support    |MINI12864 图形控制器的 FYSETC 变体，支持 SD
// https://wiki.fysetc.com/Mini12864_Panel/
//
#define FYSETC_MINI_12864_X_X    // Type C/D/E/F. No tunable RGB Backlight by default    |C/D/E/F 型。默认情况下没有可调 RGB 背光 
//#define FYSETC_MINI_12864_1_2    // Type C/D/E/F. Simple RGB Backlight (always on)    |类型 C/D/E/F。简单的 RGB 背光（常亮）
//#define FYSETC_MINI_12864_2_0    // Type A/B. Discreet RGB Backlight    |类型 A/B。低调的 RGB 背光
//#define FYSETC_MINI_12864_2_1    // Type A/B. NeoPixel RGB Backlight    |类型 A/B。 NeoPixel RGB 背光
//#define FYSETC_GENERIC_12864_1_1 // Larger display with basic ON/OFF backlight.    |具有基本开/关背光的更大显示屏。

//
// Factory display for Creality CR-10    | Creality CR-10 工厂展示 
// https://www.aliexpress.com/item/32833148327.html
//
// This is RAMPS-compatible using a single 10-pin connector.    |它使用单个 10 针连接器与 RAMPS 兼容。
// (For CR-10 owners who want to replace the Melzi Creality board but retain the display)    |（适用于想要更换 Melzi Creality 板但保留显示屏的 CR-10 用户）
//
//#define CR10_STOCKDISPLAY

//
// Ender-2 OEM display, a variant of the MKS_MINI_12864    |Ender-2 OEM 显示器，MKS_MINI_12864 的变体
//
//#define ENDER2_STOCKDISPLAY

//
// ANET and Tronxy Graphical Controller    | ANET 和 Tronxy 图形控制器
//
// Anet 128x64 full graphics lcd with rotary encoder as used on Anet A6    |Anet 128x64 全图形液晶显示屏，带旋转编码器，如 Anet A6 上使用的那样
// A clone of the RepRapDiscount full graphics display but with    |RepRapDiscount 完整图形显示的克隆，但带有
// different pins/wiring (see pins_ANET_10.h).    | 不同的引脚/接线（参见pins_ANET_10.h）。
//
//#define ANET_FULL_GRAPHICS_LCD

//
// AZSMZ 12864 LCD with SD    |
// https://www.aliexpress.com/item/32837222770.html
//
//#define AZSMZ_12864

//
// Silvergate GLCD controller    |
// https://github.com/android444/Silvergate
//
//#define SILVER_GATE_GLCD_CONTROLLER

//=============================================================================
//==============================  OLED Displays  ==============================    |OLED 显示屏
//=============================================================================

//
// SSD1306 OLED full graphics generic display    |SSD1306 OLED全显卡通用显示器
//
//#define U8GLIB_SSD1306

//
// SAV OLEd LCD module support using either SSD1306 or SH1106 based LCD modules    |SAV OLED LCD 模块支持使用基于 SSD1306 或 SH1106 的 LCD 模块
//
//#define SAV_3DGLCD
#if ENABLED(SAV_3DGLCD)
  #define U8GLIB_SSD1306
  //#define U8GLIB_SH1106
#endif

//
// TinyBoy2 128x64 OLED / Encoder Panel    |TinyBoy2 128x64 OLED /编码器面板
//
//#define OLED_PANEL_TINYBOY2

//
// MKS OLED 1.3" 128×64 FULL GRAPHICS CONTROLLER    |MKS OLED 1.3" 128×64 全图形控制器 
// https://reprap.org/wiki/MKS_12864OLED
//
// Tiny, but very sharp OLED display    |微小但非常清晰的 OLED 显示屏 
//
//#define MKS_12864OLED          // Uses the SH1106 controller (default)    |使用 SH1106 控制器（默认）
//#define MKS_12864OLED_SSD1306  // Uses the SSD1306 controller    |使用SSD1306控制器 

//
// Zonestar OLED 128×64 FULL GRAPHICS CONTROLLER    | Zonestar OLED 128×64 全图形控制器
//
//#define ZONESTAR_12864LCD           // Graphical (DOGM) with ST7920 controller    |带 ST7920 控制器的图形 (DOGM) 
//#define ZONESTAR_12864OLED          // 1.3" OLED with SH1106 controller (default)    |1.3" OLED，带 SH1106 控制器（默认）
//#define ZONESTAR_12864OLED_SSD1306  // 0.96" OLED with SSD1306 controller    |0.96" OLED，带 SSD1306 控制器 

//
// Einstart S OLED SSD1306    |
//
//#define U8GLIB_SH1106_EINSTART

//
// Overlord OLED display/controller with i2c buzzer and LEDs    |带 i2c 蜂鸣器和 LED 的 Overlord OLED 显示屏/控制器
//
//#define OVERLORD_OLED

//
// FYSETC OLED 2.42" 128×64 FULL GRAPHICS CONTROLLER with WS2812 RGB    |FYSETC OLED 2.42" 128×64 全图形控制器 带 WS2812 RGB 
// Where to find : https://www.aliexpress.com/item/4000345255731.html
//#define FYSETC_242_OLED_12864   // Uses the SSD1309 controller

//=============================================================================
//========================== Extensible UI Displays ===========================    |可扩展的 UI 显示
//=============================================================================

//
// DGUS Touch Display with DWIN OS. (Choose one.)    |带 DWIN 操作系统的 DGUS 触摸显示屏。 （选择一项。）
// ORIGIN : https://www.aliexpress.com/item/32993409517.html
// FYSETC : https://www.aliexpress.com/item/32961471929.html
//
//#define DGUS_LCD_UI_ORIGIN
//#define DGUS_LCD_UI_FYSETC
//#define DGUS_LCD_UI_HIPRECY

//
// Touch-screen LCD for Malyan M200/M300 printers    |适用于 Malyan M200/M300 打印机的触摸屏 LCD
//
//#define MALYAN_LCD
#if ENABLED(MALYAN_LCD)
  #define LCD_SERIAL_PORT 1  // Default is 1 for Malyan M200    |
#endif

//
// Touch UI for FTDI EVE (FT800/FT810) displays    |FTDI EVE (FT800/FT810) 显示器的触摸 UI
// See Configuration_adv.h for all configuration options.    |有关所有配置选项，请参阅 Configuration_adv.h。 
//
//#define TOUCH_UI_FTDI_EVE

//
// Touch-screen LCD for Anycubic printers    | 适用于 Anycubic 打印机的触摸屏 LCD
//
//#define ANYCUBIC_LCD_I3MEGA
//#define ANYCUBIC_LCD_CHIRON
#if EITHER(ANYCUBIC_LCD_I3MEGA, ANYCUBIC_LCD_CHIRON)
  #define LCD_SERIAL_PORT 3  // Default is 3 for Anycubic
  //#define ANYCUBIC_LCD_DEBUG
#endif

//
// Third-party or vendor-customized controller interfaces.    |第三方或供应商定制的控制器接口。
// Sources should be installed in 'src/lcd/extensible_ui'.    |源应安装在“src/lcd/extensible_ui”中。
//
//#define EXTENSIBLE_UI

#if ENABLED(EXTENSIBLE_UI)
  //#define EXTUI_LOCAL_BEEPER // Enables use of local Beeper pin with external display    |允许在外部显示器上使用本地蜂鸣器引脚
#endif

//=============================================================================
//=============================== Graphical TFTs ==============================    |图形 TFT 
//=============================================================================

/**
 * TFT Type - Select your Display type    |TFT 类型 -选择您的显示器类型
 *
 * Available options are:    |可用选项有
 *   MKS_TS35_V2_0,
 *   MKS_ROBIN_TFT24, MKS_ROBIN_TFT28, MKS_ROBIN_TFT32, MKS_ROBIN_TFT35,
 *   MKS_ROBIN_TFT43, MKS_ROBIN_TFT_V1_1R
 *   TFT_TRONXY_X5SA, ANYCUBIC_TFT35, LONGER_LK_TFT28
 *   TFT_GENERIC
 *
 * For TFT_GENERIC, you need to configure these 3 options:    |对于 TFT_GENERIC，您需要配置以下 3 个选项
 *   Driver:     TFT_DRIVER
 *               Current Drivers are: AUTO, ST7735, ST7789, ST7796, R61505, ILI9328, ILI9341, ILI9488
 *   Resolution: TFT_WIDTH and TFT_HEIGHT
 *   Interface:  TFT_INTERFACE_FSMC or TFT_INTERFACE_SPI
 */
//#define TFT_GENERIC

/**
 * TFT UI - User Interface Selection. Enable one of the following options:    |TFT UI -用户界面选择。启用以下选项之一
 *
 *   TFT_CLASSIC_UI - Emulated DOGM - 128x64 Upscaled    |TFT_CLASSIC_UI -模拟 DOGM -128x64 放大 
 *   TFT_COLOR_UI   - Marlin Default Menus, Touch Friendly, using full TFT capabilities    |TFT_COLOR_UI -Marlin 默认菜单，触摸友好，使用完整的 TFT 功能
 *   TFT_LVGL_UI    - A Modern UI using LVGL    |TFT_LVGL_UI -使用 LVGL 的现代 UI
 *
 *   For LVGL_UI also copy the 'assets' folder from the build directory to the    |对于 LVGL_UI，还将“assets”文件夹从构建目录复制到
 *   root of your SD card, together with the compiled firmware.    |SD卡的根目录，以及编译好的固件。 
 */
//#define TFT_CLASSIC_UI
//#define TFT_COLOR_UI
//#define TFT_LVGL_UI

/**
 * TFT Rotation. Set to one of the following values:    |TFT 旋转。设置为以下值之一
 *
 *   TFT_ROTATE_90,  TFT_ROTATE_90_MIRROR_X,  TFT_ROTATE_90_MIRROR_Y,
 *   TFT_ROTATE_180, TFT_ROTATE_180_MIRROR_X, TFT_ROTATE_180_MIRROR_Y,
 *   TFT_ROTATE_270, TFT_ROTATE_270_MIRROR_X, TFT_ROTATE_270_MIRROR_Y,
 *   TFT_MIRROR_X, TFT_MIRROR_Y, TFT_NO_ROTATION
 */
//#define TFT_ROTATION TFT_NO_ROTATION

//=============================================================================
//============================  Other Controllers  ============================    |其他控制器
//=============================================================================

//
// Ender-3 v2 OEM display. A DWIN display with Rotary Encoder.    |Ender-3 v2 OEM 显示器。带旋转编码器的 DWIN 显示器。
//
//#define DWIN_CREALITY_LCD

//
// ADS7843/XPT2046 ADC Touchscreen such as ILI9341 2.8    |ADS7843/XPT2046 ADC触摸屏如ILI9341 2.8
//
//#define TOUCH_SCREEN
#if ENABLED(TOUCH_SCREEN)
  #define BUTTON_DELAY_EDIT  50 // (ms) Button repeat delay for edit screens    |(ms) 编辑屏幕的按钮重复延迟
  #define BUTTON_DELAY_MENU 250 // (ms) Button repeat delay for menus    |(ms) 菜单按钮重复延迟

  #define TOUCH_SCREEN_CALIBRATION

  //#define XPT2046_X_CALIBRATION 12316
  //#define XPT2046_Y_CALIBRATION -8981
  //#define XPT2046_X_OFFSET        -43
  //#define XPT2046_Y_OFFSET        257
#endif

//
// RepRapWorld REPRAPWORLD_KEYPAD v1.1
// https://reprapworld.com/products/electronics/ramps/keypad_v1_0_fully_assembled/
//
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // (mm) Distance to move per key-press    |(mm) 每次按键移动的距离

//=============================================================================
//=============================== Extra Features ==============================    |额外功能
//=============================================================================

// @section extras

// Set number of user-controlled fans. Disable to use all board-defined fans.    |设置用户控制的风扇数量。禁止使用所有主板定义的风扇。
// :[1,2,3,4,5,6,7,8]
//#define NUM_M106_FANS 1

// Increase the FAN PWM frequency. Removes the PWM noise but increases heating in the FET/Arduino    |增加风扇 PWM 频率。消除 PWM 噪声，但会增加 FET/Arduino 的发热
//#define FAST_PWM_FAN

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency    |使用软件 PWM 来驱动风扇，就像加热器一样。这使用了非常低的频率|
// which is not as annoying as with the hardware PWM. On the other hand, if this frequency    |这不像硬件 PWM 那样令人烦恼。另一方面，如果这个频率
// is too low, you should also increment SOFT_PWM_SCALE.    |太低，您还应该增加 SOFT_PWM_SCALE。
#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,    |将其增加 1 将使软件 PWM 频率加倍，
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.    |如果启用了 FAN_SOFT_PWM，则会影响加热器和风扇。
// However, control resolution will be halved for each increment;    |然而，每次增量控制分辨率将减半；
// at zero value, there are 128 effective control positions.    |零值时，有 128 个有效控制位置。 
// :[0,1,2,3,4,5,6,7]
#define SOFT_PWM_SCALE 2

// If SOFT_PWM_SCALE is set to a value higher than 0, dithering can    |如果 SOFT_PWM_SCALE 设置为大于 0 的值，则可以进行抖动 
// be used to mitigate the associated resolution loss. If enabled,    |用于减轻相关的分辨率损失。如果启用，
// some of the PWM cycles are stretched so on average the desired    |一些 PWM 周期被延长，因此平均而言，所需的
// duty cycle is attained.    |达到占空比。
#define SOFT_PWM_DITHER

// Temperature status LEDs that display the hotend and bed temperature.    |温度状态 LED 显示热端和床温度。 
// If all hotends, bed temperature, and target temperature are under 54C    |如果所有热端、床温和目标温度均低于 54C 
// then the BLUE led is on. Otherwise the RED led is on. (1C hysteresis)    |然后蓝色 LED 亮起。否则红色 LED 亮起。 （1C 迟滞）
//#define TEMP_STAT_LEDS

// Support for the BariCUDA Paste Extruder    |支持 BariCUDA 糊料挤出机
//#define BARICUDA

// Support for BlinkM/CyzRgb    |支持 BlinkM/CyzRgb
//#define BLINKM

// Support for PCA9632 PWM LED driver
//#define PCA9632

// Support for PCA9533 PWM LED driver
//#define PCA9533

/**
 * RGB LED / LED Strip Control    |RGB LED /LED 灯带控制 
 *
 * Enable support for an RGB LED connected to 5V digital pins, or    |启用对连接到 5V 数字引脚的 RGB LED 的支持，或
 * an RGB Strip connected to MOSFETs controlled by digital pins.    |连接到由数字引脚控制的 MOSFET 的 RGB 灯带。
 *
 * Adds the M150 command to set the LED (or LED strip) color.    |新增M150指令设置LED（或LED灯条）颜色。
 * If pins are PWM capable (e.g., 4, 5, 6, 11) then a range of    |如果引脚支持 PWM（例如 4、5、6、11），则 | 的范围
 * luminance values can be set from 0 to 255.    |亮度值可设置为 0 到 255。
 * For NeoPixel LED an overall brightness parameter is also available.    |对于 NeoPixel LED，还提供整体亮度参数。
 *
 * *** CAUTION ***    |注意
 *  LED Strips require a MOSFET Chip between PWM lines and LEDs,    |LED 灯条需要在 PWM 线路和 LED 之间安装 MOSFET 芯片，
 *  as the Arduino cannot handle the current the LEDs will require.    |因为 Arduino 无法处理 LED 所需的电流。
 *  Failure to follow this precaution can destroy your Arduino!    |不遵守此预防措施可能会损坏您的 Arduino！
 *  NOTE: A separate 5V power supply is required! The NeoPixel LED needs    |注意：需要单独的 5V 电源！ NeoPixel LED 需求
 *  more current than the Arduino 5V linear regulator can produce.    |比 Arduino 5V 线性稳压器可产生的电流更大。
 * *** CAUTION ***
 *
 * LED Type. Enable only one of the following two options.    |LED 类型。仅启用以下两个选项之一。 
 */
//#define RGB_LED
//#define RGBW_LED

#if EITHER(RGB_LED, RGBW_LED)
  //#define RGB_LED_R_PIN 34
  //#define RGB_LED_G_PIN 43
  //#define RGB_LED_B_PIN 35
  //#define RGB_LED_W_PIN -1
#endif

// Support for Adafruit NeoPixel LED driver    |支持 Adafruit NeoPixel LED 驱动器
//#define NEOPIXEL_LED
#if ENABLED(NEOPIXEL_LED)
  #define NEOPIXEL_TYPE   NEO_GRBW // NEO_GRBW / NEO_GRB - four/three channel driver type (defined in Adafruit_NeoPixel.h)    |NEO_GRBW /NEO_GRB -四/三通道驱动程序类型（在 Adafruit_NeoPixel.h 中定义）
  #define NEOPIXEL_PIN     4       // LED driving pin    | LED驱动引脚
  //#define NEOPIXEL2_TYPE NEOPIXEL_TYPE
  //#define NEOPIXEL2_PIN    5
  #define NEOPIXEL_PIXELS 30       // Number of LEDs in the strip. (Longest strip when NEOPIXEL2_SEPARATE is disabled.)    |灯条中 LED 的数量。 （禁用 NEOPIXEL2_SEPARATE 时的最长条带。）
  #define NEOPIXEL_IS_SEQUENTIAL   // Sequential display for temperature change - LED by LED. Disable to change all LEDs at once.    |温度变化的顺序显示 -LED 接 LED。禁止同时更改所有 LED。
  #define NEOPIXEL_BRIGHTNESS 127  // Initial brightness (0-255)    |初始亮度（0-255） 
  //#define NEOPIXEL_STARTUP_TEST  // Cycle through colors at startup    |在启动时循环切换颜色

  // Support for second Adafruit NeoPixel LED driver controlled with M150 S1 ...    |支持使用 M150 S1 控制的第二个 Adafruit NeoPixel LED 驱动器
  //#define NEOPIXEL2_SEPARATE
  #if ENABLED(NEOPIXEL2_SEPARATE)
    #define NEOPIXEL2_PIXELS      15  // Number of LEDs in the second strip    |第二个灯带中的 LED 数量
    #define NEOPIXEL2_BRIGHTNESS 127  // Initial brightness (0-255)    |初始亮度（0-255）
    #define NEOPIXEL2_STARTUP_TEST    // Cycle through colors at startup    |启动时循环切换颜色
  #else
    //#define NEOPIXEL2_INSERIES      // Default behavior is NeoPixel 2 in parallel    |默认行为是并行 NeoPixel 2 
  #endif

  // Use a single NeoPixel LED for static (background) lighting    |使用单个 NeoPixel LED 进行静态（背景）照明
  //#define NEOPIXEL_BKGD_LED_INDEX  0               // Index of the LED to use    |要使用的 LED 索引
  //#define NEOPIXEL_BKGD_COLOR { 255, 255, 255, 0 } // R, G, B, W  
#endif

/**
 * Printer Event LEDs    |打印机事件 LED 
 *
 * During printing, the LEDs will reflect the printer status:    |打印过程中，LED 会反映打印机状态：
 *
 *  - Gradually change from blue to violet as the heated bed gets to target temp    |随着加热床达到目标温度，逐渐从蓝色变为紫色 
 *  - Gradually change from violet to red as the hotend gets to temperature    |随着热端达到温度，逐渐从紫色变为红色 
 *  - Change to white to illuminate work surface    |更改为白色以照亮工作表面
 *  - Change to green once print has finished    |打印完成后更改为绿色
 *  - Turn off after the print has finished and the user has pushed a button    |打印完成并且用户按下按钮后关闭
 */
#if ANY(BLINKM, RGB_LED, RGBW_LED, PCA9632, PCA9533, NEOPIXEL_LED)
  #define PRINTER_EVENT_LEDS
#endif

/**
 * Number of servos    |舵机数量
 *
 * For some servo-related options NUM_SERVOS will be set automatically.    |对于一些伺服相关的选项，NUM_SERVOS 会自动设置。
 * Set this manually if there are extra servos needing manual control.    |如果有额外的舵机需要手动控制，请手动设置。
 * Set to 0 to turn off servo support.    |设置为 0 关闭伺服支持。
 */
#define NUM_SERVOS 1 // Servo index starts with 0 for M280 command    |M280指令伺服索引从0开始

// (ms) Delay  before the next move will start, to give the servo time to reach its target angle.    |(ms) 下一次移动开始之前的延迟，以便伺服器有时间达到其目标角度。  
// 300ms is a good value but you can try less delay.    |300 毫秒是一个不错的值，但您可以尝试减少延迟。
// If the servo can't reach the requested position, increase it.    |如果舵机无法到达要求的位置，则增加它。 
#define SERVO_DELAY { 300 }

// Only power servos during movement, otherwise leave off to prevent jitter    |仅在运动过程中为舵机供电，否则停止供电以防止抖动
//#define DEACTIVATE_SERVOS_AFTER_MOVE    |

// Edit servo angles with M281 and save to EEPROM with M500    |使用 M281 编辑伺服角度并使用 M500 保存到 EEPROM
//#define EDITABLE_SERVO_ANGLES    |可编辑私服角度

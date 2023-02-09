/**
  mkZoeRobotics_command.h 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 5 August 2020 by Mike Kim
*/
#ifndef _MKZOEROBOTICS_COMMAND_H_
#define _MKZOEROBOTICS_COMMAND_H_
#include <stdint.h>
#include <stdio.h>
#include "mkZoeRobotics_globalDataStruct.h"
#define BLOCK_BUFFER_SIZE 16 // maximize block buffer
// #define MAX_CMD_SIZE 96
// #define BUFSIZE 12
#define NUM_AXIS 5
#define AXIS_RELATIVE_MODES {false, false, false, false, false}
// Travel limits after homing
#define X_MAX_POS 205
#define X_MIN_POS 0
#define Y_MAX_POS 205
#define Y_MIN_POS 0
#define Z_MAX_POS 200
#define Z_MIN_POS 0
#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
#define DEFAULT_ESTEP_PER_REV 489.6  //default setting for the extruder step calibration  1:15.3 gear
#define DEFAULT_PSTEP_PER_MM 375.3  //default setting for the puller motor step calibration  1:14 gear

#define DEFAULT_AXIS_STEPS_PER_UNIT   {40,40,40,DEFAULT_ESTEP_PER_REV, DEFAULT_PSTEP_PER_MM}    
#define DEFAULT_MAX_FEEDRATE          {500, 500, 5, 50, 50}    // (mm/sec) 

#define DEFAULT_MOTOR_ACCELERATION  200   // E and P max acceleration in mm/s^2 - controls extruder acceleration
#define DEFAULT_EJERK                 1.0    // (mm/sec)
#define DEFAULT_PJERK				  1.0	// (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,100,900,900}    // X, Y, Z, E,P maximum start speed for accelerated moves. E default values are good for Skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)
// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e, steps_p;  // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder
  #ifdef ADVANCE
    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;
  #endif

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec 
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec 
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block  
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  unsigned long winder_speed;

  volatile char busy;
} block_t;
class MKCommand 
{
    public:
    char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
    char tmpBuffer[128];
    int buflen = 0;

    uint8_t serial_char;
    int serial_count = 0;
    bool comment_mode = false;
    int bufindr = 0;
    int bufindw = 0;
    char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc

    enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, S_AXIS=3, E_AXIS=4};// S: Shoulder, E: Elbow
    const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'S', 'E'};
    float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0, 0.0};
    CIRCLEProfile circleProfile;
    SPIRALProfile spiralProfile;
    LINEARProfile linearProfile;
    EEROTATIONProfile eeRotationProfile;
    SPEEDProfile speedProfile;
    uint32_t current_steps = 0;
    // uint32_t dest_steps = 0; // total steps from accel through const speed to decel...
    // uint32_t dest_Na = 0;// total pulse(step) of Acceleration 
    // uint32_t dest_Nac = 0;// total pulse(step) from acceleration to end of constant speed
    // uint32_t dest_Nd = 0;// total pulse(step) fo deceleration
    // uint32_t dest_time0 = 0; // the begining of time period of acceleration.
    // int8_t dest_dir= 1;// direction of stepper motor

    bool axis_relative_modes[NUM_AXIS] = AXIS_RELATIVE_MODES;
    float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0}; 
    float feedrate = 150.0, next_feedrate, saved_feedrate;
    bool relative_mode = false;
    uint32_t previous_millis_cmd = 0;
    float offset[3] = {0.0, 0.0, 0.0};
    float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
    float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
    uint32_t max_acceleration_units_per_sq_second[5]; // Use M201 to override by software
    uint32_t axis_steps_per_sqr_second[NUM_AXIS];
    float max_feedrate[5]={150}; // set the max speeds[RAD/SEC]
    bool check_endstops = true;
    bool home_all_axis = true;
    float axis_steps_per_unit[5]={40,40,40,40,40};
    block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
    volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
    volatile unsigned char block_buffer_tail;           // Index of the block to process now
    volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0, 0};  //FMM added P_AXIS
    volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1, 1};  //FMM added P_AXIS
    // The current position of the tool in absolute steps
    long position[5];   //rescaled from extern when axis_steps_per_unit are changed by gcode
    static float previous_speed[5]; // Speed of previous path line segment
    static float previous_nominal_speed; // Nominal speed of previous path line segment
    const unsigned int dropsegments=5;
    float acceleration=80.0;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
    float retract_acceleration; 
    float max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
    float max_z_jerk;
    float max_e_jerk;
    float mintravelfeedrate;
    float add_homeing[3]={0,0,0};
    volatile bool endstop_x_hit=false;
    volatile bool endstop_y_hit=false;
    volatile bool endstop_z_hit=false;
    int(*callback_gen_linear_profile)(LINEARProfile &linearProfile)=NULL;//
    int(*callback_gen_EErotation_profile)(EEROTATIONProfile &eeRotationProfile)=NULL;
    int(*callback_gen_circle_profile)(CIRCLEProfile &circleProfile)=NULL;
    int(*callback_gen_spiral_profile)(SPIRALProfile &spiralProfile)=NULL;
    void(*callback_gen_speed_profile)(uint16_t num, double distance,  double speed, double accel, double decel) =NULL;
    void(*callback_set_speed_profile)(SPEEDProfile &speedProfile) =NULL;
    void(*callback_update_speed_only)(uint16_t num, uint32_t steps) =NULL;
    ///////////////////////////////////////////////////////////////////
    public:
    MKCommand()
    {

    }
    ~MKCommand() 
    {

    }
     void setCallBack_gen_linear_prfile( int(*callback)(LINEARProfile &linearProfile))
    {
        callback_gen_linear_profile=callback;
    }
    void setCallback_gen_EErotation_profile( int(*callback)(EEROTATIONProfile &eeRotationProfile))
    {
        callback_gen_EErotation_profile=callback;
    }
    void setCallBack_gen_circle_prfile( int(*callback)(CIRCLEProfile &circleProfile))
    {
        callback_gen_circle_profile=callback;
    }
      void setCallBack_gen_spiral_prfile( int(*callback)(SPIRALProfile &spiralProfile))
    {
        callback_gen_spiral_profile=callback;
    }
    void setCallBack_gen_speed_profile(void(*callback)(uint16_t num, double distance,  double speed, double accel, double decel))
    {
        callback_gen_speed_profile = callback;
    }
    void setCallBack_set_speed_profile(void(*callback)( SPEEDProfile &speedProfile))
    {
      callback_set_speed_profile = callback;
    }
     void setCallBack_update_speed_only(void(*callback)(uint16_t num,  uint32_t steps))
    {
        callback_update_speed_only = callback;
    } 
    int32_t getAbsStepPos();

    void getCommand();
    void process_commands();
    void getStartMove(int axis_sel);
    bool code_seen(char code);
    float code_value();
    long code_value_long();
    void get_coordinates();
    bool get_speed_profile();
    int get_gen_linear_motion_profile();
    int get_gen_EErotation_motion_profile();
    int get_gen_circular_motion_profile();
    int get_gen_spiral_motion_profile();
    void get_arc_coordinates();
};

extern MKCommand mkCommand;

#endif//_MKZOEROBOTICS_COMMAND_H_
#include "SmoothADS1015.h"
#include "Motor.h"
#include "EnoughTimePassed.h"
#include "SignalLED.h"
#include "WiHomeComm.h"
#include "ConfigFileJSON.h"

#ifndef GATEOPENERSTATEMACHINE_H
#define GATEOPENERSTATEMACHINE_H

#define NAVG 20
#define NVM_VALID_KEY 224
#define INVALID_POS 32767
#define INVALID_AUTO_CLOSE_TIME 0
#define INVALID_MAX_IMOTOR 0
#define INVALID_MAX_ON_TIME 0
#define POS_TOL 3
#define IMOTOR_DELAY 1000
#define IMOTOR_ZERO_TOL 5
#define LED_GO_DELAY 3000

// Possible HomeKit values for CURRENT_DOOR_STATE, TARGET_DOOR_STATE, and OBSTRUCTION_DETECTED:
#define HK_CURRENT_DOOR_STATE_OPEN 0
#define HK_CURRENT_DOOR_STATE_CLOSED 1
#define HK_CURRENT_DOOR_STATE_OPENING 2
#define HK_CURRENT_DOOR_STATE_CLOSING 3
#define HK_CURRENT_DOOR_STATE_STOPPED 4
#define HK_CURRENT_DOOR_STATE_UNKNOWN 255
#define HK_TARGET_DOOR_STATE_OPEN 0
#define HK_TARGET_DOOR_STATE_CLOSED 1
#define HK_TARGET_DOOR_STATE_UNKNOWN 255
#define HK_OBSTRUCTION_DETECTED true
#define HK_NO_OBSTRUCTION_DETECTED false

class GateOpenerStateMachine
{
  private:
    int current_state;
    int last_state;
    unsigned int pot_pin;
    unsigned int isens_pin;
    int open_pos;
    int closed_pos;
    int current_pos;
    int current_imotor;
    int imotor_offset;
    int max_imotor;
    unsigned long auto_close_time;
    unsigned long max_on_time;
    Motor* motor;
    SmoothADS1015* pos;
    EnoughTimePassed* etp_auto_close;
    EnoughTimePassed* etp_imotor_delay;
    EnoughTimePassed* etp_led_go_delay;
    EnoughTimePassed* etp_max_on_time;
    SignalLED* led_go;
    bool past_imotor_delay;
    bool just_stopped_flag;
    int nvm_offset;
    ConfigFileJSON* config;
    void nvm_save();
    void nvm_load();
  public:
    GateOpenerStateMachine(unsigned int _motor_pinA, unsigned int _motor_pinB,
      unsigned int _pot_pin, unsigned int _isens_pin, unsigned int _led_pin, int _nvm_offset);
    ~GateOpenerStateMachine();
    void set_state(int state);
    int get_state();
    int get_position();
    int get_position_percent();
    int get_hk_current_door_state();
    int get_hk_target_door_state();
    bool get_hk_obstruction_detected();
    int get_imotor();
    int get_max_imotor();
    void set_max_imotor(int _max_imotor);
    void set_auto_close_time(unsigned long _auto_close_time);
    unsigned long get_auto_close_time();
    void set_max_on_time(unsigned long _max_on_time);
    unsigned long get_max_on_time();
    bool learn_closed_position();
    bool learn_open_position();
    void set_open_position(int position);
    void set_closed_position(int position);
    int get_closed_position();
    int get_open_position();
    bool valid_closed_position();
    bool valid_open_position();
    bool just_stopped();
    bool is_running();
    void open();
    void close();
    void stop();
    int cycle();
    void check();
    void dump_flash(int addr, int cnt);
};

#endif // GATEOPENERSTATEMACHINE_H

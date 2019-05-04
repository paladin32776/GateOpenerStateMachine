#include "SmoothADS1015.h"
#include "Motor.h"
#include "EnoughTimePassed.h"
#include "SignalLED.h"
#include <EEPROM.h>
#define NAVG 20
#define NVM_VALID_KEY 223
#define INVALID_POS 32767
#define INVALID_AUTO_CLOSE_TIME 0
#define INVALID_MAX_IMOTOR 0
#define POS_TOL 3
#define IMOTOR_DELAY 1000
#define IMOTOR_ZERO_TOL 5
#define LED_GO_DELAY 3000

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
    Motor* motor;
    SmoothADS1015* pos;
    EnoughTimePassed* etp_auto_close;
    EnoughTimePassed* etp_imotor_delay;
    EnoughTimePassed* etp_led_go_delay;
    SignalLED* led_go;
    bool past_imotor_delay;
    int nvm_offset;
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
    int get_imotor();
    int get_max_imotor();
    void set_max_imotor(int _max_imotor);
    void set_auto_close_time(unsigned long _auto_close_time);
    unsigned long get_auto_close_time();
    bool learn_closed_position();
    bool learn_open_position();
    void set_open_position(int position);
    void set_closed_position(int position);
    int get_closed_position();
    int get_open_position();
    bool valid_closed_position();
    bool valid_open_position();
    void open();
    void close();
    void stop();
    int cycle();
    void check();
    void dump_flash(int addr, int cnt);
};

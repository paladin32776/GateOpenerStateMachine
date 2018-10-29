# State machine to implement gate opener control for Progressive Automations linerar actuator with self-limiting end-switches.

# Methods:

GateOpenerStateMachine(unsigned int _motor_pinA, unsigned int _motor_pinB,
	unsigned int _pot_pin, unsigned int _isens_pin, unsigned int _led_pin, int _nvm_offset);

// Constructor - arguments:
// _motor_pinA, _motor_pinB ... pin numbers for motor control (Arduiono pins)
// _pot_pin, _isens_pin ... position and current sensing (using an ADS1015, pins=0-3)
// _led_pin ... led pin (Arduino pin)
// _nvm_offset ... offset to write non-volatile user data into the flash (can be 0 if nothing else is stored in flash)

~GateOpenerStateMachine();
// Destructor

void set_state(int state);
// Set state (-1|0|1).

int get_state();
// Set current state (-1|0|1).

int get_position();
// Read current position of actuator

int get_position_percent();
// Read current position of actuator as percentage: 0% = fully open/retracted, 100% = closed / extended to stored closed position

int get_imotor();
// Read motor current in arbitrary units

int get_max_imotor();
// Read motor motor current limit, arbitrary units, same as imotor. If current exceeds this limit, motor will stop. 0 = no limit.

void set_max_imotor(int _max_imotor);
// Set motor motor current limit, arbitrary units, same as imotor. If current exceeds this limit, motor will stop. 0 = no limit.

void set_auto_close_time(unsigned long _auto_close_time);
// Set auto close time in ms. Gate will close this much time after it has been fully opened. 0 = no auto close.

unsigned long get_auto_close_time();
// Get auto close time in ms. Gate will close this much time after it has been fully opened. 0 = no auto close.

bool learn_closed_position();
// If there is no closed position stored, make the current position the closed position. 
// If there is a closed position stored, clear it.
// Only works if state is 0 (actuator is stopped).
// Returns true of closed position has been successfully stored, otherwise false.

int get_closed_position();
// Read stored closed position.

int get_open_position();
// Read stored open position. This is only used for the calculation of the position percentage. What determines the open position is the self-limiting end-switch.

bool valid_closed_position();
// Check if a closed position is stored. If so, return true, otherwise false.

void open();
// Open gate.

void close();
// Close gate.

int cycle();
// Cycle state (for push button operation)

void check();
// Function to be called in main program loop. Contains all operational logic.

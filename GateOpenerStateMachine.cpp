#include "GateOpenerStateMachine.h"
extern WiHomeComm whc;

void GateOpenerStateMachine::dump_flash(int addr, int cnt)
{
  config->dump();
}

void GateOpenerStateMachine::nvm_save()
{
  config->set("closed_pos", closed_pos, "open_pos", open_pos, "auto_close_time", auto_close_time,
              "max_imotor", max_imotor, "max_on_time", max_on_time);
  config->dump();
}

void GateOpenerStateMachine::nvm_load()
{
  closed_pos = INVALID_POS;
  open_pos = INVALID_POS;
  auto_close_time = INVALID_AUTO_CLOSE_TIME;
  max_imotor = INVALID_MAX_IMOTOR;
  max_on_time = INVALID_MAX_ON_TIME;
  config->get("closed_pos", &closed_pos, "open_pos", &open_pos, "auto_close_time", &auto_close_time,
              "max_imotor", &max_imotor, "max_on_time", &max_on_time);
  Serial.printf("nvm_load() result:\nclosed_pos=%d  open_pos=%d  auto_close_time=%d  max_imotor=%d  max_on_time=%d\n",
                closed_pos, open_pos, auto_close_time, max_imotor, max_on_time);
}

GateOpenerStateMachine::GateOpenerStateMachine(unsigned int _motor_pinA, unsigned int _motor_pinB,
  unsigned int _pot_pin, unsigned int _isens_pin, unsigned int _led_pin, int _nvm_offset)
{
  current_state = 0;
  last_state = 1;
  pot_pin = _pot_pin;
  isens_pin = _isens_pin;
  nvm_offset = _nvm_offset;
  config = new ConfigFileJSON("gateopener.cfg");
  nvm_load();
  motor = new Motor(_motor_pinA, _motor_pinB);
  pos = new SmoothADS1015(14,2); // SDA GPIO pin, SCL GPIO pin
  pos->setup(pot_pin,NAVG);
  pos->setup(isens_pin,NAVG);
  imotor_offset = pos->read(isens_pin);
  etp_auto_close = new EnoughTimePassed(auto_close_time);
  etp_auto_close->event();
  etp_max_on_time = new EnoughTimePassed(max_on_time);
  etp_imotor_delay = new EnoughTimePassed(IMOTOR_DELAY);
  past_imotor_delay = true;
  current_pos = pos->read(pot_pin);
  led_go = new SignalLED(_led_pin, SLED_OFF, false);
  etp_led_go_delay = new EnoughTimePassed(LED_GO_DELAY);
  just_stopped_flag = false;
}

GateOpenerStateMachine::~GateOpenerStateMachine()
{
  delete motor;
  delete pos;
  delete etp_auto_close;
  delete etp_imotor_delay;
  delete led_go;
  delete etp_led_go_delay;
}

void GateOpenerStateMachine::set_state(int state)
{
  if (state!=current_state)
  {
    last_state = current_state;
    current_state = state;
    past_imotor_delay = false;
    etp_imotor_delay->event();
    etp_max_on_time->event();
    if (state==0)
      just_stopped_flag=true;
  }
}

int GateOpenerStateMachine::get_state()
{
  return current_state;
}

int GateOpenerStateMachine::get_position()
{
  return current_pos;
}

int GateOpenerStateMachine::get_position_percent()
{
  if ((closed_pos==INVALID_POS) && (open_pos==INVALID_POS))
    return INVALID_POS;
  else if ((closed_pos==INVALID_POS) && current_pos<=open_pos)
    return 0;
  else if ((open_pos==INVALID_POS) && (current_pos>=closed_pos))
    return 100;
  else
  {
    int pos = int(round(float(current_pos-open_pos)/float(closed_pos-open_pos)*100));
    if (pos>100-POS_TOL)
      pos=100;
    if (pos<0+POS_TOL)
      pos=0;
    return pos;
  }
}

int GateOpenerStateMachine::get_hk_current_door_state()
{
		switch(get_state())
		{
			case 0:
				switch (get_position_percent())
				{
					case 0:
						return HK_CURRENT_DOOR_STATE_OPEN;
					case 100:
						return HK_CURRENT_DOOR_STATE_CLOSED;
					default:
						return HK_CURRENT_DOOR_STATE_STOPPED;
				}
			case 1:
				return HK_CURRENT_DOOR_STATE_CLOSING;
			case -1:
				return HK_CURRENT_DOOR_STATE_OPENING;
		}
}

int GateOpenerStateMachine::get_hk_target_door_state()
{
	switch (get_state())
	{
		case 1:
			return HK_TARGET_DOOR_STATE_CLOSED;
		case -1:
			return HK_TARGET_DOOR_STATE_OPEN;
		case 0:
			switch(get_position_percent())
			{
				case 0:
					return HK_TARGET_DOOR_STATE_OPEN;
				case 100:
					return HK_TARGET_DOOR_STATE_CLOSED;
				default:
					return HK_TARGET_DOOR_STATE_UNKNOWN;
			}
	}
}

bool GateOpenerStateMachine::get_hk_obstruction_detected()
{
  if (get_state()==0 && get_position_percent()>0 && get_position_percent()<100)
    return HK_OBSTRUCTION_DETECTED;
  else
    return HK_NO_OBSTRUCTION_DETECTED;
}

int GateOpenerStateMachine::get_imotor()
{
  return current_imotor-imotor_offset;
}

int GateOpenerStateMachine::get_max_imotor()
{
  return max_imotor;
}

void GateOpenerStateMachine::set_max_imotor(int _max_imotor)
{
  max_imotor = _max_imotor;
  nvm_save();
  Serial.printf("Max motor current set to %d\n", max_imotor);
}

void GateOpenerStateMachine::set_auto_close_time(unsigned long _auto_close_time)
{
  auto_close_time = _auto_close_time*1000;
  nvm_save();
  etp_auto_close->change_intervall(auto_close_time);
  Serial.printf("Auto close time set to %ds\n", auto_close_time/1000);
  whc.sendJSON("cmd","debug","msg","Auto close set.","auto_close_time",auto_close_time/1000);
}

unsigned long GateOpenerStateMachine::get_auto_close_time()
{
  return auto_close_time/1000;
}

void GateOpenerStateMachine::set_max_on_time(unsigned long _max_on_time)
{
  max_on_time = _max_on_time*1000;
  nvm_save();
  etp_max_on_time->change_intervall(max_on_time);
  Serial.printf("Max on time set to %ds\n", max_on_time/1000);
  whc.sendJSON("cmd","debug","msg","Max on time set.","max_on_time",max_on_time/1000);
}

unsigned long GateOpenerStateMachine::get_max_on_time()
{
  return max_on_time/1000;
}

bool GateOpenerStateMachine::learn_closed_position()
{
  if (current_state==0)
  {
    if (closed_pos==INVALID_POS)
    {
      closed_pos = current_pos;
      led_go->set(SLED_ON);
      etp_led_go_delay->event();
    }
    else
    {
      closed_pos = INVALID_POS;
      led_go->set(SLED_BLINK_FAST);
      etp_led_go_delay->event();
    }
    nvm_save();
  }
  return (closed_pos != INVALID_POS) ? (true) : (false);
}

bool GateOpenerStateMachine::learn_open_position()
{
  if (current_state==0)
  {
    if (open_pos==INVALID_POS)
    {
      open_pos = current_pos;
      led_go->set(SLED_ON);
      etp_led_go_delay->event();
    }
    else
    {
      open_pos = INVALID_POS;
      led_go->set(SLED_BLINK_FAST);
      etp_led_go_delay->event();
    }
    nvm_save();
  }
  return (open_pos != INVALID_POS) ? (true) : (false);
}


void GateOpenerStateMachine::set_open_position(int position)
{
  open_pos = position;
  nvm_save();
}

void GateOpenerStateMachine::set_closed_position(int position)
{
  closed_pos = position;
  nvm_save();
}

int GateOpenerStateMachine::get_closed_position()
{
  return closed_pos;
}

int GateOpenerStateMachine::get_open_position()
{
  return open_pos;
}

bool GateOpenerStateMachine::valid_closed_position()
{
  return (closed_pos != INVALID_POS) ? (true) : (false);
}

bool GateOpenerStateMachine::valid_open_position()
{
  return (open_pos != INVALID_POS) ? (true) : (false);
}

bool GateOpenerStateMachine::just_stopped()
{
  if (just_stopped_flag && past_imotor_delay)
  {
    just_stopped_flag=false;
    return true;
  }
  else
    return false;
}

bool GateOpenerStateMachine::is_running()
{
  return (current_state != 0);
}

void GateOpenerStateMachine::open()
{
  stop();
  set_state(-1);
}

void GateOpenerStateMachine::close()
{
  stop();
  set_state(1);
}

void GateOpenerStateMachine::stop()
{
  if (current_state!=0)
  {
    set_state(0);
  }
}

int GateOpenerStateMachine::cycle()
{
  if (current_state==0)
    set_state(-last_state);
  else
    set_state(0);
  return current_state;
}

void GateOpenerStateMachine::check()
{
  current_pos = pos->read(pot_pin);
  current_imotor = pos->read(isens_pin);

  if (current_state==0 && abs(current_imotor-imotor_offset)>IMOTOR_ZERO_TOL && past_imotor_delay)
  {
    imotor_offset=current_imotor;
    whc.sendJSON("cmd","debug", "msg","Setting new imotor_offset.",
                 "imotor_offset",imotor_offset);
  }

  if (etp_led_go_delay->enough_time())
    led_go->set(SLED_OFF);
  led_go->check();

  if (etp_imotor_delay->enough_time())
    past_imotor_delay = true;

  if (current_pos>open_pos+POS_TOL || current_state!=0 || auto_close_time==INVALID_AUTO_CLOSE_TIME || closed_pos==INVALID_POS || open_pos==INVALID_POS)
    etp_auto_close->event();
  else
    Serial.print("#");

  if (etp_auto_close->enough_time() && auto_close_time!=INVALID_AUTO_CLOSE_TIME && closed_pos!=INVALID_POS && open_pos!=INVALID_POS)
  {
    set_state(1);
    Serial.println("Auto close initiated.");
    whc.sendJSON("cmd","debug","msg","Auto close initiated.");
  }

  // Stop motor if:
  // No open position stored (open_pos==INVALID_POS), and arm ran into open position end switch: (imotor=0) && (current_state=-1) && (past_imotor_delay):
  // or
  // valid open position stored (open_pos!=INVALID_POS), and arm ran below open_position: (current_pos<open_pos+POS_TOL) && (current_state==-1)
  if (((abs(get_imotor())<IMOTOR_ZERO_TOL) && (current_state==-1) && past_imotor_delay && (open_pos==INVALID_POS)) || ((open_pos!=INVALID_POS) && (current_pos<open_pos+POS_TOL) && (current_state==-1)))
  {
    set_state(0);
    led_go->set(SLED_BLINK_FAST_3);
    etp_led_go_delay->event();
    if (open_pos!=INVALID_POS)
    {
      Serial.println("Open position reached.");
      whc.sendJSON("cmd","debug", "msg","Open position reached.",
                   "imotor",get_imotor(), "current_imotor",current_imotor,
                   "imotor_offset", imotor_offset);
    }
    else
    {
      Serial.println("Open end switch position reached.");
      whc.sendJSON("cmd","debug", "msg","Open end switch position reached.",
                   "imotor",get_imotor(), "current_imotor",current_imotor,
                   "imotor_offset", imotor_offset);
      open_pos = current_pos;
      nvm_save();
    }
  }
  if (current_pos>=closed_pos && current_state==1)
  {
    set_state(0);
    led_go->set(SLED_BLINK_FAST_1);
    etp_led_go_delay->event();
    Serial.println("Closed position reached.");
    whc.sendJSON("cmd","debug","msg","Closed position reached.");
  }

  if ((abs(get_imotor())>max_imotor) && max_imotor!=INVALID_MAX_IMOTOR && past_imotor_delay)
  {
    set_state(0);
    led_go->set(SLED_BLINK_SLOW);
    etp_led_go_delay->event();
    Serial.println("Motor current limit!");
    whc.sendJSON("cmd","debug","msg","Motor current limit!");
  }

  if((current_state!=0) && (max_on_time!=INVALID_MAX_ON_TIME) && (etp_max_on_time->enough_time()))
  {
    set_state(0);
    led_go->set(SLED_BLINK_FAST);
    etp_led_go_delay->event();
    Serial.println("On time limit!");
    whc.sendJSON("cmd","debug","msg","On time limit!");
  }

  if (current_state==0)
    motor->stop();
  if (current_state==-1)
    motor->rev();
  if (current_state==1)
    motor->fwd();
}

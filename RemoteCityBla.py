#
# Name: Remote Bla Bla for CityHub
#
# Description:
# Remote City Bla is a Pybricks (https://pybricks.com) program for a LEGO
# PoweredUp CityHub and a LEGO PoweredUp Remote Control where you
# first define how each device answers to remote control button actions
# (config mode) and then you play with your MOC/Set using the defined
# configuration (play mode).
#
# City Hubs have much less memory than Technic Hubs so they cannot run
# Remote Bla Bla config mode. Instead, you have to use a Technic Hub to
# configure your City Hub (that is, use only ports A and B) and save the
# configuration. The configuration is saved in the Remote Control.
# Then you can connect the Remote Control to a City Hub running this program.
#
#
# There is a users manual in PDF avaiable.
# https://github.com/vascolp/RemoteBlaBla
#
# Version: 1.0
#
# Author VascoLP: vascolp.lego@gmail.com
#
# Date: March 2022
#
# Installing:
# Install Pybricks firmware with Remote Bla Bla (this program) included, on a CityHub.
# You should use pybricks firmware version v3.1.0 on 2021-12-16 or later.
#

from micropython import const
from pybricks.hubs import ThisHub
from pybricks.tools import StopWatch
from pybricks.iodevices import PUPDevice
from pybricks.pupdevices import DCMotor, Motor, Remote
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
#from uerrno import ENODEV
from math import trunc

_btn_RP= const(0) # Button.RIGHT_PLUS
_btn_LP= const(1) # Button.LEFT_PLUS
#_btn_RR=         const(2)  # Button.RIGHT (+) in three command play mode
_btn_SRP=const(2) # Shift-Button.RIGHT_PLUS  in four command play mode/config mode
_btn_SLP=const(3) # Shift-Button.LEFT_PLUS   in four command play mode/config mode
_btn_RM= const(4) # Button.RIGHT_MINUS
_btn_LM= const(5) # Button.LEFT_MINUS
#_btn_LR=         const(6)  # Button.LEFT  (-) in three command play mode
_btn_SRM=const(6) # Shift-Button.RIGHT_MINUS in four command play mode/config mode
_btn_SLM=const(7) # Shift-Button.LEFT_MINUS  in four command play mode/config mode

bla_play_color=Color.GREEN*0.8   # Color for play mode
bla_send_color=Color.GREEN*0.8
bla_error_color=Color.RED
bla_dev_init_color=Color.RED*0.4
_bla_auto_repeat_timeout=const(250)
_bla_vari_auto_repeat_timeout=const(100)
_bla_leave_play_timeout=const(500)
_bla_shutdown_timeout=const(1000)
_bla_timeout=const(300000) # 5*60*1000: Idle more than  this time shutsdown
_bla_play_tick=const(10) 
_bla_init_speed=const(250) # Speed used when initializing motors
_bla_default_speed=const(1500)
bla_n_modes=const(2)
bla_n_ports=const(2)
bla_ports=(Port.A,Port.B)

# Global variables
hub=None # the Hub
rem=None # the Remote

defined_BLAs=[] # List of defined BLAs

# Devive types
_bla_SENSOR_MOTOR=const(1)
_bla_SIMPLE_MOTOR=const(2)
#_bla_LIGHT=const(3) # not supported yet
######################################################################
class BLABase:
    bla_list=()

    @staticmethod
    def load_config(name_p):
        _bla_saved_param_prefix='Rb'
        _bla_cfg_empty='.'
        _bla_cfg_codes='ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/' # Use Base64
        #               0123456789012345678901234567890123456789012345678901234567890123
        #                         1         2         3         4         5         6
        global defined_BLAs

        name=name_p[2:]

        for i in range(bla_n_modes):
            if name[i] == _bla_cfg_empty:
                break
            bla=_bla_cfg_codes.index(name[i])
            mode=(_bla_cfg_codes.index(name[4+i//2])>>i%2*3)&7
            ports=[_bla_cfg_codes.index(name[p])&3 if name[p]!=_bla_cfg_empty and (_bla_cfg_codes.index(name[p])>>4)==i else 0
                   for p in range(6,6+bla_n_ports)]
            # This test should be here... it will happen if you get a remote ony for ports C and D...but uses too much memory
            # if sum(ports) == 0: # Mode defined for ports not existing in this hub
            #     raise ValueError('No ports defined')
            defined_BLAs.append(BLABase.bla_list[int(bla/2)][0](bla, None, ports, mode, True if bla%2 == 1 else False, *(BLABase.bla_list[int(bla/2)][1])))
            
    def __init__(self, bla_p, ports_p, cfg_ports_p, mode_p, is_reverse_p):
        self.bla=bla_p
        # self.cfg_ports = self.get_cfg_ports(ports_p) if cfg_ports_p == None else cfg_ports_p
        self.cfg_ports = cfg_ports_p
        self.mode=mode_p
        self.devices=[]
        self.current_pos=0
        if is_reverse_p:
            self.mdir = (Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE) if self.mode <4 else (Direction.COUNTERCLOCKWISE, Direction.CLOCKWISE)
        else:
            self.mdir = (Direction.CLOCKWISE, Direction.CLOCKWISE) if self.mode <4 else (Direction.COUNTERCLOCKWISE, Direction.COUNTERCLOCKWISE)
            
    def dev_init(self):
        self.current_pos=0

    def plus_on(self, ts_p = None):
        return self.plus_or_minus_on(1, self.plus_on, ts_p)

    def minus_on(self, ts_p = None):
        return self.plus_or_minus_on(-1, self.minus_on, ts_p)

    def minus_off(self, ts_p = None):
        return self.plus_off(ts_p)
        
    def dev_stop(self, force_p=False):
        self.current_pos=0
        for d in self.devices:
            # d.stop() # This makes a slow stop
            d.dc(0) # This stps immediately... dont know why...
            
    def blink_on_limit(self, ts_p = None):
        rem.light.on(bla_error_color)
        rem.light.on(bla_play_color)
        return self.blink_on_limit
        
    def blink_on_zero(self, ts_p = None):
        rem.light.on(Color.MAGENTA)
        rem.light.on(bla_play_color)

######################################################################
# Maximum difference between measured speed and speed to set
_bla_vari_max_diff=const(220)
# BLA Steps types
_bla_STP_SIMPLE = const(1)
_bla_STP_SPEED  = const(2)
_bla_STP_VARI   = const(3)
_bla_STP_TIME   = const(4)

class BLAStepsMotor(BLABase):
    # mtype:
    #  _bla_STP_SIMPLE: simples motor, sets motor power in one or several steps
    #  _bla_STP_SPEED:  sets motor speed in one or several steps
    #  _bla_STP_VARI:   sets motor speed in small steps, with faster autorepeat
    # When is_speed_p, only works with motors with sensors
    # When is_reverse_p is true, with two or more devices the rotation of each
    # device is the reverse of the previous one.
    # If  is_one_step_p is true, _bla_STP_SIMPLE _bla_STP_SPEED will only have on/off mode.
    # Otherwise they might have several atand True one_step/pressed needed
    # If  is_one_step_p is true in _bla_STP_VARI, speed will increase until maximum but, button
    # must be pressed. When button is released motor goes to zero.
    #
    # Mode _bla_STP_SPEED assumes maximum speeds for each type of motor.
    # There are lots of motors, each one with its own characteristics, which makes it a bit difficult
    # to have a general purpose   # thing... 
    # I got no-load speeds in https://www.philohome.com/motors/motorcomp.htm, multiplied by 0.9
    # and got a list of maximum speeds per device. If the device does not match... it uses 1000deg/s (why not?).
    #
    # 1  Powered Up Medium Motor
    # 2  Powered Up Train Motor
    # 38 BOOST Interactive Motor
    # 46 Technic Control+ Large Motor
    # 47 Technic Control+ XL Motor
    # 48 SPIKE Prime Medium Motor
    # 49 SPIKE Prime Large Motor
    # 75 Technic Medium Angular Motor, gray
    # 76 Technic Large Angular motor, gray
    # 
    
    def __init__(self, bla_p, ports_p, cfg_ports_p, mode_p, is_reverse_p, mtype_p, is_one_step_p):
        self.mtype = mtype_p
        super().__init__(bla_p, ports_p, cfg_ports_p, mode_p, is_reverse_p)

        # For Vari
        devs_max_speed = { 38:1377, 46:1700, 47:1780, 48:1230, 49:1150, 75:1230, 76:1150 }
        dev_def_max_speed=1000

        ndev=sum([1 if i>0 else 0 for i in self.cfg_ports])
        
        self.is_one_step = is_one_step_p

        if self.mtype in (_bla_STP_SIMPLE, _bla_STP_SPEED):
            bla_pwrs=((0,14,29,43,57,71,86,100),
                      (0,20,40,60,80,100),
                      (0,33,67,100),
                      (0,11,22,33,44,56,67,78,89,100))
            self.powers = (0, (100, 80, 60, 40)[self.mode%4]) if is_one_step_p else bla_pwrs[self.mode%4]
            self.powers_max=len(self.powers)-1
        if self.mtype == _bla_STP_SPEED:
            self.devs_speed=[]
        if self.mtype in (_bla_STP_SPEED, _bla_STP_VARI):
            self.step=(50, 10, 80, 120)[self.mode%4]
        if self.mtype == _bla_STP_TIME:
            self.step=(300,500,1000,2000)[self.mode%4]
            self.powers=(0, is_one_step_p)
            self.init_time=None
            
        for i,p in enumerate(self.cfg_ports):
            if p == 0:
                continue
            if p == _bla_SENSOR_MOTOR: 
                d=Motor(bla_ports[i], self.mdir[len(self.devices)%2])
                if self.mtype == _bla_STP_SPEED:
                    s=devs_max_speed.get(PUPDevice(bla_ports[i]).info()['id'], dev_def_max_speed)
                    self.devs_speed.append(s)
                    d.control.limits(speed=s, acceleration=2000)
                if self.mtype == _bla_STP_VARI:
                    d.control.limits(speed=3000,acceleration=10000)
            if p == _bla_SIMPLE_MOTOR:
                d=DCMotor(bla_ports[i], self.mdir[len(self.devices)%2])
            self.devices.append(d)


    def _set_power(self):
        sig=1 if self.current_pos>=0 else -1
        for i,d in enumerate(self.devices):
            if self.mtype == _bla_STP_SPEED:
                d.run(sig*self.powers[abs(self.current_pos)]*self.devs_speed[i]/100)
            else:
                d.dc(sig*self.powers[abs(self.current_pos)])
            
    def plus_or_minus_on(self, dir, ar, ts_p):
        if self.mtype in (_bla_STP_SIMPLE, _bla_STP_SPEED):
            if self.is_one_step:
                self.current_pos = dir
                self._set_power()
                return
            if self.current_pos == dir*self.powers_max:
                return self.blink_on_limit
            self.current_pos = self.current_pos+dir
            self._set_power()
            if self.current_pos == 0:
                self.blink_on_zero()
            return ar

        if self.mtype == _bla_STP_TIME:
            if self.init_time is not None or self.current_pos == dir:
                self.blink_on_limit()
                return
            self.init_time = ts_p
            self.current_pos=dir
            self._set_power()
            return (ar, _bla_vari_auto_repeat_timeout, True)

        # mtype == _bla_STP_VARI:
        if abs(self.current_pos + dir*self.step) > abs(self.current_pos):
            doit=False
            for d in self.devices:
                if abs(d.speed()-self.current_pos) < _bla_vari_max_diff:
                    doit=True
        else:
            doit=True # No restrictions to reduce speed

        if doit:
            self.current_pos += (dir*self.step)
            for d in self.devices:
                d.run(self.current_pos)
            if self.current_pos == 0:
                self.blink_on_zero()
            return (ar, _bla_vari_auto_repeat_timeout)
        return self.blink_on_limit
                
    def time_off(self, ts_p):
        if self.init_time is None:
            return
        if ts_p-self.init_time >= self.step:
            for d in self.devices:
                d.dc(0)
            self.init_time=None
            return
        return (self.time_off, _bla_vari_auto_repeat_timeout, True)

    def plus_on(self, ts_p = None):
        return self.plus_or_minus_on(1, self.time_off if self.mtype == _bla_STP_TIME else super().plus_on, ts_p)

    def minus_on(self, ts_p = None):
        return self.plus_or_minus_on(-1, self.time_off if self.mtype == _bla_STP_TIME else super().minus_on, ts_p)

    def dev_stop(self, force_p=False):
        if self.mtype == _bla_STP_TIME and not force_p:
            return
        super().dev_stop(force_p)
                
    def plus_off(self, ts_p = None):
        if self.mtype == _bla_STP_TIME:
            return (self.time_off, _bla_vari_auto_repeat_timeout, True)
        if self.is_one_step:
            self.dev_stop()

######################################################################
# BLASteering types
_bla_STE_STEPPER_ZERO=       const(0) # initialize on motor absolute zero and go on from there
_bla_STE_STEPPER_FIND_INIT=  const(1) # Finds ititial position by stalling and go on from there
_bla_STE_STEPPER_FIND_LIMITS=const(2) # Finds ititial position, find final position and use step size to get number of speeds
_bla_STE_STEERING_SERVO=     const(3) # servo mode, steps configurable
_bla_STE_STEERING_LIMITS=    const(4) # find limits and steer, steps configurable

class BLASteering(BLABase):
    # round_angle=True

    #--------------------------------------------------
    # is_reverse_p - with two or more devices the rotation of each device is the reverse of the previous one
    def __init__(self, bla_p, ports_p, cfg_ports_p, mode_p, is_reverse_p, type_p, step_p, n_steps_p):
        # For the record: In a gearbox with N speeds there are N-1 steps

        super().__init__(bla_p, ports_p, cfg_ports_p, mode_p, is_reverse_p)
        ndev=sum([1 if i>0 else 0 for i in self.cfg_ports])

        self.bla_type=type_p
        self.then = (Stop.HOLD, Stop.COAST)[self.mode%2]
        self.step=step_p # step size in degrees
        self.speed = _bla_default_speed
        self.n_steps=n_steps_p
        if self.bla_type == _bla_STE_STEPPER_FIND_LIMITS:
            # if ndev>1:
            #     raise ValueError('Find Limits does not work with more than one device!')
            if  self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
                self.step*=2
        if self.bla_type in(_bla_STE_STEERING_SERVO, _bla_STE_STEERING_LIMITS)  and self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
            self.n_steps+=2
        if self.bla_type in (_bla_STE_STEPPER_ZERO, _bla_STE_STEPPER_FIND_INIT) and self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
            self.n_steps+=1

        for i,p in enumerate(self.cfg_ports):
            if p == 0:
                continue
            d=Motor(bla_ports[i], self.mdir[len(self.devices)%2])
            d.control.limits(acceleration=5000)
            self.devices.append(d)
            
    #--------------------------------------------------
    def dev_init(self):
        self.angles=[]
        self.current_pos=0
        for di, d in enumerate(self.devices):
            self.angles.append([])
            if self.bla_type ==_bla_STE_STEPPER_ZERO:
                d.reset_angle() # Set initial angle to absolute 0
                if d.angle()<0: # Go to absolute zero always in backwards direction
                    d.run_angle(-_bla_init_speed, 180, self.then, True)
                    d.reset_angle()
                d.run_target(_bla_init_speed, 0, self.then, True)
                for i in range(self.n_steps+1):
                    self.angles[di].append(i*self.step)

            if self.bla_type == _bla_STE_STEPPER_FIND_INIT:
                d.reset_angle() 
                # Find initial position by stalling
                lower_end=d.run_until_stalled(-_bla_init_speed, self.then)
                # if self.round_angle:
                rnd = trunc(lower_end/self.step)*self.step
                lower_end = rnd + self.step if rnd < lower_end else rnd
                d.run_target(_bla_init_speed, lower_end, self.then, True)
                d.reset_angle(0)
                for i in range(self.n_steps+1):
                    self.angles[di].append(i*self.step)
 
            if self.bla_type == _bla_STE_STEPPER_FIND_LIMITS:
                d.reset_angle() 
                # Find limits by stalling and guess number os speeds
                lower_end=d.run_until_stalled(-_bla_init_speed, self.then)
                # if self.round_angle:
                rnd = trunc(lower_end/self.step)*self.step
                lower_end = rnd + self.step if rnd < lower_end else rnd
                d.run_target(_bla_init_speed, lower_end, self.then, True)
                d.reset_angle(0)
                #d.run_target(_bla_init_speed, 0, self.then, True)
                upper_end=d.run_until_stalled(_bla_init_speed, self.then)            
                # if self.round_angle:
                rnd = trunc(upper_end/self.step)*self.step
                upper_end = rnd - self.step if rnd > upper_end else rnd
                d.run_target(_bla_init_speed, upper_end, self.then, True)
                self.angles[di].append(0)
                self.n_steps=0
                while self.angles[di][self.n_steps]+self.step <=upper_end:
                    self.angles[di].append(self.angles[di][self.n_steps]+self.step)
                    self.n_steps+=1
                for a in reversed(self.angles[di]):
                    d.run_target(self.speed, a, self.then, True)
                    
            if self.bla_type ==_bla_STE_STEERING_SERVO:
                # Set initial angle to absolute 0                
                d.reset_angle()
                d.run_target(_bla_init_speed, 0, self.then, True)
                for i in range(self.n_steps+1):
                    self.angles[di].append(i*self.step/self.n_steps)

            if self.bla_type == _bla_STE_STEERING_LIMITS:
                # Find limits positions. Will find angles by dividing the range of positions in n_steps
                lower_end=d.run_until_stalled(-_bla_init_speed, self.then)
                upper_end=d.run_until_stalled(_bla_init_speed, self.then)            
                sa=(upper_end - lower_end) / 2.0
                d.reset_angle(sa)
                d.run_target(_bla_init_speed, 0, self.then, True) # Make sa angle the 0 angle
                for i in range(self.n_steps+1):
                    self.angles[di].append(i*sa/self.n_steps)

    def _set_step(self, wait_p=False):
        sig=1 if self.current_pos>=0 else -1
        for di, d in enumerate(self.devices):
            d.run_target(self.speed, sig*self.angles[di][abs(self.current_pos)], self.then, wait_p)
                    
    def plus_or_minus_on(self, dir, ar, ts_p):
        lmt = self.n_steps if dir == 1 else (
             -self.n_steps if self.bla_type in (_bla_STE_STEERING_SERVO, _bla_STE_STEERING_LIMITS) else 0)
        if self.current_pos == lmt:
            return self.blink_on_limit if self.n_steps>1 else None
        self.current_pos = self.current_pos+dir
        self._set_step()
        if self.current_pos == 0:
            self.blink_on_zero()
        return ar
        
    def plus_off(self, ts_p = None):
        if self.bla_type in (_bla_STE_STEERING_SERVO,_bla_STE_STEERING_LIMITS) and self.n_steps == 1:
            self.current_pos = 0
            for d in self.devices:
                d.run_target(self.speed, 0, self.then, False)

    def dev_stop(self, force_p=False):
        self.current_pos=0
        for di, d in enumerate(self.devices):
            d.run_target(self.speed, self.angles[di][0], Stop.COAST if force_p else self.then, False)

######################################################################
# Existing BLAs definition tupple. Maximum 64, each one in its position in the list.
# Each element is a tuple where first position is a BLA object and second position a tuple of extra arguments
# to the BLA Object costructor 
#
BLABase.bla_list=(
    (BLAStepsMotor, (_bla_STP_SIMPLE,              True)  ), # BLASimpleMotor          CYAN    A,B
    (BLAStepsMotor, (_bla_STP_SIMPLE,              False) ), # BLAStepsMotor           YELLOW  C,D
    (BLAStepsMotor, (_bla_STP_SPEED,               True)  ), # BLASpeedMotor           GRAY    E,F
    (BLAStepsMotor, (_bla_STP_SPEED,               False) ), # BLAStepSpeedMotor       ORANGE  G,H
    (BLAStepsMotor, (_bla_STP_VARI,                False) ), # BLAVariMotor            BLUE    I,J
    (BLAStepsMotor, (_bla_STP_VARI,                True)  ), # BLAVariPressMotor       MAGENTA K,L
    (BLASteering,   (_bla_STE_STEERING_LIMITS,     90, 1) ), # BlaSteering_1_3         GREEN   M,N
    (BLASteering,   (_bla_STE_STEERING_LIMITS,     90, 5) ), # BlaSteering_5_7         WHITE   O,P
    (BLASteering,   (_bla_STE_STEPPER_FIND_LIMITS, 90, 0) ), # BLAStepperLimits90_180  CYAN    Q,R
    (BLASteering,   (_bla_STE_STEERING_SERVO,      90, 1) ), # BLAServo90_1_3          YELLOW  S,T
    (BLASteering,   (_bla_STE_STEERING_SERVO,      90, 5) ), # BLAServo90_5_7          GRAY    U,V
    (BLASteering,   (_bla_STE_STEPPER_ZERO,        90, 1) ), # BLAStepperZero90_2_3    ORANGE  W,X
    (BLASteering,   (_bla_STE_STEPPER_ZERO,        90, 3) ), # BLAStepperZero90_4_5    BLUE    Y,Z
    (BLASteering,   (_bla_STE_STEPPER_ZERO,        90, 5) ), # BLAStepperZero90_6_7    MAGENTA a,b
    (BLASteering,   (_bla_STE_STEPPER_ZERO,        90, 7) ), # BLAStepperZero90_8_9    GREEN   c,d
    (BLASteering,   (_bla_STE_STEPPER_FIND_INIT,   90, 1) ), # BLAStepperInit90_2_3    WHITE   e,f
    (BLASteering,   (_bla_STE_STEPPER_FIND_INIT,   90, 3) ), # BLAStepperInit90_4_5    CYAN    g,h
    (BLASteering,   (_bla_STE_STEPPER_FIND_INIT,   90, 5) ), # BLAStepperInit90_6_7    YELLOW  i,j
    (BLASteering,   (_bla_STE_STEPPER_FIND_INIT,   90, 7) ), # BLAStepperInit90_8_9    GRAY    k,l
    (BLASteering,   (_bla_STE_STEPPER_FIND_INIT,   60, 2) ), # BLAStepperInit60_3_4    ORANGE  m,n
    (BLASteering,   (_bla_STE_STEPPER_FIND_INIT,   60, 4) ), # BLAStepperInit60_5_6    BLUE    o,p
    (BLAStepsMotor, (_bla_STP_TIME,                100)   ), # BLAUpDown100            MAGENTA q,r
    (BLAStepsMotor, (_bla_STP_TIME,                80)    ), # BLAUpDown80             GREEN   s,t
    (BLAStepsMotor, (_bla_STP_TIME,                60)    ), # BLAUpDown60             WHITE   u,v
)
    
######################################################################
# Autorepeat for selected modes.
# The methods that implement a given button function, like, for instance, PLUS, can return a method pointer 
# for the autorepeat action. This method will be called if the button is kept pressed for some time.
# If the returned method returns a funcion pointer, the auto repeat is rearmed. If it returns null,
# autorepeat stops.
# Each position in the list represents a different button.
auto_repeat = [None, None, None, None, None, None]#, None, None]

def save_autorepeat(btn_pos_p, ara_p = None, time_p = None, btn_p = None):
    # Can return a tuple where second element is timemout
    if type(ara_p) is tuple:
        (ara, tmt) = ara_p[:2]
        if len(ara_p) == 3:
            btn_p = None            
    else:
        (ara, tmt) = (ara_p, _bla_auto_repeat_timeout)
    auto_repeat[btn_pos_p]=(ara, time_p, btn_p, tmt) if ara_p != None else None

######################################################################
def bla_play():

    global defined_BLAs,rem,hub

    # A flag (for each button press) indicating the corresponding on action was done.
    # Off actions only happen if a corresponding on action existed.
    # This is importante because of shift button actions, on City that does not exist
    # btn_on_action=[False, False, False, False, False, False, False, False]

    hub.light.on(bla_dev_init_color) # Put hub light in red while initializing
    for d in defined_BLAs:
        d.dev_init()
    rem.light.on(bla_play_color)
    hub.light.on(bla_play_color)
    prev_pressed=()
    sw=StopWatch()   # General purpose timer
    idle_init=0
    n_def_BLAs=len(defined_BLAs)
    hub_button_pressed=None

    while True:
        wait(_bla_play_tick)
        this_tick= sw.time()

        if Button.CENTER in hub.button.pressed():
            if hub_button_pressed == None:
                hub_button_pressed=this_tick
            if this_tick-hub_button_pressed>_bla_shutdown_timeout:
                hub.system.shutdown()
        else:
            hub_button_pressed=None
        
        pressed = rem.buttons.pressed()
        lp=len(pressed)
        if lp == 0 and this_tick-idle_init>_bla_timeout:
            hub.light.blink(bla_error_color, [200, 100])
            wait(800)
            hub.system.shutdown()
        if lp != 0:
            idle_init=this_tick

        # BLA Command 1
        if Button.RIGHT_PLUS not in pressed and Button.RIGHT_PLUS in prev_pressed:
            ara=defined_BLAs[0].plus_off(this_tick)
            save_autorepeat(_btn_RP, ara, this_tick, Button.RIGHT_PLUS)
                
        if Button.RIGHT_MINUS not in pressed and Button.RIGHT_MINUS in prev_pressed:
            ara=defined_BLAs[0].minus_off(this_tick)
            save_autorepeat(_btn_RM, ara, this_tick, Button.RIGHT_MINUS)

        # Not need if because it is always less than 4
        # if n_def_BLAs < 4 or (Button.RIGHT not in pressed and Button.LEFT not in pressed):
        if Button.RIGHT_PLUS in pressed and Button.RIGHT_PLUS not in prev_pressed:
            ara=defined_BLAs[0].plus_on(this_tick)
            save_autorepeat(_btn_RP, ara, this_tick, Button.RIGHT_PLUS)
            
        if Button.RIGHT_MINUS in pressed and Button.RIGHT_MINUS not in prev_pressed:
            ara=defined_BLAs[0].minus_on(this_tick)
            save_autorepeat(_btn_RM, ara, this_tick, Button.RIGHT_MINUS)


        # BLA Command 2
        if n_def_BLAs == 2:
            if Button.LEFT_PLUS not in pressed and Button.LEFT_PLUS in prev_pressed:
                ara=defined_BLAs[1].plus_off(this_tick)
                save_autorepeat(_btn_LP, ara, this_tick, Button.LEFT_PLUS)

            if Button.LEFT_MINUS not in pressed and Button.LEFT_MINUS in prev_pressed:
                ara=defined_BLAs[1].minus_off(this_tick)
                save_autorepeat(_btn_LM, ara, this_tick, Button.LEFT_MINUS)

            if Button.LEFT_PLUS in pressed and Button.LEFT_PLUS not in prev_pressed:
                ara=defined_BLAs[1].plus_on(this_tick)
                save_autorepeat(_btn_LP, ara, this_tick, Button.LEFT_PLUS)

            if Button.LEFT_MINUS in pressed and Button.LEFT_MINUS not in prev_pressed:
                ara=defined_BLAs[1].minus_on(this_tick)
                save_autorepeat(_btn_LM, ara, this_tick, Button.LEFT_MINUS)


        # 2 BLA defined, can use red buttons to stop corresponding device...
        if lp==1 and Button.RIGHT in pressed:
            defined_BLAs[0].dev_stop()
        if n_def_BLAs == 2 and lp==1 and Button.LEFT in pressed:
            defined_BLAs[1].dev_stop()
                           
        for i,a in enumerate(auto_repeat):
            if a == None:
                continue
            if (a[2] is None or (a[2] in pressed and a[2] in prev_pressed)) and this_tick-a[1] > a[3]:
                nara=a[0](this_tick)
                save_autorepeat(i, nara, this_tick, a[2])

        prev_pressed=pressed

        
hub=ThisHub()
hub.system.set_stop_button(None)
hub.light.blink(Color.WHITE, (200,200, 100, 400))
rem = Remote()
hub.light.on(bla_dev_init_color)

try:
    BLABase.load_config(rem.name())
    bla_play()
except Exception as e:
    hub.light.blink(Color.ORANGE, (100,50))
    wait(3000)

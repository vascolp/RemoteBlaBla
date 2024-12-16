#
# Name: Remote Bla Bla for TechnicHub
#
# Description:
# Remote Bla Bla is a Pybricks (https://pybricks.com) program for a LEGO
# PoweredUp TechnicHub and a LEGO PoweredUp Remote Control where you
# first define how each device answers to remote control button actions
# (config mode) and then you play with your MOC/Set using the defined
# configuration (play mode).
#
# The configuration is saved so that you can turn off the hub, change
# batteries if needed, go back to your hub and everything is there. Just
# play. If the configuration is not what you want, you can go back to
# config mode and reconfigure the behavior.
#
# This version should also work on an InventorHub or PrimeHub, (4 modes but 6 ports)
# but I haven't one to test it.
#
# Keep in mind that this is a computer with an input device of 7 buttons
# and an output device of just two LEDs. This is a Star Trek like
# interface, a real YDSWIG interface: you don't see what you get.
# However, once you learn it, you will notice that it is simple.
# No screens attached!
#
# There is a users manual in PDF avaiable.
# https://github.com/vascolp/RemoteBlaBla
#
# Version: 1.01
#
# Author VascoLP: vascolp.lego@gmail.com
#
# Date: April 2022
#
# Installing:
# Install Pybricks firmware with Remote Bla Bla (this program) included, on a TechnicHub.
# It should also work on an InventorHub or PrimeHub, but I haven't got one to test it.
# You should use pybricks firmware version v3.1.0 on 2021-12-16 or later.
#
# Note: all this code would be much nicer with a class for each BLA mode. But classes
# use much more memory than if/else statments... 
#

from micropython import const
from pybricks.hubs import ThisHub
from pybricks.tools import StopWatch
from pybricks.iodevices import PUPDevice
from pybricks.pupdevices import DCMotor, Motor, Remote
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
from uerrno import ENODEV
#from math import trunc

_btn_RP= const(0) # Button.RIGHT_PLUS
_btn_LP= const(1) # Button.LEFT_PLUS
_btn_RR=         const(2)  # Button.RIGHT (+) in three command play mode
_btn_SRP=const(2) # Shift-Button.RIGHT_PLUS  in four command play mode/config mode
_btn_SLP=const(3) # Shift-Button.LEFT_PLUS   in four command play mode/config mode
_btn_RM= const(4) # Button.RIGHT_MINUS
_btn_LM= const(5) # Button.LEFT_MINUS
_btn_LR=         const(6)  # Button.LEFT  (-) in three command play mode
_btn_SRM=const(6) # Shift-Button.RIGHT_MINUS in four command play mode/config mode
_btn_SLM=const(7) # Shift-Button.LEFT_MINUS  in four command play mode/config mode

bla_config_color=Color.WHITE*0.8 # Color for config Mode
bla_config_reverse_remote_color=Color.MAGENTA # Color for config Mode on remote when defining a mode with reverse
_bla_config_tick=const(100) # Should be enough for config mode...
_bla_play_fast_zero_timeout=const(300) #double click for FastZero timeout 
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
bla_n_modes=4 # possibly redefined later
bla_n_ports=4 # possibly redefined later
bla_ports=()

_bla_version_color=Color.CYAN
_bla_version_num=5

# Global variables
hub=None # the Hub
rem=None # the Remote

defined_BLAs=[] # List of defined BLAs

# Parameter saving:
_bla_saved_param_prefix='Rb'
_bla_cfg_empty='.'
_bla_cfg_codes='ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/' # Use Base64
#               0123456789012345678901234567890123456789012345678901234567890123
#                         1         2         3         4         5         6

# Devive types
_bla_SENSOR_MOTOR=const(1)
_bla_SIMPLE_MOTOR=const(2)
#_bla_LIGHT=const(3) # not supported yet
######################################################################
class BLABase:
    bla_list=()

    @staticmethod
    def load_config(name_p):
        global defined_BLAs

        if len(name_p)!=14 or name_p[0:2]!=_bla_saved_param_prefix:
            wait(100) # for luck
            return False
        name=name_p[2:]
        try:
            wait(100) # for luck
            for i in range(bla_n_modes):
                if name[i] == _bla_cfg_empty:
                    break
                wait(100) # for luck
                bla=_bla_cfg_codes.index(name[i])
                mode=(_bla_cfg_codes.index(name[4+i//2])>>i%2*3)&7
                ports=[_bla_cfg_codes.index(name[p])&3 if name[p]!=_bla_cfg_empty and (_bla_cfg_codes.index(name[p])>>4)==i else 0
                       for p in range(6,6+bla_n_ports)]
                if sum(ports) == 0: # Mode defined for ports not existing in this hub
                    raise ValueError('No ports defined')
                defined_BLAs.append(BLABase.bla_list[int(bla/2)][0](bla, None, ports, mode, bla%2, *(BLABase.bla_list[int(bla/2)][1])))
            return  len(defined_BLAs) > 0
        except ValueError as ve:
            wait(100) # for luck
            return False
        except OSError as os:
            if os.args[0] == ENODEV:
                wait(100) # for luck
            return False        
       
    def get_cfg_ports(self, ports_p):
        cfg_ports=[0 for _ in range(bla_n_ports)]
        for p in ports_p:
            cfg_ports[bla_ports.index(p)]=_bla_SENSOR_MOTOR
        return cfg_ports
    
    @staticmethod
    def save_config(r):
        # Parameters saved in 12 chars 6bit coded, this way:
        # bbbbb1
        # bbbbb2
        # bbbbb3
        # bbbbb4
        # mm2mm1
        # mm4mm3
        # pp00p1 - pp is the bla that uses this port
        # pp00p2
        # pp00p3
        # pp00p4
        # pp00p5
        # pp00p6

        if len(defined_BLAs) == 0:
            return False
        name = _bla_saved_param_prefix
        modes=[0 for _ in range(4)]
        ports=[None for _ in range(6)]
        for m,b in enumerate(defined_BLAs):
            name += _bla_cfg_codes[b.bla]
            modes [m] = b.mode
            for i,p in enumerate(b.cfg_ports):
                if p:
                    ports[i] = (m<<4)|p
        name += _bla_cfg_empty*(4-len(defined_BLAs))
        name += _bla_cfg_codes[modes[1]<<3|modes[0]]
        name += _bla_cfg_codes[modes[3]<<3|modes[2]]
        for p in ports:
            name += _bla_cfg_empty if p == None else _bla_cfg_codes[p]
        name += _bla_cfg_empty*(6-len(ports))
        r.name(name)
        return True
        
    @classmethod
    def bla_list_len(cls):
        return len(cls.bla_list)
    
    @classmethod
    def make(cls, bla_p, ports_p, mode_p):
        return cls.bla_list[int(bla_p/2)][0](bla_p, ports_p, None, mode_p, bla_p%2, *(cls.bla_list[int(bla_p/2)][1]))

    def __init__(self, bla_p, ports_p, cfg_ports_p, mode_p, is_reverse_p):
        self.bla=bla_p
        self.cfg_ports = self.get_cfg_ports(ports_p) if cfg_ports_p == None else cfg_ports_p
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
_bla_vari_max_diff=const(220) # deg/s
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

    def get_cfg_ports(self, ports_p):
        if self.mtype in (_bla_STP_SPEED, _bla_STP_VARI):
            return super().get_cfg_ports(ports_p)
        
        cfg_ports=[0 for _ in range(bla_n_ports)]
        for i,p in enumerate(ports_p):
            try:
                td=PUPDevice(p)
                did=td.info()['id']
                if did in (1,2): # Powered Up Medium Motor, Powered Up Train Motor (sensorless motors)
                    cfg_ports[bla_ports.index(p)]=_bla_SIMPLE_MOTOR
                else: # Assume motor with sensor
                    cfg_ports[bla_ports.index(p)]=_bla_SENSOR_MOTOR
            except OSError as ex:
                raise ValueError('Invalid device type!') # Comunicate that the given device is not valid
        return cfg_ports
    
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
            try:
                if p == _bla_SENSOR_MOTOR: 
                    d=Motor(bla_ports[i], self.mdir[len(self.devices)%2])
                    if self.mtype == _bla_STP_SPEED:
                        s=devs_max_speed.get(PUPDevice(bla_ports[i]).info()['id'], dev_def_max_speed)
                        self.devs_speed.append(s)
                        d.control.limits(speed=s, acceleration=2000)
                    if self.mtype == _bla_STP_VARI:
                        d.control.limits(speed=3000,acceleration=10000)
                elif p == _bla_SIMPLE_MOTOR:
                    d=DCMotor(bla_ports[i], self.mdir[len(self.devices)%2])
            except OSError as ex:
                raise ValueError('Invalid device type!') # Comunicate that the given device is not valid            
            self.devices.append(d)
        self.plus_ts_on = None
        self.minus_ts_on = None

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
        if self.current_pos*dir > 0 and abs(self.current_pos + dir*self.step) > abs(self.current_pos):
            doit=False
            for d in self.devices:
                s=d.speed()
                if s*dir < 0 or abs(s-self.current_pos) < _bla_vari_max_diff:
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
            # return self.plus_or_minus_on(-self.current_pos, self.time_off, ts_p)
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
            return

        # Double click to stop
        if  self.current_pos < 0 and len(defined_BLAs)>2:
            if self.plus_ts_on is None:
                self.plus_ts_on=ts_p
            elif ts_p-self.plus_ts_on < _bla_play_fast_zero_timeout:
                self.dev_stop()
                rem.light.on(Color.MAGENTA)
                rem.light.on(bla_play_color)
                self.plus_ts_on=None                
            else:
                self.plus_ts_on=ts_p
            
    def minus_off(self, ts_p = None):
        if self.mtype == _bla_STP_TIME:
            return (self.time_off, _bla_vari_auto_repeat_timeout, True)
        if self.is_one_step:
            self.dev_stop()
            return

        # Double click to stop
        if  self.current_pos > 0 and len(defined_BLAs)>2:
            if self.minus_ts_on is None:
                self.minus_ts_on=ts_p
            elif ts_p-self.minus_ts_on < _bla_play_fast_zero_timeout:
                self.dev_stop()
                rem.light.on(Color.MAGENTA)
                rem.light.on(bla_play_color)
                self.minus_ts_on=None                
            else:
                self.minus_ts_on=ts_p

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
            if ndev>1:
                raise ValueError('Find Limits does not work with more than one device!')
            if  self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
                self.step*=2
        elif self.bla_type in(_bla_STE_STEERING_SERVO, _bla_STE_STEERING_LIMITS)  and self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
            self.n_steps+=2
        elif self.bla_type in (_bla_STE_STEPPER_ZERO, _bla_STE_STEPPER_FIND_INIT) and self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
            self.n_steps+=1

        for i,p in enumerate(self.cfg_ports):
            if p == 0:
                continue
            try:
                d=Motor(bla_ports[i], self.mdir[len(self.devices)%2])
                d.control.limits(acceleration=5000)
                self.devices.append(d)
            except OSError as ex:
                raise ValueError('Invalid device type!') # Comunicate that the given device is not valid
            
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

    def minus_off(self, ts_p = None):
        return self.plus_off(ts_p)

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
def set_bla_config_BLA_blink(m):
    # """ Sets the hub light for m config mode definition
    # """
    # Defined only up to 40 combinations...
    global hub

    m2=int(m/2)
    c= ( Color.CYAN, Color.YELLOW, Color.GRAY, Color.ORANGE, Color.BLUE, Color.MAGENTA, Color.GREEN, Color.WHITE )[m2%8]     
    b=(
        (5, 300, 1500, 5),
        (5, 200,  500, 5), 
        (5, 100,  300, 5),
        (5,  50,  150, 5)
        )[int((m2-8)/8)]
    
    if m%2:
        rem.light.on(bla_config_reverse_remote_color)
    else:
        rem.light.on(bla_config_color)
    if m2 < 8: # No blinking
        hub.light.on(c)
    else:
        hub.light.blink(c,b)
    
######################################################################
def hub_mini_error():
    # """ Flashes an error sequence for small errors
    # """
    global hub
    
    hub.light.blink(bla_error_color, [200, 100])
    wait(800)
    hub.light.off() #Needed! dont ask why...

######################################################################
def check_shutdown(this_tick, hub_button_pressed):
    if Button.CENTER in hub.button.pressed():
        if hub_button_pressed == None:
            return this_tick
        if this_tick-hub_button_pressed>_bla_shutdown_timeout:
            hub.system.shutdown()
        return hub_button_pressed
    return None

######################################################################
def get_new_ports(used_ports_p, stop_if_more_than_one_p=False):
    new_ports=[]
    for i,p in enumerate(bla_ports):
        if p in used_ports_p:
            continue
        try:
            _=PUPDevice(p)
            new_ports.append(p)
            if stop_if_more_than_one_p and len(new_ports)>1:
                return new_ports
        except OSError as ex:
            if ex.args[0] != ENODEV:    #No device found on this port.
                raise
    return new_ports

######################################################################
def bla_config():
    # """ This function does the process of port configuration.
    # """

    global defined_BLAs,rem,hub

    rem.light.on(bla_config_color)
    defined_BLAs=[]
    cur_BLA=0 # Current mode
    set_bla_config_BLA_blink(cur_BLA)

    used_ports=[]                
    prev_pressed=rem.buttons.pressed()
    sw=StopWatch()   # General purpose timer
    idle_init=0
    bla_rrb=False
    bla_lrb=False
    hub_button_pressed=None
    did_something=False
    while True:
        wait(_bla_config_tick) # in the beginning so that we can use nice continue statments
        
        if sw.time()-idle_init>_bla_timeout: # Idle timeout
            hub_mini_error()
            hub.system.shutdown()

        pressed = rem.buttons.pressed()
        hub_pressed = hub.button.pressed()
        lp=len(pressed)
        hlp=len(hub_pressed)

        if lp == 0 and hlp==0:
            did_something = False

        if lp == 0:
            hub_button_pressed=check_shutdown(sw.time(),hub_button_pressed)

        if did_something:
            continue

        if (Button.RIGHT in pressed or Button.LEFT in pressed) and Button.CENTER in hub_pressed:
            # Shift Hub button deletes last BLA created (go to the hub and delete last created...)
            if len(defined_BLAs) == 0: # nothing to delete
                hub_mini_error()
            else:
                b=defined_BLAs[len(defined_BLAs)-1]
                for i,p in enumerate(b.cfg_ports):
                    if p != 0:
                        used_ports.remove(bla_ports[i])
                defined_BLAs.remove(b)
                hub.light.blink(bla_send_color, [50, 50]) # Show user success message
                wait(300)
                hub.light.on(bla_send_color)
                wait(200)
            set_bla_config_BLA_blink(cur_BLA)
            bla_rrb = False
            bla_lrb = False
            did_something=True
            continue

        if prev_pressed == pressed:
            continue
        idle_init=sw.time()

        prev_pressed=pressed
        # Only zero or one button pressed, or two buttons with one of them the red ones (acting as shift) or special 3 button
        if not ((lp==0) or (lp==1) or
                (lp==2 and Button.RIGHT in     pressed and Button.LEFT not in pressed) or
                (lp==2 and Button.RIGHT not in pressed and Button.LEFT in     pressed) or
                (lp==3 and Button.RIGHT     in pressed and Button.LEFT in     pressed and Button.CENTER in pressed)):
            continue

        if (lp == 1 and Button.RIGHT in pressed):
            bla_rrb = True
            continue
        elif (lp == 1 and Button.LEFT in pressed):
            bla_lrb = True
            continue
        elif lp==0 and (bla_rrb or bla_lrb):
            if cur_BLA%2: # odd position means reverse mode
                cur_BLA -= 1
            if bla_rrb:
                cur_BLA= 0 if cur_BLA==(2*BLABase.bla_list_len())-2 else cur_BLA+2
            else:
                cur_BLA= (2*BLABase.bla_list_len())-2 if cur_BLA==0 else cur_BLA-2

            set_bla_config_BLA_blink(cur_BLA)
            bla_rrb = False
            bla_lrb = False
            did_something = True
            continue
        bla_rrb = False
        bla_lrb = False

        if Button.CENTER in pressed:
            if lp==1:
                # Center button leaves config, if at least one BLA is defined
                # It also saves configuration
                hub.light.blink(Color.GREEN if len(defined_BLAs)>0 else Color.RED, [100, 50])
                wait(500)
                if BLABase.save_config(rem):
                    break
                else:
                    set_bla_config_BLA_blink(cur_BLA)
                    continue
                did_something = True
                
            if lp==2: # Shift-Center changes to Reverse BLA mode (if usefull) or back
                if cur_BLA%2 == 0:
                    np=get_new_ports(used_ports, True)
                    if len(np)>1:
                        cur_BLA+=1
                    else:
                        hub_mini_error()
                else:
                    cur_BLA -= 1
                set_bla_config_BLA_blink(cur_BLA)
                did_something = True
                continue
            
            elif lp==3: # Right and Left and Center all pressed at the same time show version
                hub.light.off()
                wait(1000)
                for i in range(_bla_version_num):
                    hub.light.on(_bla_version_color)
                    wait(200)
                    hub.light.off()
                    wait(200)
                wait(1000)                    
                set_bla_config_BLA_blink(cur_BLA)
                did_something = True
                continue

        # Tries to get a mode from buttons pressed
        if   Button.RIGHT_PLUS in pressed:
            mode= _btn_SRP if Button.RIGHT in pressed or Button.LEFT in pressed else _btn_RP
        elif Button.LEFT_PLUS in pressed:
            mode= _btn_SLP if Button.RIGHT in pressed or Button.LEFT in pressed else _btn_LP
        elif Button.RIGHT_MINUS in pressed:
            mode= _btn_SRM if Button.RIGHT in pressed or Button.LEFT in pressed else _btn_RM
        elif Button.LEFT_MINUS in pressed:
            mode= _btn_SLM if Button.RIGHT in pressed or Button.LEFT in pressed else _btn_LM
        else:
            continue

        if len(defined_BLAs)==bla_n_modes : # warn (because on PrimeHub there are more ports than modes)
            hub_mini_error()
            set_bla_config_BLA_blink(cur_BLA)
            continue

        hub.light.on(Color.RED*0.4) # Put hub light in red while detecting devices, because device detection fails sometimes
                                    # specially with DCMotors and low battery
        new_ports=get_new_ports(used_ports)
        set_bla_config_BLA_blink(cur_BLA)

        if len(new_ports)==0:
            hub_mini_error()
            set_bla_config_BLA_blink(cur_BLA)
            continue

        try: # Tries to create the mode (might fail if mode does not handle connected devices)
            m=BLABase.make(cur_BLA, new_ports, mode)
            defined_BLAs.append(m)
            for p in new_ports:
                used_ports.append(p)
            hub.light.blink(bla_send_color, [50, 50]) # Show user success message
            wait(700)
            hub.light.on(bla_send_color)
            wait(200)
        except ValueError as ve:
            hub_mini_error() # Show user that the device does not allow the selected mode
        if cur_BLA%2: # If just created a revrse, go back to non reverse
            cur_BLA -= 1
        set_bla_config_BLA_blink(cur_BLA)
        did_something = True

######################################################################
# Autorepeat for selected modes.
# The methods that implement a given button function, like, for instance, PLUS, can return a method pointer 
# for the autorepeat action. This method will be called if the button is kept pressed for some time.
# If the returned method returns a funcion pointer, the auto repeat is rearmed. If it returns null,
# autorepeat stops.
# Each position in the list represents a different button.
auto_repeat = [None, None, None, None, None, None, None, None]

#def save_autorepeat(btn_pos_p, ara_p = None, time_p = None, btn_p = None):
def save_autorepeat(btn_pos_p, ara_p, time_p, btn_p):
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

    global defined_BLAs,rem,hub, auto_repeat

    # A flag (for each button press) indicating the corresponding on action was done.
    # Off actions only happen if a corresponding on action existed.
    btn_on_action=[False, False, False, False, False, False, False, False]

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
        
        hub_button_pressed=check_shutdown(this_tick, hub_button_pressed)

        pressed = rem.buttons.pressed()
        lp=len(pressed)
        if lp == 0 and this_tick-idle_init>_bla_timeout:
            hub_mini_error()
            hub.system.shutdown()
        if lp != 0:
            idle_init=this_tick
        
        if lp==1 and Button.CENTER in pressed and Button.CENTER not in prev_pressed:
            button_center_pressed=this_tick

        if lp==1 and Button.CENTER in pressed and Button.CENTER in prev_pressed and this_tick-button_center_pressed>_bla_leave_play_timeout:
            rem.light.on(Color.ORANGE*0.8) # Immediate feedback because 1.2s in remote green button turns off remote...
            hub.light.on(Color.ORANGE*0.8)

        if lp==0 and Button.CENTER not in pressed and Button.CENTER in prev_pressed and this_tick-button_center_pressed>_bla_leave_play_timeout:
            prev_pressed=pressed
            for d in defined_BLAs:
                d.dev_stop(True)
            auto_repeat = [None, None, None, None, None, None, None, None] # Needed because of _bla_STP_TIME mode...
            break

        # BLA Command 1
        if Button.RIGHT_PLUS not in pressed and Button.RIGHT_PLUS in prev_pressed and btn_on_action[_btn_RP]:
            ara=defined_BLAs[0].plus_off(this_tick)
            save_autorepeat(_btn_RP, ara, this_tick, Button.RIGHT_PLUS)
            btn_on_action[_btn_RP]=False
                
        if Button.RIGHT_MINUS not in pressed and Button.RIGHT_MINUS in prev_pressed and btn_on_action[_btn_RM]:
            ara=defined_BLAs[0].minus_off(this_tick)
            save_autorepeat(_btn_RM, ara, this_tick, Button.RIGHT_MINUS)
            btn_on_action[_btn_RM]=False
            
        if n_def_BLAs < 4 or (Button.RIGHT not in pressed and Button.LEFT not in pressed):
            if Button.RIGHT_PLUS in pressed and Button.RIGHT_PLUS not in prev_pressed:
                ara=defined_BLAs[0].plus_on(this_tick)
                save_autorepeat(_btn_RP, ara, this_tick, Button.RIGHT_PLUS)
                btn_on_action[_btn_RP]=True
            
            if Button.RIGHT_MINUS in pressed and Button.RIGHT_MINUS not in prev_pressed:
                ara=defined_BLAs[0].minus_on(this_tick)
                save_autorepeat(_btn_RM, ara, this_tick, Button.RIGHT_MINUS)
                btn_on_action[_btn_RM]=True


        # BLA Command 2
        if Button.LEFT_PLUS not in pressed and Button.LEFT_PLUS in prev_pressed and btn_on_action[_btn_LP]:
            ara=defined_BLAs[1].plus_off(this_tick)
            save_autorepeat(_btn_LP, ara, this_tick, Button.LEFT_PLUS)
            btn_on_action[_btn_LP]=False

        if Button.LEFT_MINUS not in pressed and Button.LEFT_MINUS in prev_pressed and btn_on_action[_btn_LM]:
            ara=defined_BLAs[1].minus_off(this_tick)
            save_autorepeat(_btn_LM, ara, this_tick, Button.LEFT_MINUS)
            btn_on_action[_btn_LM]=False

        if n_def_BLAs >= 2 and (n_def_BLAs < 4 or (Button.RIGHT not in pressed and Button.LEFT not in pressed)):
            if Button.LEFT_PLUS in pressed and Button.LEFT_PLUS not in prev_pressed:
                ara=defined_BLAs[1].plus_on(this_tick)
                save_autorepeat(_btn_LP, ara, this_tick, Button.LEFT_PLUS)
                btn_on_action[_btn_LP]=True

            if Button.LEFT_MINUS in pressed and Button.LEFT_MINUS not in prev_pressed:
                ara=defined_BLAs[1].minus_on(this_tick)
                save_autorepeat(_btn_LM, ara, this_tick, Button.LEFT_MINUS)
                btn_on_action[_btn_LM]=True


        # 2 BLA defined, can use red buttons to stop corresponding device...
        if n_def_BLAs in (1,2) and lp==1 and Button.RIGHT in pressed:
            defined_BLAs[0].dev_stop()
        if n_def_BLAs == 2 and lp==1 and Button.LEFT in pressed:
            defined_BLAs[1].dev_stop()
                
        # 3 BLAs defined if there are only 3 commands. Red right and left buttons are plus and minus
        if n_def_BLAs == 3:
            if Button.RIGHT not in pressed and Button.RIGHT in prev_pressed and btn_on_action[_btn_RR]:
                ara=defined_BLAs[2].plus_off(this_tick)
                save_autorepeat(_btn_RR, ara, this_tick, Button.RIGHT)

            if Button.LEFT not in pressed and Button.LEFT in prev_pressed and btn_on_action[_btn_LR]:
                ara=defined_BLAs[2].minus_off(this_tick)
                save_autorepeat(_btn_LR, ara, this_tick, Button.LEFT)

            if Button.RIGHT in pressed and Button.RIGHT not in prev_pressed:
                ara=defined_BLAs[2].plus_on(this_tick)
                save_autorepeat(_btn_RR, ara, this_tick, Button.RIGHT)
                btn_on_action[_btn_RR]=True

            if Button.LEFT in pressed and Button.LEFT not in prev_pressed:
                ara=defined_BLAs[2].minus_on(this_tick)
                save_autorepeat(_btn_LR, ara, this_tick, Button.LEFT)
                btn_on_action[_btn_LR]=True


        # 4 BLAs defined, red buttons do shift
        if n_def_BLAs == 4 and (Button.RIGHT in pressed or Button.LEFT in pressed):
            # Shift Buttons RIGHT
            if Button.RIGHT_PLUS in pressed and Button.RIGHT_PLUS not in prev_pressed:
                ara=defined_BLAs[2].plus_on(this_tick)
                save_autorepeat(_btn_SRP, ara, this_tick, Button.RIGHT_PLUS)
                btn_on_action[_btn_SRP]=True

            if Button.RIGHT_MINUS in pressed and Button.RIGHT_MINUS not in prev_pressed:
                ara=defined_BLAs[2].minus_on(this_tick)
                save_autorepeat(_btn_SRM, ara, this_tick, Button.RIGHT_MINUS)
                btn_on_action[_btn_SRM]=True

            # Shift Buttons LEFT
            if Button.LEFT_PLUS in pressed and Button.LEFT_PLUS not in prev_pressed:
                ara=defined_BLAs[3].plus_on(this_tick)
                save_autorepeat(_btn_SLP, ara, this_tick, Button.LEFT_PLUS)
                btn_on_action[_btn_SLP]=True

            if Button.LEFT_MINUS in pressed and Button.LEFT_MINUS not in prev_pressed:
                ara=defined_BLAs[3].minus_on(this_tick)
                save_autorepeat(_btn_SLM, ara, this_tick, Button.LEFT_MINUS)
                btn_on_action[_btn_SLM]=True

        if n_def_BLAs == 4:
            if Button.RIGHT_PLUS not in pressed and Button.RIGHT_PLUS in prev_pressed and btn_on_action[_btn_SRP]:
                ara=defined_BLAs[2].plus_off(this_tick)
                save_autorepeat(_btn_SRP, ara, this_tick, Button.RIGHT_PLUS)
                btn_on_action[_btn_SRP]=False

            if Button.RIGHT_MINUS not in pressed and Button.RIGHT_MINUS in prev_pressed and btn_on_action[_btn_SRM]:
                ara=defined_BLAs[2].minus_off(this_tick)
                save_autorepeat(_btn_SRM, ara, this_tick, Button.RIGHT_MINUS)
                btn_on_action[_btn_SRM]=False

            if Button.LEFT_PLUS not in pressed and Button.LEFT_PLUS in prev_pressed and btn_on_action[_btn_SLP]:
                ara=defined_BLAs[3].plus_off(this_tick)
                save_autorepeat(_btn_SLP, ara, this_tick, Button.LEFT_PLUS)
                btn_on_action[_btn_SLP]=False
               
            if Button.LEFT_MINUS not in pressed and Button.LEFT_MINUS in prev_pressed and btn_on_action[_btn_SLM]:
                ara=defined_BLAs[3].minus_off(this_tick)
                save_autorepeat(_btn_SLM, ara, this_tick, Button.LEFT_MINUS)
                btn_on_action[_btn_SLM]=False
           
        for i,a in enumerate(auto_repeat):
            if a == None:
                continue
            if (a[2] is None or (a[2] in pressed and a[2] in prev_pressed)) and this_tick-a[1] > a[3]:
                nara=a[0](this_tick)
                save_autorepeat(i, nara, this_tick, a[2])

        prev_pressed=pressed

        
hub=ThisHub()
bla_n_ports = { '<TechnicHub>': 4, '<PrimeHub>': 6, '<InventorHub>': 6 }[str(hub)]
bla_ports=(Port.A,Port.B,Port.C,Port.D) if bla_n_ports==4 else (Port.A,Port.B,Port.C,Port.D,Port.E,Port.F)
bla_n_modes = 4
hub.system.set_stop_button(None)
hub.light.blink(Color.WHITE, (200,200, 100, 400))
rem = Remote()
rem.light.on(bla_play_color)
if not BLABase.load_config(rem.name()):
     bla_config()
    
while True:
    try:
        bla_play()
    except Exception as e:
        hub.light.blink(Color.ORANGE, (100,50))
        wait(3000) # a lot, yes!
    bla_config() 

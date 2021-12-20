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
# Keep in mind that this is a computer with an input device of 7 buttons
# and an output device of just two LEDs. This is a Star Trek like
# interface, a real YDSWIG interface: you don't see what you get.
# However, once you learn it, you will notice that it is simple.
# No screens attached!
#
# There is a users manual in PDF avaiable.
#
# Version: 0.98
#
# Author VascoLP
#
# Date: December 2021
#
# Installing:
# Install Pybricks firmware with Remote Bla Bla (this program) included, on a TechnicHub
# You should use pybricks firmware beta version v3.1.0c1 on 2021-11-19.
# It should also run on the soon to be released pybricks version v3.1 firmware.
#

from micropython import const
from pybricks.hubs import TechnicHub
from pybricks.tools import StopWatch
from pybricks.iodevices import PUPDevice
from pybricks.pupdevices import DCMotor, Motor, Remote
from pybricks.parameters import Port, Direction, Stop, Button, Color
from pybricks.tools import wait
from uerrno import ENODEV
from math import trunc

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
bla_play_color=Color.GREEN*0.8   # Color for play mode
bla_send_color=Color.GREEN*0.8
bla_error_color=Color.RED
bla_dev_init_color=Color.RED*0.4
_bla_auto_repeat_timeout=const(300)
_bla_leave_play_timeout=const(500)
_bla_shutdown_timeout=const(1000)
_bla_timeout=const(5*60*1000) # In config mode, after this time shutsdown
_bla_config_tick=const(100) # Should be enough for config mode...
_bla_play_tick=const(10) 
_bla_init_speed=const(250) # Speed used when initializing motors
_bla_saved_param_prefix='rb'
_bla_ports=(Port.A, Port.B, Port.C, Port.D)
_bla_n_ports=const(4)
_bla_default_speed=const(1500)

_bla_version_color=Color.CYAN
_bla_version_num=1

# Global variables
hub=None # the Hub
rem=None # the Remote

defined_BLAs=[] # List of defined BLAs

bla_dir12=( Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE )
bla_dir21=( Direction.COUNTERCLOCKWISE, Direction.CLOCKWISE )
bla_dir11=( Direction.CLOCKWISE, Direction.CLOCKWISE )
bla_dir22=( Direction.COUNTERCLOCKWISE, Direction.COUNTERCLOCKWISE )
bla_pwr3=(0,33,67,100)
bla_pwr5=(0,20,40,60,80,100)
bla_pwr7=(0,14,29,43,57,71,86,100)
bla_pwr9=(0,11,22,33,44,56,67,78,89,100)

######################################################################
def get_dir_for_mode(mode_p, n_ports_p):
    if n_ports_p==1:
        return (bla_dir11  if mode_p in (_btn_RP, _btn_LP, _btn_SRP, _btn_SLP) else
                bla_dir22 #if mode_p in (_btn_RM, _btn_LM, _btn_SRM, _btn_SLM)
        )
    return (bla_dir11 if mode_p in (_btn_RP, _btn_SRP) else bla_dir12  if mode_p in (_btn_LP, _btn_SLP) else
            bla_dir22 if mode_p in (_btn_RM, _btn_SRM) else bla_dir21 #if mode_p in (_btn_LM, _btn_SLM)
    )

######################################################################
class BLABase:
    bla_list=()
    cfg_codes='0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_-'
    cfg_empty='...'

    @staticmethod
    def get_cfg_ports(ports_p):
        cfg_ports=[0 for _ in range(_bla_n_ports)]
        for i,p in enumerate(ports_p):
            cfg_ports[_bla_ports.index(p)]=1
        return cfg_ports
    
    @classmethod
    def bla_list_len(cls):
        return len(cls.bla_list)
    
    @classmethod
    def make(cls, bla_p, ports_p, mode_p):
        return cls.bla_list[bla_p][0](bla_p, ports_p, None, mode_p, *(cls.bla_list[bla_p][1]))

    @classmethod
    def make_from_cfg(cls, cfg_p):
        #  bbbbb. mmm.pp pppppp
        p=(cls.cfg_codes.index(cfg_p[0])<<12)|(cls.cfg_codes.index(cfg_p[1])<<6)|(cls.cfg_codes.index(cfg_p[2]))
        bla=(p>>13)&31
        mode=(p>>9)&7
        ports_mask=p&255
        cfg_ports=[]
        for i in range(_bla_n_ports):
            cfg_ports.append((ports_mask>>(_bla_n_ports-i)*2-2)&3)
        return cls.bla_list[bla][0](bla, None, cfg_ports, mode, *(cls.bla_list[bla][1]))
    
    def __init__(self, bla_p, ports_p, cfg_ports_p, mode_p):
        self.bla=bla_p
        self.cfg_ports = self.get_cfg_ports(ports_p) if cfg_ports_p == None else cfg_ports_p
        self.mode=mode_p
        self.devices=[]
        
    def blink_on_limit(self):
        rem.light.on(bla_error_color)
        rem.light.on(bla_play_color)
        return self.blink_on_limit
        
    def blink_on_zero(self):
        rem.light.on(Color.MAGENTA)
        rem.light.on(bla_play_color)

    def get_cfg(self):
        #  bbbbb. mmm.pp pppppp
        ports_mask=0
        for i,p in enumerate(self.cfg_ports):
            ports_mask |= p<<((_bla_n_ports-i)*2-2)
        p=(self.bla<<13)|(self.mode<<9)|ports_mask
        return ''.join((self.cfg_codes[p>>12&0b111111], self.cfg_codes[p>>6&0b111111], self.cfg_codes[p&0b111111]))
        
######################################################################
class BLAStepsMotor(BLABase):

    @staticmethod
    def get_cfg_ports(ports_p):
        cfg_ports=[0 for _ in range(_bla_n_ports)]
        for i,p in enumerate(ports_p):
            try:
                td=PUPDevice(p)
                did=td.info()['id']
                if did in (1,2): # Powered Up Medium Motor, Powered Up Train Motor (sensorless motors)
                    cfg_ports[_bla_ports.index(p)]=2
                #elif did in (8): # Powered Up Lights - should work but I don't have any
                else: # Assume motor with sensor
                    cfg_ports[_bla_ports.index(p)]=1
            except OSError as ex:
                raise ValueError('Invalid device type!') # Comunicate that the given device is not valid
        return cfg_ports
    
    def __init__(self, bla_p, ports_p, cfg_ports_p, mode_p, is_one_step_p):
        super().__init__(bla_p, ports_p, cfg_ports_p, mode_p)

        ndev=sum([1 if i>0 else 0 for i in self.cfg_ports])
        
        if ndev==1:
            dir= get_dir_for_mode(self.mode,1)
            if is_one_step_p:
                speed = (100, 80, 60, 40, 100, 80, 60, 40)[self.mode]
                self.powers=(0, speed)
            else:
                self.powers = (bla_pwr7, bla_pwr5, bla_pwr3, bla_pwr9,bla_pwr7, bla_pwr5, bla_pwr3, bla_pwr9)[self.mode]
        else:
            dir= get_dir_for_mode(self.mode,2)
            if is_one_step_p:
                speed = (100, 100, 70, 70, 100, 100, 70, 70)[self.mode]
                self.powers=(0, speed)
            else:
                self.powers = (bla_pwr7, bla_pwr7, bla_pwr5, bla_pwr5, bla_pwr7, bla_pwr7, bla_pwr5, bla_pwr5)[self.mode]
                
        self.powers_max=len(self.powers)-1
        self.current_power=0
        self.is_one_step = is_one_step_p

        for i,p in enumerate(self.cfg_ports):
            if p == 0:
                continue
            d = None
            if p == 1:
                d=Motor(_bla_ports[i], dir[i%2])
            elif p == 2:
                d=DCMotor(_bla_ports[i], dir[i%2])
            self.devices.append(d)

            
    def dev_init(self):
        self.current_power=0

    def _set_power(self):
        for d in self.devices:
            d.dc((1 if self.current_power>=0 else -1)*self.powers[abs(self.current_power)])
            
    def plus_or_minus_on(self, dir, ar):
        if self.is_one_step:
            self.current_power = dir
            self._set_power()
            return
        
        if self.current_power == dir*self.powers_max:
            return self.blink_on_limit
        self.current_power = self.current_power+dir
        self._set_power()
        if self.current_power == 0:
            self.blink_on_zero()
        return ar

    def plus_on(self):
        return self.plus_or_minus_on(1, self.plus_on)

    def minus_on(self):
        return self.plus_or_minus_on(-1, self.minus_on)

    def dev_stop(self):
        self.current_power=0
        for d in self.devices:
            d.dc(0)

    def plus_off(self):
        if self.is_one_step:
            self.dev_stop()

    def minus_off(self):
        if self.is_one_step:
            self.dev_stop()
    

# BLASteering types
_bla_ste_stepper_zero=       const(0) # initialize on motor absolute zero and go on from there
_bla_ste_stepper_find_init=  const(1) # Finds ititial position by stalling and go on from there
_bla_ste_stepper_find_limits=const(2) # Finds ititial position, find final position and use step size to get number of speeds
_bla_ste_steering_servo=     const(3) # servo mode, steps configurable
_bla_ste_steering_limits=    const(4) # find limits and steer, steps configurable

class BLASteering(BLABase):

    # Phylo says: L Motor: 315rpm, XL Motor: 330rpm. In degs/s 1rpm=6deg/s so: 1890deg/s and 1980deg/s.

    round_angle=True

    #--------------------------------------------------
    def __init__(self, bla_p, ports_p, cfg_ports_p, mode_p, type_p, step_p, n_steps_p, dir1_p, dir2_p):
        # For the record: In a gearbox with N speeds there are N-1 steps

        super().__init__(bla_p, ports_p, cfg_ports_p, mode_p)
        ndev=sum([1 if i>0 else 0 for i in self.cfg_ports])

        self.bla_type=type_p
        self.then = (Stop.HOLD, Stop.COAST)[self.mode%2]
        self.step=step_p # step size in degrees
        dir = dir1_p if self.mode <4 else dir2_p
        self.speed = _bla_default_speed
        self.n_steps=n_steps_p
        if self.bla_type == _bla_ste_stepper_find_limits:
            if ndev>1:
                raise ValueError('Find Limits does not work with more than one device!')
            if  self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
                self.step*=2
        elif self.bla_type in(_bla_ste_steering_servo, _bla_ste_steering_limits) and self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
            self.n_steps+=2
        elif self.bla_type in (_bla_ste_stepper_zero, _bla_ste_stepper_find_init) and self.mode in (_btn_SRP,_btn_SLP,_btn_SRM,_btn_SLM):
            self.n_steps+=1

        for i,p in enumerate(self.cfg_ports):
            if p == 0:
                continue
            d=Motor(_bla_ports[i], dir[i%2])
            #d.control.limits(speed=2000,acceleration=15000,duty=100,torque=1500)
            #d.control.limits(speed=1800,acceleration=5000)
            d.control.limits(acceleration=5000)
            self.devices.append(d)
            
    #--------------------------------------------------
    def dev_init(self):
        self.angles=[]
        self.current_step=0
        for di, d in enumerate(self.devices):
            self.angles.append([])
            if self.bla_type ==_bla_ste_stepper_zero:
                d.reset_angle() # Set initial angle to absolute 0
                if d.angle()<0: # Go to absolute zero always in backwards direction
                    d.run_angle(-_bla_init_speed, 180, self.then, True)
                    d.reset_angle()
                d.run_target(_bla_init_speed, 0, self.then, True)
                for i in range(self.n_steps+1):
                    self.angles[di].append(i*self.step)

            if self.bla_type == _bla_ste_stepper_find_init:
                d.reset_angle() 
                # Find initial position by stalling
                lower_end=d.run_until_stalled(-_bla_init_speed, self.then)
                if self.round_angle:
                    rnd = trunc(lower_end/self.step)*self.step
                    lower_end = rnd + self.step if rnd < lower_end else rnd
                    d.run_target(_bla_init_speed, lower_end, self.then, True)
                d.reset_angle(0)
                for i in range(self.n_steps+1):
                    self.angles[di].append(i*self.step)
 
            if self.bla_type == _bla_ste_stepper_find_limits:
                d.reset_angle() 
                # Find limits by stalling and guess number os speeds
                lower_end=d.run_until_stalled(-_bla_init_speed, self.then)
                if self.round_angle:
                    rnd = trunc(lower_end/self.step)*self.step
                    lower_end = rnd + self.step if rnd < lower_end else rnd
                    d.run_target(_bla_init_speed, lower_end, self.then, True)
                d.reset_angle(0)
                #d.run_target(_bla_init_speed, 0, self.then, True)
                upper_end=d.run_until_stalled(_bla_init_speed, self.then)            
                if self.round_angle:
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
                    
            if self.bla_type ==_bla_ste_steering_servo:
                # Set initial angle to absolute 0                
                d.reset_angle()
                d.run_target(_bla_init_speed, 0, self.then, True)
                for i in range(self.n_steps+1):
                    self.angles[di].append(i*self.step/self.n_steps)

            if self.bla_type == _bla_ste_steering_limits:
                # Find limits positions. Will find angles by dividing the range of positions in n_steps
                lower_end=d.run_until_stalled(-_bla_init_speed, self.then)
                upper_end=d.run_until_stalled(_bla_init_speed, self.then)            
                sa=(upper_end - lower_end) / 2.0
                d.reset_angle(sa)
                d.run_target(_bla_init_speed, 0, self.then, True) # Make sa angle the 0 angle
                for i in range(self.n_steps+1):
                    self.angles[di].append(i*sa/self.n_steps)

    def _set_step(self, wait_p=False):
        for di, d in enumerate(self.devices):
            d.run_target(self.speed,
                         (1 if self.current_step>=0 else -1)*self.angles[di][abs(self.current_step)],
                         self.then, wait_p)
                    
    def plus_on(self):
        if self.current_step == self.n_steps:
            return self.blink_on_limit if self.n_steps>1 else None
        self.current_step = self.current_step+1
        self._set_step()
        if self.current_step == 0:
            self.blink_on_zero()
        return self.plus_on

    def minus_on(self):
        lmt = -self.n_steps if self.bla_type in (_bla_ste_steering_servo, _bla_ste_steering_limits) else 0
        if self.current_step == lmt:
            return self.blink_on_limit if self.n_steps>1 else None
        self.current_step = self.current_step-1
        self._set_step()
        if self.current_step == 0:
            self.blink_on_zero()
        return self.minus_on
            
    def plus_off(self):
        if self.bla_type in (_bla_ste_steering_servo,_bla_ste_steering_limits) and self.n_steps == 1:
            self.current_step = 0
            for d in self.devices:
                d.run_target(self.speed, 0, self.then, False)

    def minus_off(self):
        return self.plus_off()

    def dev_stop(self):
        self.current_step=0
        for d in self.devices:
            self._set_step(True)
            d.stop()

######################################################################
# Existing BLAs definition tupple. Maximum 31, each one in its position in the list.
# Each element is a tuple where first position is a BLA  object and second position a tuple of extra arguments to the BLA Object costructor 
#
BLABase.bla_list=(
    (BLAStepsMotor, (True,)                                                    ), # BLASimpleMotor          CYAN
    (BLAStepsMotor, (False,)                                                   ), # BLAStepsMotor           YELLOW
    (BLASteering,   (_bla_ste_steering_limits,     90, 1, bla_dir11, bla_dir22)), # BlaSteering_1_3         GRAY
    (BLASteering,   (_bla_ste_steering_limits,     90, 5, bla_dir11, bla_dir22)), # BlaSteering_5_7         ORANGE
    (BLASteering,   (_bla_ste_stepper_find_limits, 90, 0, bla_dir11, bla_dir22)), # BLAStepperLimits90_180  BLUE
    (BLASteering,   (_bla_ste_steering_servo,      90, 1, bla_dir11, bla_dir22)), # BLAServo90_1_3          MAGENTA
    (BLASteering,   (_bla_ste_steering_servo,      90, 5, bla_dir11, bla_dir22)), # BLAServo90_5_7          GREEN
    (BLASteering,   (_bla_ste_stepper_zero,        90, 1, bla_dir11, bla_dir22)), # BLAStepperZero90_2_3    WHITE 
    (BLASteering,   (_bla_ste_stepper_zero,        90, 3, bla_dir11, bla_dir22)), # BLAStepperZero90_4_5    CYAN   
    (BLASteering,   (_bla_ste_stepper_zero,        90, 5, bla_dir11, bla_dir22)), # BLAStepperZero90_6_7    YELLOW 
    (BLASteering,   (_bla_ste_stepper_zero,        90, 7, bla_dir11, bla_dir22)), # BLAStepperZero90_8_9    GRAY   
    (BLASteering,   (_bla_ste_stepper_find_init,   90, 1, bla_dir11, bla_dir22)), # BLAStepperInit90_2_3    ORANGE 
    (BLASteering,   (_bla_ste_stepper_find_init,   90, 3, bla_dir11, bla_dir22)), # BLAStepperInit90_4_5    BLUE   
    (BLASteering,   (_bla_ste_stepper_find_init,   90, 5, bla_dir11, bla_dir22)), # BLAStepperInit90_6_7    MAGENTA
    (BLASteering,   (_bla_ste_stepper_find_init,   90, 7, bla_dir11, bla_dir22)), # BLAStepperInit90_8_9    GREEN  
    (BLASteering,   (_bla_ste_steering_limits,     90, 1, bla_dir12, bla_dir21)), # BlaSteeringRev_1_3      WHITE 
    (BLASteering,   (_bla_ste_steering_limits,     90, 5, bla_dir12, bla_dir21)), # BlaSteeringRev_5_7      CYAN   
    (BLASteering,   (_bla_ste_steering_servo,      90, 1, bla_dir12, bla_dir21)), # BLAServo90Rev_1_3       YELLOW 
    (BLASteering,   (_bla_ste_steering_servo,      90, 5, bla_dir12, bla_dir21)), # BLAServo90Rev_5_7       GRAY   
    (BLASteering,   (_bla_ste_stepper_zero,        90, 1, bla_dir12, bla_dir21)), # BLAStepperZero90Rev_2_3 ORANGE 
    (BLASteering,   (_bla_ste_stepper_zero,        90, 3, bla_dir12, bla_dir21)), # BLAStepperZero90Rev_4_5 BLUE   
    (BLASteering,   (_bla_ste_stepper_zero,        90, 5, bla_dir12, bla_dir21)), # BLAStepperZero90Rev_6_7 MAGENTA
    (BLASteering,   (_bla_ste_stepper_zero,        90, 7, bla_dir12, bla_dir21)), # BLAStepperZero90Rev_8_9 GREEN  
    (BLASteering,   (_bla_ste_stepper_find_init,   90, 1, bla_dir12, bla_dir21)), # BLAStepperInit90Rev_2_3 WHITE 
    (BLASteering,   (_bla_ste_stepper_find_init,   90, 3, bla_dir12, bla_dir21)), # BLAStepperInit90Rev_4_5 CYAN   
    (BLASteering,   (_bla_ste_stepper_find_init,   90, 5, bla_dir12, bla_dir21)), # BLAStepperInit90Rev_6_7 YELLOW 
    (BLASteering,   (_bla_ste_stepper_find_init,   90, 7, bla_dir12, bla_dir21)), # BLAStepperInit90Rev_8_9 GRAY   
    (BLASteering,   (_bla_ste_stepper_find_init,   60, 4, bla_dir11, bla_dir22))  # BLAStepperInit60_5_6    ORANGE 
)

######################################################################
def set_bla_config_BLA_blink(m):
    # """ Sets the hub light for m config mode definition
    # """
    
    global hub
    c= (
        Color.CYAN   ,
        Color.YELLOW ,
        Color.GRAY   ,
        Color.ORANGE ,
        Color.BLUE   ,
        Color.MAGENTA,
        Color.GREEN  ,
        Color.WHITE   
        )[m%8]     
    b=(
        (5, 300, 1500, 5),
        (5, 200, 500, 5), 
        (5, 100, 300, 5)
        )[int((m-8)/8)]

    if m < 8: # No blinking
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
def config_from_parameters(n):
    global defined_BLAs,hub

    hub.light.on(bla_dev_init_color)
    if len(n)!=14 or n[0:2]!=_bla_saved_param_prefix:
        wait(100) # for luck
        return False
    try:
        par=n[2:]
        defined_BLAs=[]
        for i in range(0,12,3):
            pp=par[i:i+3]
            if pp==BLABase.cfg_empty:
                break
            m=BLABase.make_from_cfg(pp)
            defined_BLAs.append(m)
        wait(100) # for luck
    except ValueError as ve:
        wait(100) # for luck
        return False
    except OSError as os:
        if os.args[0] == ENODEV:
            wait(100) # for luck
            return False
    return True if len(defined_BLAs) else False

######################################################################
def check_shutdown(sw, hub_button_pressed):
    if hub_button_pressed == None and Button.CENTER in hub.button.pressed():
        return sw.time()

    if hub_button_pressed != None and Button.CENTER in hub.button.pressed():
        if sw.time()-hub_button_pressed>_bla_shutdown_timeout:
            hub.system.shutdown()
        else:
            return hub_button_pressed

    return None
    
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
    while True:
        wait(_bla_config_tick) # in the beginning so that we can use nice continue statments
        hub_button_pressed=check_shutdown(sw,hub_button_pressed)
        
        pressed = rem.buttons.pressed()

        if sw.time()-idle_init>_bla_timeout:
            # It will also shutdown if you press a button for _bla_timeout miliseconds,
            # because of next test... 
            hub_mini_error()
            hub.system.shutdown()

        if prev_pressed == pressed:
            continue
        idle_init=sw.time()

        prev_pressed=pressed
        lp=len(pressed)
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
            if bla_rrb:
                cur_BLA= 0 if cur_BLA==BLABase.bla_list_len()-1 else cur_BLA+1
            else:
                cur_BLA= BLABase.bla_list_len()-1 if cur_BLA==0 else cur_BLA-1
            set_bla_config_BLA_blink(cur_BLA)
            bla_rrb = False
            bla_lrb = False
            continue
        bla_rrb = False
        bla_lrb = False

        if Button.CENTER in pressed:
            if lp==1:
                # Center button leaves config, if at least one BLA is defined
                # It also saves configuration
                hub.light.blink(Color.GREEN if len(defined_BLAs)>0 else Color.RED, [100, 50])
                wait(500)
                if len(defined_BLAs)>0:
                    cfg_par = _bla_saved_param_prefix
                    for b in defined_BLAs:
                        cfg_par += b.get_cfg()
                    cfg_par += BLABase.cfg_empty*(_bla_n_ports-len(defined_BLAs))
                    rem.name(cfg_par)
                    break
                else:
                    set_bla_config_BLA_blink(cur_BLA)
                    continue
            if lp==2: # Shift-Center button deletes last BLA created
                if len(defined_BLAs) == 0: # nothing to delete
                    hub_mini_error()
                else:
                    b=defined_BLAs[len(defined_BLAs)-1]
                    for i,p in enumerate(b.cfg_ports):
                        if p != 0:
                            used_ports.remove(_bla_ports[i])
                    defined_BLAs.remove(b)
                    hub.light.blink(bla_send_color, [50, 50]) # Show user success message
                    wait(300)
                    hub.light.on(bla_send_color)
                    wait(200)
                set_bla_config_BLA_blink(cur_BLA)
            elif lp==3: # Right and Left and Center all pressed at the same time show version
                hub.light.on(Color.WHITE)
                wait(1000)
                for i in range(_bla_version_num):
                    hub.light.on(_bla_version_color)
                    wait(200)
                    hub.light.on(Color.WHITE)
                    wait(200)
                hub.light.on(Color.WHITE)
                wait(1000)                    
                set_bla_config_BLA_blink(cur_BLA)
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

        new_ports=[]
        ports_mask=0
        hub.light.on(Color.RED*0.4) # Put hub light in red while detecting devices, because device detection fails sometimes
                                    # specially with DCMotors and low battery
        for i,p in enumerate(_bla_ports):
            if p in used_ports:
                continue
            try:
                _=PUPDevice(p)
                new_ports.append(p)
            except OSError as ex:
                if ex.args[0] != ENODEV:    #No device found on this port.
                    raise
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
        set_bla_config_BLA_blink(cur_BLA)

######################################################################
def bla_play():

    global defined_BLAs,rem,hub

    # Autorepeat for selected modes.
    # The methods that implement a given button function, like, for instance, PLUS, can return a method pointer 
    # for the autorepeat action. This method will be called if the button is kept pressed for some time.
    # If the returned method returns a funcion pointer, the auto repeat is rearmed. If it returns null,
    # autorepeat stops.
    # Each position in the list represents a different button.
    auto_repeat = [None, None, None, None, None, None, None, None]
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
        hub_button_pressed=check_shutdown(sw, hub_button_pressed)

        pressed = rem.buttons.pressed()
        lp=len(pressed)
        if lp == 0 and sw.time()-idle_init>_bla_timeout:
            # It will also shutdown if you press a button for _bla_timeout miliseconds,
            # because of next test...
            hub_mini_error()
            hub.system.shutdown()
        if lp != 0:
            idle_init=sw.time()
        
        if lp==1 and Button.CENTER in pressed and Button.CENTER not in prev_pressed:
            button_center_pressed=sw.time()

        if lp==1 and Button.CENTER in pressed and Button.CENTER in prev_pressed and sw.time()-button_center_pressed>_bla_leave_play_timeout:
            rem.light.on(Color.ORANGE*0.8) # Immediate feedback because 1.2s in remote green button turns off remote...
            hub.light.on(Color.ORANGE*0.8)

        if lp==0 and Button.CENTER not in pressed and Button.CENTER in prev_pressed and sw.time()-button_center_pressed>_bla_leave_play_timeout:
            prev_pressed=pressed
            for d in defined_BLAs:
                d.dev_stop()
            break

        # BLA Command 1
        if Button.RIGHT_PLUS not in pressed and Button.RIGHT_PLUS in prev_pressed and btn_on_action[_btn_RP]:
            defined_BLAs[0].plus_off()
            auto_repeat[_btn_RP]=None
            btn_on_action[_btn_RP]=False
                
        if Button.RIGHT_MINUS not in pressed and Button.RIGHT_MINUS in prev_pressed and btn_on_action[_btn_RM]:
            defined_BLAs[0].minus_off()
            auto_repeat[_btn_RM]=None
            btn_on_action[_btn_RM]=False
            
        if n_def_BLAs < 4 or (Button.RIGHT not in pressed and Button.LEFT not in pressed):
            if Button.RIGHT_PLUS in pressed and Button.RIGHT_PLUS not in prev_pressed:
                ara=defined_BLAs[0].plus_on()
                auto_repeat[_btn_RP]=(ara, sw.time(), Button.RIGHT_PLUS) if ara != None else None
                btn_on_action[_btn_RP]=True
            
            if Button.RIGHT_MINUS in pressed and Button.RIGHT_MINUS not in prev_pressed:
                ara=defined_BLAs[0].minus_on()
                auto_repeat[_btn_RM]=(ara, sw.time(), Button.RIGHT_MINUS) if ara != None else None
                btn_on_action[_btn_RM]=True


        # BLA Command 2
        if Button.LEFT_PLUS not in pressed and Button.LEFT_PLUS in prev_pressed and btn_on_action[_btn_LP]:
            defined_BLAs[1].plus_off()
            auto_repeat[_btn_LP]=None
            btn_on_action[_btn_LP]=False

        if Button.LEFT_MINUS not in pressed and Button.LEFT_MINUS in prev_pressed and btn_on_action[_btn_LM]:
            defined_BLAs[1].minus_off()
            auto_repeat[_btn_LM]=None
            btn_on_action[_btn_LM]=False

        if n_def_BLAs >= 2 and (n_def_BLAs < 4 or (Button.RIGHT not in pressed and Button.LEFT not in pressed)):
            if Button.LEFT_PLUS in pressed and Button.LEFT_PLUS not in prev_pressed:
                ara=defined_BLAs[1].plus_on()
                auto_repeat[_btn_LP]=(ara, sw.time(), Button.LEFT_PLUS) if ara != None else None
                btn_on_action[_btn_LP]=True

            if Button.LEFT_MINUS in pressed and Button.LEFT_MINUS not in prev_pressed:
                ara=defined_BLAs[1].minus_on()
                auto_repeat[_btn_LM]=(ara, sw.time(), Button.LEFT_MINUS) if ara != None else None
                btn_on_action[_btn_LM]=True


        # 2 BLA defined, can use red buttons to stop corresponding device...
        if n_def_BLAs in (1,2) and lp==1 and Button.RIGHT in pressed:
            defined_BLAs[0].dev_stop()
        if n_def_BLAs == 2 and lp==1 and Button.LEFT in pressed:
            defined_BLAs[1].dev_stop()
                
        # 3 BLAs defined if there are only 3 commands. Red right and left buttons are plus and minus
        if n_def_BLAs == 3:
            if Button.RIGHT not in pressed and Button.RIGHT in prev_pressed:
                auto_repeat[_btn_RR]=None
                defined_BLAs[2].plus_off()

            if Button.LEFT not in pressed and Button.LEFT in prev_pressed:
                auto_repeat[_btn_LR]=None
                defined_BLAs[2].minus_off()

            if Button.RIGHT in pressed and Button.RIGHT not in prev_pressed:
                ara=defined_BLAs[2].plus_on()
                auto_repeat[_btn_RR]=(ara, sw.time(), Button.RIGHT) if ara != None else None

            if Button.LEFT in pressed and Button.LEFT not in prev_pressed:
                ara=defined_BLAs[2].minus_on()
                auto_repeat[_btn_LR]=(ara, sw.time(), Button.LEFT) if ara != None else None


        # 4 BLAs defined, red buttons do shift
        if n_def_BLAs == 4 and (Button.RIGHT in pressed or Button.LEFT in pressed):
            # Shift Buttons RIGHT
            if Button.RIGHT_PLUS in pressed and Button.RIGHT_PLUS not in prev_pressed:
                ara=defined_BLAs[2].plus_on()
                auto_repeat[_btn_SRP]=(ara, sw.time(), Button.RIGHT_PLUS) if ara != None else None
                btn_on_action[_btn_SRP]=True

            if Button.RIGHT_MINUS in pressed and Button.RIGHT_MINUS not in prev_pressed:
                ara=defined_BLAs[2].minus_on()
                auto_repeat[_btn_SRM]=(ara, sw.time(), Button.RIGHT_MINUS) if ara != None else None
                btn_on_action[_btn_SRM]=True

            # Shift Buttons LEFT
            if Button.LEFT_PLUS in pressed and Button.LEFT_PLUS not in prev_pressed:
                ara=defined_BLAs[3].plus_on()
                auto_repeat[_btn_SLP]=(ara, sw.time(), Button.LEFT_PLUS) if ara != None else None
                btn_on_action[_btn_SLP]=True

            if Button.LEFT_MINUS in pressed and Button.LEFT_MINUS not in prev_pressed:
                ara=defined_BLAs[3].minus_on()
                auto_repeat[_btn_SLM]=(ara, sw.time(), Button.LEFT_MINUS) if ara != None else None
                btn_on_action[_btn_SLM]=True

        if n_def_BLAs == 4:
            if Button.RIGHT_PLUS not in pressed and Button.RIGHT_PLUS in prev_pressed and btn_on_action[_btn_SRP]:
                defined_BLAs[2].plus_off()
                auto_repeat[_btn_SRP]=None
                btn_on_action[_btn_SRP]=False

            if Button.RIGHT_MINUS not in pressed and Button.RIGHT_MINUS in prev_pressed and btn_on_action[_btn_SRM]:
                defined_BLAs[2].minus_off()
                auto_repeat[_btn_SRM]=None
                btn_on_action[_btn_SRM]=False

            if Button.LEFT_PLUS not in pressed and Button.LEFT_PLUS in prev_pressed and btn_on_action[_btn_SLP]:
                defined_BLAs[3].plus_off()
                auto_repeat[_btn_SLP]=None
                btn_on_action[_btn_SLP]=False
               
            if Button.LEFT_MINUS not in pressed and Button.LEFT_MINUS in prev_pressed and btn_on_action[_btn_SLM]:
                defined_BLAs[3].minus_off()
                auto_repeat[_btn_SLM]=None
                btn_on_action[_btn_SLM]=False
           
        for i,a in enumerate(auto_repeat):
            if a == None:
                continue
            if a[2] in pressed and a[2] in prev_pressed and sw.time()-a[1] >_bla_auto_repeat_timeout:
                na=a[0]()
                auto_repeat[i]= (na, sw.time(), a[2]) if na != None else None
            
        prev_pressed=pressed

hub=TechnicHub()
hub.system.set_stop_button(None)
hub.light.blink(Color.WHITE, (200,200, 100, 400))
rem = Remote()
rem.light.on(bla_play_color)
if not config_from_parameters(rem.name()):
    bla_config()

while True:
    try:
        bla_play()
    except Exception as e:
        hub.light.blink(Color.ORANGE, (100,50))
        wait(3000) # a lot, yes!
    bla_config() 

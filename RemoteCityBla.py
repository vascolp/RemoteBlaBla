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
# There is a users manual in PDF avaiable.
#
# Version: 0.98
#
# Author VascoLP
#
# Date: December 2021
#
# Installing:
# Install Pybricks firmware with Remote Bla Bla (this program) included, on a CityHub
# You should use pybricks firmware beta version v3.1.0c1 on 2021-11-19.
# It should also run on the soon to be released pybricks version v3.1 firmware.
#

from micropython import const
from pybricks.hubs import CityHub
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

bla_play_color=Color.GREEN*0.8   # Color for play mode
bla_send_color=Color.GREEN*0.8
bla_error_color=Color.RED
bla_dev_init_color=Color.RED*0.4
_bla_auto_repeat_timeout=const(300)
_bla_leave_play_timeout=const(500)
_bla_shutdown_timeout=const(1000)
_bla_timeout=const(5*60*1000) # In config mode, after this time shutsdown
_bla_play_tick=const(10) 
_bla_init_speed=const(250) # Speed used when initializing motors
_bla_saved_param_prefix='rb'
_bla_ports=(Port.A, Port.B)
_bla_n_ports=const(2)
_bla_default_speed=const(1500)


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

    @classmethod
    def make_from_cfg(cls, cfg_p):
        #  bbbbb. mmm.pp pppppp
        p=(cls.cfg_codes.index(cfg_p[0])<<12)|(cls.cfg_codes.index(cfg_p[1])<<6)|(cls.cfg_codes.index(cfg_p[2]))
        bla=(p>>13)&31
        mode=(p>>9)&7
        ports_mask=p&255
        cfg_ports=[]
        for i in range(_bla_n_ports):
            # cfg_ports.append((ports_mask>>(_bla_n_ports-i)*2-2)&3)
            cfg_ports.append((ports_mask>>(4-i)*2-2)&3) # must be 4...
        return cls.bla_list[bla][0](bla, None, cfg_ports, mode, *(cls.bla_list[bla][1]))
    
    def __init__(self, bla_p, ports_p, cfg_ports_p, mode_p):
        self.bla=bla_p
        # self.cfg_ports = self.get_cfg_ports(ports_p) if cfg_ports_p == None else cfg_ports_p
        self.cfg_ports = cfg_ports_p
        self.mode=mode_p
        self.devices=[]
        
    def blink_on_limit(self):
        rem.light.on(bla_error_color)
        rem.light.on(bla_play_color)
        return self.blink_on_limit
        
    def blink_on_zero(self):
        rem.light.on(Color.MAGENTA)
        rem.light.on(bla_play_color)

######################################################################
class BLAStepsMotor(BLABase):

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
                raise ValueError('DNWM1D') # 'Find Limits does not work with more than one device!'
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
                
        for i,a in enumerate(auto_repeat):
            if a == None:
                continue
            if a[2] in pressed and a[2] in prev_pressed and sw.time()-a[1] >_bla_auto_repeat_timeout:
                na=a[0]()
                auto_repeat[i]= (na, sw.time(), a[2]) if na != None else None
            
        prev_pressed=pressed

hub=CityHub()
hub.system.set_stop_button(None)
hub.light.blink(Color.WHITE, (200,200, 100, 400))
rem = Remote()
hub.light.on(bla_dev_init_color)

try:
    config_from_parameters(rem.name())
    bla_play()
except Exception as e:
    hub.light.blink(Color.ORANGE, (100,50))
    wait(5000) # a lot, yes!
    raise

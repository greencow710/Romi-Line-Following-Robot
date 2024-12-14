''' main
    brief: main code that uses scheduler to run Romi in a circle
    details: comprised of 6 tasks, run at different priorities and frequencies,
        to update motors, print data,check imu, run a controller for the angular velocity
    authors: James Monroe and Nathan Sullivan
    date: 11/19/2024
'''


import gc
import pyb
import cotask
import task_share
from pyb import Pin, Timer, ExtInt
from EncoderDriver import Encoder
from MotorDriver import Motor
from bno055 import BNO055
from LineSensor import LineSensor
from BumpSensor import BumpSensor
from time import sleep_ms
from micropython import const

class task1_readSensors:
    ''' brief: task class for recording data, mostly or debugging purposes.
        details: when running, it takes q's for several parameters, adds them to a list so they can be printed later 
    '''
    def __init__(self,share_centroid,share_thickness,share_bumpSensorFlag):
        ''' brief: this function intializes the task, sets state
            details: 
        '''
        
        self.centroid = share_centroid
        self.thickness = share_thickness
        self.bumpSensorFlag = share_bumpSensorFlag
        
        self.S0_INIT = 0
        self.S1_ReadSensors = 1
        
        
        
        # Instantiate line sensor objects
        sensorPins = ['B13', 'A6', 'A7', 'C5', 'B1', 'C4', 'C3', 'C2']
        self.line_sensor = LineSensor(sensorPins)
        
        # Instantiate Bump Sensor Objects
        left_sensor_pin = Pin.board.PC0
        right_sensor_pin = Pin.board.PC1
        self.left_sensor = BumpSensor(left_sensor_pin)
        self.right_sensor = BumpSensor(right_sensor_pin)
        
        self.state = 1
        pass
    def run(self):
        ''' brief: used for developement, to store q's display later. not used in final program
            details: not much else
        '''
        while(True):
        
            if (self.state == self.S0_INIT):
                pass
                
            elif (self.state == self.S1_ReadSensors):
                sensorOutput = self.line_sensor.read()
                self.centroid.put(sensorOutput[0])
                self.thickness.put(sensorOutput[1])
                if self.left_sensor.is_triggered() or self.right_sensor.is_triggered():
                    self.bumpSensorFlag.put(True)
                else:
                    self.bumpSensorFlag.put(False)
                pass
                #print(self.centroid.get())
                #print(self.thickness.get())
            else:
                pass
            yield 0 
            
        
class task2_updateRightMotor:
    ''' brief: updates motor, reads velocity, runs PI controller
        details: takes prescribed velocity, reads encoder velocity,runs Kp and Ki controller, posts motor linear velocity
    '''
    def __init__(self,share_m_kp, share_m_ki, share_mode, share_step_time, share_R_mot_target_v, share_R_mot_v,share_R_mot_x ):
        ''' brief: initializes shares, sets finite state machine, constants, initializes motor and encoder.
            details: also establishes timers
        '''
        self.m_kp = share_m_kp
        self.m_ki = share_m_ki
        self.mode = share_mode
        self.step_time = share_step_time
        self.R_mot_target_v = share_R_mot_target_v
        self.R_mot_v = share_R_mot_v
        self.R_mot_x = share_R_mot_x
        
        self.S0_INIT = 0
        self.S1_Standby = 1
        self.S2_Moving = 2
        
        self.wheelRadius = 1.375
        self.wheelCircum = 2*3.1415*self.wheelRadius
        self.Enc_ticks_per_motor_rev = 1440
        
        #instantiate motor
        tim4 = Timer(4, freq=20000)
        self.mot_R = Motor(tim4, Pin.cpu.A4, Pin.cpu.B7, Pin.cpu.B6)
        self.mot_R.enable()
        
        #Instantiate Encoder
        tim_2 = Timer(2, period = 65535, prescaler = 0)
        self.ENC_Mot_R = Encoder(Pin.cpu.A0, Pin.cpu.A1, tim_2)
        
        self.state = 1
        
        self.MotorEffort = 0
        self.integralAccumulator = 0
        
        pass
    def run(self):
        ''' brief: FSM for right motor
            details: finite state machine, do nothing if in state 1. if in state 2 (mode 0), running, then take motor effort, run PI controller
            cap motoreffort output to not fry board. ALso writes linear velocity to share.
        '''
        while(True):
            
            if (self.state == self.S1_Standby):
                if self.mode.get() == 1:
                    self.state = self.S2_Moving
                    
                pass
            elif (self.state == self.S2_Moving):
                
                velocity = self.recordVelocity()
                self.R_mot_v.put(velocity)
                
                #kp
                self.MotorEffort = (self.R_mot_target_v.get()-velocity)*self.m_kp.get()
                #ki
                self.integralAccumulator += self.m_ki.get()*(self.R_mot_target_v.get()-velocity)*self.step_time.get()/1000
                
                self.MotorEffort += self.integralAccumulator
                
                if self.MotorEffort > 100:
                    self.MotorEffort = 100
                if self.MotorEffort < -100:
                    self.MotorEffort = -100   
                self.mot_R.set_duty(self.MotorEffort)
                
                if self.mode.get() == 0:
                    self.state = 1
                    self.mot_R.set_duty(0)
                    
                #print("mode",self.mode.get())
                #print("state",self.state)
                    
                pass
            else:
                pass
            yield 0      
        
    def recordVelocity(self):
        ''' brief: Subroutine that records linear velocity of motor wheel,using encoder and constants
            details: 
        '''
        self.ENC_Mot_R.update()
        linearVelocity = self.ENC_Mot_R.get_delta()/(self.step_time.get()/1000)*self.wheelCircum/self.Enc_ticks_per_motor_rev
        self.R_mot_x.put(self.R_mot_x.get()+self.ENC_Mot_R.get_delta()*self.wheelCircum/self.Enc_ticks_per_motor_rev)
        return linearVelocity
        pass  
        
            
class task3_updateLeftMotor:
    ''' brief: updates motor, reads velocity, runs PI controller
        details: takes prescribed velocity, reads encoder velocity,runs Kp and Ki controller, posts motor linear velocity
    '''
    def __init__(self,selfshare_m_kp, share_m_ki, share_mode, share_step_time, share_L_mot_target_v, share_L_mot_v,share_L_mot_x  ):
        ''' brief: initializes shares, sets finite state machine, constants, initializes motor and encoder.
            details: also establishes timers
        '''
        self.m_kp = share_m_kp
        self.m_ki = share_m_ki
        self.mode = share_mode
        self.step_time = share_step_time
        self.L_mot_target_v = share_L_mot_target_v
        self.L_mot_v = share_L_mot_v
        self.L_mot_x = share_L_mot_x
        
        self.S0_INIT = 0
        self.S1_Standby = 1
        self.S2_Moving = 2
        
        self.wheelRadius = 1.375
        self.wheelCircum = 2*3.1415*self.wheelRadius
        self.Enc_ticks_per_motor_rev = 1440
        
        #Instantiate Motor
        tim8 = Timer(8, freq=20000)
        self.mot_L= Motor(tim8, Pin.cpu.B3, Pin.cpu.C7, Pin.cpu.C6)
        self.mot_L.enable()
        
        #Instantiate Encoder
        tim_3 = Timer(3, period = 65535, prescaler = 0)
        self.ENC_Mot_L = Encoder(Pin.cpu.B4, Pin.cpu.B5, tim_3)
        
        self.state = 1
        
        self.MotorEffort = 0
        self.integralAccumulator = 0
        
        pass
    def run(self):
        ''' brief: FSM for right motor
            details: finite state machine, do nothing if in state 1. if in state 2 (mode 0), running, then take motor effort, run PI controller
            cap motoreffort output to not fry board. ALso writes linear velocity to share.
        '''
        while(True):
            
            if (self.state == self.S1_Standby):
                if self.mode.get() == 1:
                    self.state = self.S2_Moving
                    
                pass
            elif (self.state == self.S2_Moving):
                
                
                velocity = self.recordVelocity()
                self.L_mot_v.put(velocity)
                
                #kp
                self.MotorEffort = (self.L_mot_target_v.get()-velocity)*self.m_kp.get()
                #ki
                self.integralAccumulator += self.m_ki.get()*(self.L_mot_target_v.get()-velocity)*self.step_time.get()/1000
                
                self.MotorEffort += self.integralAccumulator
                
                #print("Motor Velocity:",velocity)
                #print("Sent Velocity",self.L_mot_target_v.get())
                #print("Accumulator:",self.integralAccumulator)
                #print("Motor Effort:",self.MotorEffort,"\n")
                
                if self.MotorEffort > 100:
                    self.MotorEffort = 100
                if self.MotorEffort < -100:
                    self.MotorEffort = -100   
                self.mot_L.set_duty(self.MotorEffort)
                
                
                if self.mode.get() == 0:
                    self.state = 1
                    self.mot_L.set_duty(0)
                pass
            else:
                pass
            yield 0          
    def recordVelocity(self):
        ''' brief: Subroutine that records linear velocity of motor wheel,using encoder and constants
            details: 
        '''
        self.ENC_Mot_L.update()
        linearVelocity = self.ENC_Mot_L.get_delta()/(self.step_time.get()/1000)*self.wheelCircum/self.Enc_ticks_per_motor_rev
        self.L_mot_x.put(self.L_mot_x.get()+self.ENC_Mot_L.get_delta()*self.wheelCircum/self.Enc_ticks_per_motor_rev)
        print(self.L_mot_x.get())
        return linearVelocity
        pass    
    
   
        
            


class task4_user:
    ''' brief: FSM which calculates velocity, yaw, motor speeds, and velocity and yaw PI controller
        details: 
    '''
    def __init__(self,share_m_kp, share_m_ki, share_mode, share_step_time, share_R_mot_target_v, share_L_mot_target_v, share_R_mot_v, share_L_mot_v,share_centroid,share_thickness,share_bumpSensorFlag,share_R_mot_x,share_L_mot_x):
        self.m_kp = share_m_kp
        self.m_ki = share_m_ki       
        self.mode = share_mode
        self.step_time = share_step_time
        self.R_mot_target_v = share_R_mot_target_v
        self.L_mot_target_v = share_L_mot_target_v
        self.R_mot_v = share_R_mot_v
        self.L_mot_v = share_L_mot_v
        self.centroid = share_centroid
        self.lastGoodCentroid = 0
        self.thickness = share_thickness
        self.bumpSensorFlag = share_bumpSensorFlag
        self.R_mot_x = share_R_mot_x
        self.L_mot_x = share_L_mot_x
        
        
        self.S0_Standby = 0
        self.S1_Start = 1
        self.S2_LineFollowSegmentOne = 2
        self.S3_Backup = 3
        self.S4_TurnLeft = 4
        self.S5_Forward = 5
        self.S6_TurnRight = 6
        self.S7_GetCentered = 7
        
        
        
        
        self.speed = 5
        
        self.state = self.S0_Standby
        
        self.lineThicknessThreshold = 8
        
        self.velocity = 0
        self.yaw = 0
       
        '''
        self.imu_sensor = BNO055(1)
        sleep_ms(700)
        status = self.imu_sensor.retrieve_parse_cal()
        self.imu_sensor.change_mode(const(0x0c))
        sleep_ms(10)
        if status < 243:
            while (status < 243):
                status = self.imu_sensor.retrieve_parse_cal()
                print(status)
                print("Still Calibrating")
                sleep_ms(500)
        
        sleep_ms(5000)
        self.Euler_Home = self.imu_sensor.read_Euler()
        print(self.Euler_Home)
        '''
        
        self.count=0
        self.printCount = 0
        
        self.buttonPressed = 0
        
        self.wheelbase = 5.5
        self.wheelRadius = 1.375
        
        
        self.distanceTraveled = 0
        self.angularTraveled = 0
        
        
        '''Velocity and Yaw controller gains'''
        self.v_kp = 1
        self.v_ki = .1
        self.y_kp = .5
        self.y_ki = .03
        self.v_integralAccumulator = 0
        self.y_integralAccumulator = 0
        
        self.Start_Button = Pin(Pin.cpu.C13, Pin.IN, Pin.PULL_UP)
        
        pass
    def run(self):
        while(True):
            if (self.state == self.S0_Standby):
                if self.userButtonPush():
                    self.mode.put(1)
                    self.state = self.S7_GetCentered
                
                
                
            if (self.state == self.S1_Start):
        
                self.GoForward(self.speed)
                if self.thickness.get() > self.lineThicknessThreshold:
                    self.state = self.S2_LineFollowSegmentOne
                
                pass
            elif (self.state == self.S2_LineFollowSegmentOne):
                self.FollowLine()
                if self.bumpSensorFlag.get():
                    self.state = self.S3_Backup
                    self.ResetDistanceAndAngle()
                pass
            
            elif (self.state == self.S3_Backup):
                if self.DriveDistance(self.speed,-3):
                    self.state = self.S4_TurnLeft
                    self.ResetDistanceAndAngle()
                pass
            
                    
            elif (self.state == self.S4_TurnLeft):
                if self.TurnInPlace(self.speed, -3.1415/4*1.1):
                    self.state = self.S5_Forward
                    self.ResetDistanceAndAngle()
                pass
           
            elif (self.state == self.S5_Forward):
                if self.Go(self.speed,3):
                    self.state = self.S6_TurnRight
                    self.ResetDistanceAndAngle()
                pass
           
            elif (self.state == self.S6_TurnRight):
                if self.TurnInPlace(self.speed, 3.1415/4*1.1):
                    self.state = self.S7_GetCentered
                    self.ResetDistanceAndAngle()
                    self.count = 0
                pass
            
            elif (self.state == self.S7_GetCentered):
                self.ReadVelocity()
                '''
                print("Angle", self.angularTraveled)
                print("distance",self.distanceTraveled)
                
                print("left motor",self.L_mot_x.get())
                print("Right motor",self.R_mot_x.get())
                print(" ")
                '''
            else:
                pass
        
                
                
            yield 0   
    
    
    
    
    def userButtonPush(self):
        if self.checkButton() == 1:
            self.buttonPressed = 1
        if self.buttonPressed == 1:
            return(self.pauseTicks(20))
        
        
        if self.count == 20:
            
            self.mode.put(1)
            self.count = 0
            self.buttonPressed = 1
    
    def pauseTicks(self,ticks):
        if self.count < ticks:
            self.count+=1
            return 0
        else:
            self.count = 0
            return 1
        
        
    def checkButton(self):
        button = self.Start_Button.value()
        
        if button == 0:
            return 1
        else: 
            return 0
        return 
    
    
           
    def ResetDistanceAndAngle(self):
        self.distanceTraveled = 0
        self.angularTraveled = 0
        self.count = 0
        pass
    
    def FollowLine(self):
        if self.thickness.get() < .5:
            centerOfLine = self.lastGoodCentroid
        else:
            centerOfLine = self.centroid.get()
        yawTarget = centerOfLine*5
        self.Drive(self.speed,yawTarget)
        tempMatrix1 = [centerOfLine,self.thickness.get()]
        tempMatrix2 = [self.speed,yawTarget]
        print("Sensor Data",tempMatrix1)
        print("Motor Data",tempMatrix2)
        print(" ")
        pass
    
    def GoForward(self,speed):
        self.Drive(speed,0)
        pass
    
    def Stop(self):
        self.Drive(0,0)
        pass
   
    def Drive(self, velocityTarget, yawTarget):
        self.R_mot_target_v.put((1/self.wheelRadius*velocityTarget)+self.wheelbase/2/self.wheelRadius*yawTarget)
        self.L_mot_target_v.put((1/self.wheelRadius*velocityTarget)-self.wheelbase/2/self.wheelRadius*yawTarget)
        pass
    
    def ReadVelocity(self):
        '''read velocity from motors, yaw from motors and imu'''
        self.velocity = (self.L_mot_v.get()+self.R_mot_v.get())/2
        #yawIMU = self.imu_sensor.read_ang_vel()
        self.yaw = (self.L_mot_v.get()-self.R_mot_v.get())/self.wheelbase
        #self.distanceTraveled += self.velocity*30/1000
        #self.angularTraveled += self.yaw*30/1000
        pass
    
    def Arc(self,LorR,radius):
        if LorR == "left":
            # turning arc in left direction
            self.Drive(self.speed, -1.5)
        else: 
            # turning arc in right direction
            self.Drive(self.speed, 1.5)
        pass

        
    
    def DriveDistance(self, speed, distance):
        if distance < 0:
            DirectionalSpeed = speed*-1
        else:
            DirectionalSpeed = speed
        self.GoForward(DirectionalSpeed)
        self.ReadVelocity()
        print(DirectionalSpeed)
        print(self.distanceTraveled)
        if distance > 0:
            if self.distanceTraveled > distance:
                return 1
        else:
            if self.distanceTraveled < distance:
                return 1
        
        return 0
    
    def TurnInPlace(self, speed, angleRad):
        if angleRad > 0:
            DirectionalSpeed = speed*-1
        else:
            DirectionalSpeed = speed
        self.Drive(0,DirectionalSpeed)
        self.ReadVelocity()
        print(angleRad)
        print(self.angularTraveled)
        if angleRad > 0:
            if self.angularTraveled > angleRad:
                return 1
        else:
            if self.angularTraveled < angleRad:
                return 1
        
        return 0


# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":
    print("Testing ME405 stuff in cotask.py and task_share.py\r\n   Press Ctrl-C to stop and show diagnostics.")

    # Create a share and a queue to test function and diagnostic printouts   
    share_centroid = task_share.Share('f', thread_protect=False, name="centroid")     # centroid of line sensor
    share_thickness = task_share.Share('f', thread_protect=False, name="thickness")     # centroid of line sensor
    share_bumpSensorFlag = task_share.Share('b', thread_protect=False, name="bumpSensorFlag")     # centroid of line sensor
    
    share_m_kp      = task_share.Share('f', thread_protect=False, name="m_kp")     # Proportional motor constant
    share_m_ki      = task_share.Share('f', thread_protect=False, name="m_ki")     # Integral control
    share_mode      = task_share.Share('b', thread_protect=False, name="mode")    # 0 = on hold, #1 = running. 
    share_step_time = task_share.Share('f', thread_protect=False, name="step_time")
    

    
    share_R_mot_target_v = task_share.Share('f', thread_protect=False, name="R_mot_target_v")
    share_L_mot_target_v = task_share.Share('f', thread_protect=False, name="L_mot_target_v") 
    share_R_mot_v = task_share.Share('f', thread_protect=False, name="R_mot_v") 
    share_L_mot_v = task_share.Share('f', thread_protect=False, name="L_mot_v") 
    share_R_mot_x = task_share.Share('f', thread_protect=False, name="R_mot_x") 
    share_L_mot_x = task_share.Share('f', thread_protect=False, name="L_mot_x")
    
    share_step_time.put(30) # milliseconds
    share_m_kp.put(3)
    share_m_ki.put(3)
    

    Task1 = task1_readSensors(share_centroid,share_thickness,share_bumpSensorFlag)
    Task2 = task2_updateRightMotor(share_m_kp, share_m_ki, share_mode, share_step_time, share_R_mot_target_v, share_R_mot_v ,share_R_mot_x )
    Task3 = task3_updateLeftMotor(share_m_kp, share_m_ki, share_mode, share_step_time, share_L_mot_target_v, share_L_mot_v ,share_L_mot_x)
    Task4 = task4_user(share_m_kp, share_m_ki, share_mode, share_step_time, share_R_mot_target_v, share_L_mot_target_v, share_R_mot_v, share_L_mot_v ,share_centroid,share_thickness,share_bumpSensorFlag,share_R_mot_x,share_L_mot_x )
   
   
    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for 
    # debugging and set trace to False when it's not needed
    #task_fcn = Task1.run(share0)
    task1 = cotask.Task(Task1.run,  name="Task_1", priority=0, period=10,   profile=True, trace=False)
    task2 = cotask.Task(Task2.run,  name="Task_2", priority=2, period=share_step_time.get(),  profile=True, trace=False)
    task3 = cotask.Task(Task3.run,  name="Task_3", priority=2, period=share_step_time.get(),  profile=True, trace=False)
    task4 = cotask.Task(Task4.run,  name="Task_4", priority=1, period=30,   profile=True, trace=False)
    
   
    
    cotask.task_list.append(task1)
    cotask.task_list.append(task2)
    cotask.task_list.append(task3)
    cotask.task_list.append(task4)
   

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()
    

    
    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break
    
    
    # Print a table of task data and a table of shared information data
    print('\n' + str (cotask.task_list))
    print(task_share.show_all())
    print(task1.get_trace())
    print('')
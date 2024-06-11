import numpy as np
from random import shuffle
import copy

# Ensuring a totally random seed every step
np.random.seed()

# Define states
FB = 0  # Forward/Backward state
LAT = 1  # Lateral state
ROT = 2  # Rotational state
COMBI = 3  # Combinational state

# Define modes
FWD = 0  # Forward mode
ALL = 1  # All states mode

class BezierStepper():
    def __init__(self,
                 pos=np.array([0.0, 0.0, 0.0]),
                 orn=np.array([0.0, 0.0, 0.0]),
                 StepLength=0.04,
                 LateralFraction=0.0,
                 YawRate=0.0,
                 StepVelocity=0.001,
                 ClearanceHeight=0.045,
                 PenetrationDepth=0.003,
                 episode_length=5000,
                 dt=0.01,
                 num_shuffles=2,
                 mode=FWD):
        
        # Initialize position and orientation
        self.pos = pos
        self.orn = orn
        
        # Initialize step parameters
        self.desired_StepLength = StepLength
        self.StepLength = StepLength
        self.StepLength_LIMITS = [-0.05, 0.05]
        self.LateralFraction = LateralFraction
        self.LateralFraction_LIMITS = [-np.pi / 2.0, np.pi / 2.0]
        self.YawRate = YawRate
        self.YawRate_LIMITS = [-1.0, 1.0]
        self.StepVelocity = StepVelocity
        self.StepVelocity_LIMITS = [0.1, 1.5]
        self.ClearanceHeight = ClearanceHeight
        self.ClearanceHeight_LIMITS = [0.0, 0.04]
        self.PenetrationDepth = PenetrationDepth
        self.PenetrationDepth_LIMITS = [0.0, 0.02]
        
        # Set mode and time step
        self.mode = mode
        self.dt = dt
        
        # Initialize time and episode parameters
        self.time = 0
        self.max_time = episode_length
        self.order = [FB, LAT, ROT, COMBI]
        
        # Shuffle the order of states
        for _ in range(num_shuffles):
            shuffle(self.order)
        
        # Ensure Forward/Backward is always first
        self.reshuffle()
        
        # Set current state and time per episode
        self.current_state = self.order[0]
        self.time_per_episode = int(self.max_time / len(self.order))

    def ramp_up(self):
        """Gradually increase StepLength to the desired value."""
        if self.StepLength < self.desired_StepLength:
            self.StepLength += self.desired_StepLength * self.dt

    def reshuffle(self):
        """Ensure Forward/Backward is the first state."""
        self.time = 0
        FB_index = self.order.index(FB)
        if FB_index != 0:
            self.order[FB_index], self.order[0] = self.order[0], FB

    def which_state(self):
        """Determine the current state based on the elapsed time."""
        np.random.seed()
        if self.time > self.max_time:
            self.current_state = COMBI
            self.time = 0
        else:
            index = int(self.time / self.time_per_episode)
            self.current_state = self.order[min(index, len(self.order) - 1)]

    def StateMachine(self):
        """Execute the state machine logic."""
        if self.mode == ALL:
            self.which_state()
            if self.current_state == FB:
                self.FB()
            elif self.current_state == LAT:
                self.LAT()
            elif self.current_state == ROT:
                self.ROT()
            elif self.current_state == COMBI:
                self.COMBI()
        return self.return_bezier_params()

    def return_bezier_params(self):
        """Clip parameters to their limits and return a copy."""
        self.StepLength = np.clip(self.StepLength, *self.StepLength_LIMITS)
        self.StepVelocity = np.clip(self.StepVelocity, *self.StepVelocity_LIMITS)
        self.LateralFraction = np.clip(self.LateralFraction, *self.LateralFraction_LIMITS)
        self.YawRate = np.clip(self.YawRate, *self.YawRate_LIMITS)
        self.ClearanceHeight = np.clip(self.ClearanceHeight, *self.ClearanceHeight_LIMITS)
        self.PenetrationDepth = np.clip(self.PenetrationDepth, *self.PenetrationDepth_LIMITS)
        
        return (
            copy.deepcopy(self.pos),
            copy.deepcopy(self.orn),
            copy.deepcopy(self.StepLength),
            copy.deepcopy(self.LateralFraction),
            copy.deepcopy(self.YawRate),
            copy.deepcopy(self.StepVelocity),
            copy.deepcopy(self.ClearanceHeight),
            copy.deepcopy(self.PenetrationDepth)
        )

    def FB(self):
        """Modulate StepLength and StepVelocity."""
        StepLength_DELTA = self.dt * (self.StepLength_LIMITS[1] - self.StepLength_LIMITS[0]) / 6.0
        StepVelocity_DELTA = self.dt * (self.StepVelocity_LIMITS[1] - self.StepVelocity_LIMITS[0]) / 2.0
        
        if self.StepLength < -self.StepLength_LIMITS[0] / 2.0:
            StepLength_DIRECTION = np.random.randint(-1, 3)
        elif self.StepLength > self.StepLength_LIMITS[1] / 2.0:
            StepLength_DIRECTION = np.random.randint(-2, 2)
        else:
            StepLength_DIRECTION = np.random.randint(-1, 2)
        
        StepVelocity_DIRECTION = np.random.randint(-1, 2)
        
        self.StepLength += StepLength_DIRECTION * StepLength_DELTA
        self.StepLength = np.clip(self.StepLength, *self.StepLength_LIMITS)
        self.StepVelocity += StepVelocity_DIRECTION * StepVelocity_DELTA
        self.StepVelocity = np.clip(self.StepVelocity, *self.StepVelocity_LIMITS)

    def LAT(self):
        """Modulate StepLength and LateralFraction."""
        LateralFraction_DELTA = self.dt * (self.LateralFraction_LIMITS[1] - self.LateralFraction_LIMITS[0]) / 2.0
        LateralFraction_DIRECTION = np.random.randint(-1, 2)
        
        self.LateralFraction += LateralFraction_DIRECTION * LateralFraction_DELTA
        self.LateralFraction = np.clip(self.LateralFraction, *self.LateralFraction_LIMITS)

    def ROT(self):
        """Modulate StepLength and YawRate."""
        YawRate_DELTA = (self.YawRate_LIMITS[1] - self.YawRate_LIMITS[0]) / 2.0
        YawRate_DIRECTION = np.random.randint(-1, 2)
        
        self.YawRate += YawRate_DIRECTION * YawRate_DELTA
        self.YawRate = np.clip(self.YawRate, *self.YawRate_LIMITS)

    def COMBI(self):
        """Modulate all parameters."""
        self.FB()
        self.LAT()
        self.ROT()

def clamp(val, valmin, valmax):
    """Simple clamping function, limits to [min, max]"""
    if val < valmin: return valmin
    if val > valmax: return valmax
    return val

def makeSimpleProfile(output, input, slop):
    """
    From trutlebot_teleop, adds a bit of smoothing to startup/slowdown
    """
    
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


class PID:
    """Discrete PID controller"""
    
    def __init__(self, P=1, I=1, D=1, Imax=100, Imin=100):
        
        self.P = P
        self.I = I
        self.D = D
        
        self.I_range = (Imax, Imin)
        self.I_value = 0
        
        self.empty_states()
    
    def calc(self, target, state):
        
        # append the values
        self.state_list.append(state)
        
        error = target - state

        self.P_value = error
        self.D_value = error - self.derivative()
        self.I_value = clamp(self.I_value + error, *self.I_range)
        
        out =self.P * self.P_value + self.I * self.I_value \
            + self.D * self.D_value
        
        return out
        
    def derivative(self):
        """Calculate the derivative, discretely"""
        
        if len(self.state_list) > 1:
            return self.state_list[-1] - self.state_list[-2]
        else:  # If there's only one value then the change is 0
            return 0
        
    def empty_states(self):
        self.state_list = []
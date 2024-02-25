import numpy as np

class PID():
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, gain = 1.0, dt = 0.0020):
        # Define variables
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.gain = gain
        self.dt = dt # Sampling time

        # Set state parameters
        self.integral_state = 0.0
        self.previous_error = 0.0

    def reset(self):
        self.integral_state = 0.0
        self.previous_error = 0.0

    def setParameter(self, kp = None, ki = None, kd = None):
        # Set kp value
        if not kp == None:
            self.kp = kp

        # Set ki value
        if not ki == None:
            self.ki = ki

        # Set kd value
        if not kd == None:
            self.kd = kd

    def run(self, setpoint, feedback):
        # Find error value
        error = setpoint - feedback
        print(error)

        # Calculate proportional output
        proportional = self.kp * error

        # Calculate integral output
        integral = self.ki * (self.integral_state + (error * self.dt))

        # Calculate differential output
        differential = self.kd * (error - self.previous_error) * self.dt

        # Update states
        self.integral_state = self.integral_state + (error * self.dt) # Update integral state
        self.previous_error = error

        return float(proportional + integral + differential) * self.gain
    
class ThirdOrderPlant():
    """
    constant = [1, c, b, a]
    the transfer function will be:
    1 / (s^3 + cs^2 + bs + a)
    """

    """
    Penurunan rumus:
    Y/X = 1 / (s^3 + cs^2 + bs + a)
    (s^3 + cs^2 + bs + a)Y = X
    Ys^3 + Ycs^2 + Ybs + Ya = X
    
    State space model :
    | q1_dot |   | 0    1   0  | | q1 |   | 0 |
    | q2_dot | = | 0    0   1  | | q2 | + | 0 | x(t)
    | q3_dot |   | -c   -b  -a | | q3 |   | 1 |
    
    Output function :
    y(t) = | 1  0   0 | | q1 |
                        | q2 |
                        | q3 |

    """

    def __init__(self, constant, dt = 0.002):
        self.a = constant[3]
        self.b = constant[2]
        self.c = constant[1]
        self.dt = dt

        # Initiate state
        self.q = np.zeros((3, 1))
        self.q_dot = np.zeros((3, 1))
        self.q_dot_previous = np.zeros((3, 1))
        
        # Initiate matrix
        self.state_matrix = np.array(
            [[0, 1, 0],
            [0, 0, 1],
            [-self.c, -self.b, -self.a]]
        )
        self.input_matrix = np.array(
            [[0],
            [0],
            [1]]
        )

        self.output_matrix = np.array(
            [1, 0, 0]
        )

    def resetState(self):
        # Initiate state
        self.q = np.zeros((3, 1))
        self.q_dot = np.zeros((3, 1))
        self.q_dot_previous = np.zeros((3, 1))

    def run(self, input_signal):
        # Calculate q_dot value
        self.q_dot = np.matmul(self.state_matrix, self.q) + (self.input_matrix * input_signal)

        # Integrate to calculate q
        self.q = self.q + (self.q_dot_previous * self.dt)

        # Save current state
        self.q_dot_previous = self.q_dot

        # Output function
        return float(np.matmul(self.output_matrix, self.q))


class SecondOrderPlant():
    """
    constant = [1, b, a]
    the transfer function will be:
    1 / (s^2 + bs + a)
    """

    """
    Penurunan rumus:
    Y/X = 1 / (s^2 + bs + a)
    (s^2 + bs + a)Y = X
    Ys^2 + Ybs + Ya = X
    
    State space model :
    | q1_dot |   | 1    0  | | q1 |   | 0 |
    | q2_dot | = | -b  -a  | | q2 | + | 1 | x(t)
    
    Output function :
    y(t) = | 1  0 | | q1 |
                    | q2 |

    """

    def __init__(self, constant, dt = 0.002):
        self.a = constant[2]
        self.b = constant[1]
        self.dt = dt

        # Initiate state
        self.q = np.zeros((2, 1))
        self.q_dot = np.zeros((2, 1))
        self.q_dot_previous = np.zeros((2, 1))
        
        # Initiate matrix
        self.state_matrix = np.array(
            [[0, 1],
            [-self.b, -self.a]]
        )

        self.input_matrix = np.array(
            [[0],
            [1]]
        )

        self.output_matrix = np.array(
            [1, 0]
        )

    def resetState(self):
        # Initiate state
        self.q = np.zeros((2, 1))
        self.q_dot = np.zeros((2, 1))
        self.q_dot_previous = np.zeros((2, 1))

    def run(self, input_signal):
        # Calculate q_dot value
        self.q_dot = np.matmul(self.state_matrix, self.q) + (self.input_matrix * input_signal)

        # Integrate to calculate q
        self.q = self.q + (self.q_dot_previous * self.dt)

        # Save current state
        self.q_dot_previous = self.q_dot

        # Output function
        return float(np.matmul(self.output_matrix, self.q))

class FirstOrderPlant():
    """
    constant = [1, a]
    the transfer function will be:
    1 / (s + a)
    """

    """
    Penurunan rumus:
    Y/X = 1 / (s + a)
    (s + a)Y = X
    Ys + Ya = X
    
    State space model :
    | q1_dot |   | -a | | q1 | + | 1 | x(t)
    
    Output function :
    y(t) = | 1 | | q1 |

    """

    def __init__(self, constant, dt = 0.002):
        self.a = constant[1]
        self.dt = dt

        # Initiate state
        self.q = np.zeros((1, 1))
        self.q_dot = np.zeros((1, 1))
        self.q_dot_previous = np.zeros((1, 1))
        
        # Initiate matrix
        self.state_matrix = np.array(
            [[-self.a]]
        )

        self.input_matrix = np.array(
            [[1]]
        )

        self.output_matrix = np.array(
            [1]
        )

    def resetState(self):
        # Initiate state
        self.q = np.zeros((1, 1))
        self.q_dot = np.zeros((1, 1))
        self.q_dot_previous = np.zeros((1, 1))

    def run(self, input_signal):
        # Calculate q_dot value
        self.q_dot = np.matmul(self.state_matrix, self.q) + (self.input_matrix * input_signal)

        # Integrate to calculate q
        self.q = self.q + (self.q_dot_previous * self.dt)

        # Save current state
        self.q_dot_previous = self.q_dot

        # Output function
        return float(np.matmul(self.output_matrix, self.q))




    


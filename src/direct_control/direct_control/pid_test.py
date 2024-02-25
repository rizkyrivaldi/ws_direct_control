from utils import pid
from matplotlib import pyplot as plt

# Global sampling time
dt = 0.02

# plant
plant = pid.SecondOrderPlant([1, 3, 1], dt = 0.02)
# plant = pid.FirstOrderPlant([1, 3], dt = dt)

def plant_test():
    # input units
    input_signal = []
    for i in range(50):
        input_signal.append(0)

    for i in range(1000):
        input_signal.append(1)

    # Get the output
    output_signal = []
    for i in range(1050):
        output_signal.append(plant.run(input_signal[i]))

    plt.plot(range(len(output_signal)), output_signal)
    plt.show()

def pid_test():
    # Set input_signal
    input_signal = []
    for i in range(50):
        input_signal.append(0)

    for i in range(3000):
        input_signal.append(0.5)

    controller = pid.PID(kp = 2.5, ki = 1.8, gain = 1.2, dt = dt)
    
    feedback = 0.0
    output_signal = []

    for i in range(3050):
        controller_output = controller.run(setpoint = input_signal[i], feedback = feedback)
        # print(controller_output)
        output_signal.append(plant.run(controller_output))
        feedback = output_signal[i]

    plt.plot(range(len(input_signal)), input_signal)
    plt.plot(range(len(output_signal)), output_signal)
    plt.show()
    
        
# plant_test()
pid_test()


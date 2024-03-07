import numpy as np
import pickle

class BPNN():
    def __init__(self, layers = None, weight = None, activation = "tanh", alpha = 0.1):
        """
        The layers variable is to determine the architecture of the NN
        First index indicates the number of input neurons
        The last index indicates the number of output neurons
        The inbetween index indicates the hidden layer count and number of neurons in each layer

        Layers example:
        [3, 5, 7, 2]
        3 input neurons
        2 hidden layers with 5 and 7 neurons on each layers
        2 output neurons
        """
        
        if weight == None:
            # Error checking
            ## Check if the layer is type list
            if type(layers) != list:
                raise TypeError("layer argument must be list")

            ## Check if the layer is less than 3
            elif len(layers) < 3:
                raise ValueError("The layer argument must include 3 element at minimum")

            ## Check if the layer is multidimensional and non-integer
            for element in layers:
                if isinstance(element, list):
                    raise ValueError("The size of the layer argument must be one dimension")

                elif type(element) != int:
                    raise TypeError("The element in layer argument must be integer")

        # Variable Initialization
        self.predicted = None

        # Generate random seed
        np.random.seed()

        # Generate weight if not yet initialized
        if weight == None:
            # Neural Network Initialization
            self.layers = layers
            self.layers_count = len(self.layers)
            self.input_neuron = self.layers[0]
            self.output_neuron = self.layers[-1]
            self.hidden_neuron = self.layers[1:-1]

            # Initialize weight
            self.initializeWeight()

        else:
            self.loadWeight(weight)
            self.layers_count = len(self.layers)
            self.input_neuron = self.layers[0]
            self.output_neuron = self.layers[-1]
            self.hidden_neuron = self.layers[1:-1]

        # Set alpha
        self.setAlpha(alpha)

        # Set activation function
        self.setActivation(activation)

        # Debug message
        print(f"Initialization successfull with NN architecture {self.layers}")
    
    def initializeWeight(self):
        """
        Weight randomization, using normal numpy uniform pseudo-random algorithm
        """
        # Randomize weight
        self.weight = []
        for n in range(self.layers_count - 1):
            self.weight.append(np.random.uniform(-0.5, 0.5, (self.layers[n], self.layers[n+1])))

        self.weight_bias = []
        for n in range(self.layers_count - 1):
            self.weight_bias.append(np.random.uniform(-0.5, 0.5, (self.layers[n+1])))

    def setActivation(self, activation):
        """
        Set activation function, feasible functions:
        "tanh"
        """

        # Check if the input is not string
        if type(activation) != str:
            raise TypeError(f"Unable to set activation function, argument is not valid")

        elif activation == "tanh":
            self.activation = np.tanh
            self.activation_diff = lambda x: 1 - np.tanh(x)**2

        else:
            self.activation = np.tanh
            self.activation_diff = lambda x: 1 - np.tanh(x)**2
            print("Set Activation argument is not valid, switching to tanh activation as default")

    def setAlpha(self, alpha):
        """
        Set learning rate alpha
        """
        self.alpha = alpha

    def feedForward(self, input_vector = []):
        """
        Feed forward function, accepts input vector as an argument
        Will results predicted value
        """
        # Error checking
        ## Check if the type is a list or numpy array
        if type(input_vector) != np.ndarray and type(input_vector) != list:
            raise TypeError("Feedforward input type must be list or numpy array")

        elif len(input_vector) != self.input_neuron:
            raise ValueError(f"The dimension of input vector did not match the input neuron count, expected {self.input_neuron} but get {len(input_vector)} instead")

        ## Change the datatype to numpy error if input as a list
        if type(input_vector) != np.ndarray:
            self.input_vector = np.array(input_vector)
        else:
            self.input_vector = input_vector

        # Start feedforward
        self.z_in_vector = []
        self.z_out_vector = []
        for n in range(self.layers_count - 1):
            if n == 0:
                self.z_in = np.matmul(self.input_vector, self.weight[n]) + self.weight_bias[n]
                self.z_in_vector.append(self.z_in)
                self.z_out_vector.append(self.activation(self.z_in))

            else:
                self.z_in = np.matmul(self.z_out_vector[n - 1], self.weight[n]) + self.weight_bias[n]
                self.z_in_vector.append(self.z_in)
                self.z_out_vector.append(self.activation(self.z_in))

        self.predicted = self.z_out_vector[n]

        return self.predicted

    def backPropagation(self, actual_output_vector = []):
        """
        Back Propagation function, accepts actual output vector as training references
        by using gradient descent optimizer
        """

        # Error checking
        ## Check if the type is a list or numpy array
        if type(actual_output_vector) != np.ndarray and type(actual_output_vector) != list:
            raise TypeError("Backpropagation input type must be list or numpy array")

        elif len(actual_output_vector) != self.output_neuron:
            raise ValueError(f"The dimension of input vector did not match the output neuron count, expected {self.output_neuron} but get {len(actual_output_vector)} instead")

        ## Change the datatype to numpy error if input as a list
        if type(actual_output_vector) != np.ndarray:
            self.actual_output_vector = np.array(actual_output_vector)
        else:
            self.actual_output_vector = actual_output_vector

        # Check error value
        self.error = self.actual_output_vector - self.predicted

        # Start backpropagation
        weight_delta = []
        weight_bias_delta = []

        for n in range(self.layers_count - 2, -1, -1):
            if n == self.layers_count - 2:
                do = (self.actual_output_vector - self.predicted) * (self.activation_diff(self.z_in_vector[n]))
                weight_delta.append(self.alpha * self.z_out_vector[n-1][:, None] * do)
                weight_bias_delta.append(self.alpha * do)
                prev_do = do
            
            elif n != 0:
                do_in = np.sum(self.weight[n+1] * prev_do[None, :], axis = 1)
                do = do_in * self.activation_diff(self.z_in_vector[n])
                weight_delta.append(self.alpha * self.z_out_vector[n-1][:, None] * do)
                weight_bias_delta.append(self.alpha * do)
                prev_do = do

            else:
                do_in = np.sum(self.weight[n+1] * prev_do[None, :], axis = 1)
                do = do_in * self.activation_diff(self.z_in_vector[n])
                weight_delta.append(self.alpha * self.input_vector[:, None] * do)
                weight_bias_delta.append(self.alpha * do)
                prev_do = do
            
        weight_delta.reverse()
        weight_bias_delta.reverse()

        # Update weight
        for n in range(self.layers_count - 1):
            self.weight[n] += weight_delta[n]
            self.weight_bias[n] += weight_bias_delta[n]

    def saveWeight(self, filename: str):
        with open(filename, "wb") as fp:
            pickle.dump([self.layers, self.weight, self.weight_bias], fp)

    def loadWeight(self, filename: str):
        with open(filename, "rb") as fp:
            self.layers, self.weight, self.weight_bias = pickle.load(fp)

    def train(input_vector, output_vector, epoch):
        pass


    

    


        
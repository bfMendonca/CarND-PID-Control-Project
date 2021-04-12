## PID Controller Project Writeup ##

This project contains an implementation for an PID controller, written, in C++, and a class for hyperparameter optmization using the "Twiddle" technique. Below the important parts of those will be highlighted for better understanding. 

### PID Controller ###
The PID Controller is written at the class PID, in PID.cpp and PID.h files. The paremeters choosen for implementation and that should be initialized at the constructor are:

* kp : Proportional gain
* ki : Integral gain
* kd : Derivative gain
* dt : Timestep between calls
* maxVal : Max value to be assigned as output, work as a clamp, or limiter
* maxISum : Max value for integrator sum term. To prevent Wind up
* outputFilterAlpha : An paremeter to be used for filtering the output. Should be in the [0,999] range, being 0 the value that "disable" this filtering.

Also for allowing the "tunning" of some of the parameters, the method "UpdateConstants", PID.h line 30, can be used to change the kp, ki, kd values. The usage is straight forward as the error should be update using the "UpdateError" method and, following that, the updated output of the PID controller will be available using GetOutput


### TWiddle Hyperparameter Tunning ###
For tunning the Kp, Ki and Kd constants a class denominated "TwiddleStateMachine" was written under the twiddle_machine.cpp and twiddle_machine.h files. This class countains an State Machine used for tunnign and arbitrary number of hyperparameters. As the parameters for this class, the constructors takes the following parameters:

* numberOfParams: Number of parameters to be tunned
* n: The number of iterations for each tunning cycle.
* params: an vector<double> of numberOfParams size containining the initial value for the parameters
* initDp: an vector< double> of numberOfParams size that specifies the initial variations that each parameter should suffer. 

The "API" for using this class is simple calling the method "update" which receives two arguments: error and action. These two arguments are simple the error, the same inserted into the PID controller, and action is the output of the PID controller. The second argument was implemented in order to bias the "Twiddle" to achieve the minimal possible error with the minimal possible action from the controller, thus, seeking for a solution more overdamped. The usage of the TwiddleStateMachine can be enabled at the code by uncomenting the "STEERING_ENABLE_TWIDDLE" define, at main.cpp file line 34.

After uncomenting the Twiddle is used as can be seen at main.cpp line 118, were the "update" method is called. The choose n for the optmization, hardcoded at 3800, main.cpp line 70, is such as this value represent one lap at the track, so, the Twiddle will be evaluating the performance of the controller with the given parameters after one lap. This was choosen in order that each parameter set can be compared evenly. 

Diving into the details of the "TwiddleStateMachine", there are 6 states implemented. 

* STATE_1*: This is the initialization state. This allows the code to achieve a first reference error sum with the given initialization parameters values. After this state the code goes to STATE_2.

* STATE_2*: This is were all iteration cycle of the Twiddle begins. When transiting to this state using the "goToState2" method, this will make one parameter bee summed of it's variation value, dp, twiddle_machine.cpp line 125, and start an new evaluation cycle. After summing up the error for N inputs, specified at the constructor, the code will evaluate if the error has improved or no, lines 85 to 92 at the same cpp file. If has improved then the state machine transits to the STATE_3, otherwise, STATE_4 is choosen

* STATE_3*: This is the end of an iteration that could achieve a better solution than the previous one. It will only multiply the "variation" for that paremeter by 1.1, as discribed in the classes, and get the next hyperparameter for evaluation starting the cycle again at the STATE_2. 

* STATE_4*: State that follows if an previous variation of the parameters has not improved the results. This will evaluate if the error will improve if that given parameter has decreased by "dp", instead of increasing. After N iterations then the code checks if the error has improved. If the actual results are better than the previous one, than the code goes to STATE_3, which marks the end of all iterations with positive results, otherwise the code transits to the STATE_5

* STATE_5*: This is the final step if an parameter sweep has not improved for either sides. It will multiply the variation for that given parameter by 0.9 and start the cycle again for the next hyperparameter, goint to STATE_2

* STATE_6*: This state was designed so that the Tweedle could "disable itself" when finished, but was never implemented in this code. Left for future improves. 


For the project code, after some tests, the Tweedle was set to hyperparameter search for an PD controller. The I constant of the PID seemed to make the system too instable and was left outside tunning. The values achieved after using the hyperparameter described above are at the main.cpp file, at lines 49 through 57. Those results were the best achieved from autotunning and were the output after several days of lefting the Tweedle tuning it. Sample output in "pid-output" at this repository

### Reflection ### 
For the PID Controller itself, a brief reflection for the parameters are:

* P: The proportional parameter will make the controller respond to the "actual" error. As this values increases, this should make the error decrease faster but, as a side effect, it can make the controller overshoot and make it oscillate at the reference. Increasing it can make the "steady state error" decrease

* D: The derivative term makes the controller to respond to a variation of the error. In simple terms, it serves as an "forecast" and can make the controller to decrease it's output if the error is decreasing, making the controller less susceptible for oscilations, even for larger P values. One nasty side effect for these are that spikes in the derivative, which are very common, can lead to "noisy" output. There are some implementations of PID which includes an filter for the Derivative term, to decrease the effects of this "noise" on the output, but this is not the case for this implementation. 

* I: The integral term is mainly used for reducing the "steady state error". It will make the controller slowly, hopefully, drift torward it's reference, even if the Proportional term was not able to do that. In order to decrease the steady-state error using only the P gain, one would be forced to used very large P values, but as discussed above, this could lead to oscillations and even instability for some systems. One collateral effect for the "I" term is the one called "wind-up", which is caused when the system is too far off reference and the "summation" of the integral term achieve very high values. This can lead the system to a "low frequency" oscilation for itself. In order to prevent this "wind-up", some techniques can be used as limitting the maximum output for the Sum or, only enabling the "Integral term" once the system has a low error, so it's kick in only for steady steate correction. 

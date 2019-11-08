# Passenger Car Kinematic Model

This package contains a functional kinematic bicycle model for a passenger car. The model is based primarily off the model presented in [1] with some tweaks based off concepts presented in [2] and [3]

## Parameters

* length\_to\_f: Distance from front wheel to CG in m
* length\_to\_r: Distance from rear wheel to CG in m
* unloaded\_wheel\_radius_f: Unloaded radius of front tire in m
* unloaded\_wheel\_radius_r: Unloaded radius of rear tire in m
* loaded\_wheel\_radius_f: Loaded radius of front tire in m
* loaded\_wheel\_radius_r: Loaded radius of rear tire in m
* speed\_kP: The proportional value used to convert error in current speed into acceleration
* acceleration\_limit: The maximum possible acceleration of the vehicle (m/s^2)
* deceleration\_limit: The maximum possible deceleration of the vehicle (m/s^2)
* hard\_braking\_threshold: he speed error required for the controller to enter a hard braking state where is forces maximum deceleration until near the setpoint (m/s)

## References
1. P. a. A. F. a. N. B. a. d. L. F. A. Polack, "The Kinematic Bicycle Model: a Consistent Model for Planning Feasible Trajectories for Autonomous Vehicles?," in 2017 IEEE Intelligent Vehicles Symposium (IV), Los Angeles, CA, USA, 2017.
2. R. N. Jazar, Vehicle dynamics: theory and application, Springer, 2017. 
3. R. A. L. a. H. M. Pepy, "Path planning using a dynamic vehicle model.," in International Conference on Information & Communication Technologies Vol 1., 2006.
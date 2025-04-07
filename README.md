# HUMAN BODY MODEL

This project provides a human body model that can be used to calculate the effective mass for arbitrary body positions and body locations. The effective mass is an important input value if a sptially unconstrained human-robot collision needs to be simulated or if results from collision measurement needs to projected on a contact situation under spatially unconstrained conditions [1].

[1] Herbster, Sebastian, Roland Behrens, and Norbert Elkmann. “A New Conversion Method to Evaluate the Hazard Potential of Collaborative Robots in Free Collisions.” Experimental Robotics, 2021, 222–32. https://doi.org/10.1007/978-3-030-71151-1_20.

# SETUP

Run the script `setup_clsHuman.py`. It will then automatically intall all required packages on your system.

# USAGE

Execute the following steps to calculate the effective mass of body location:

1. Initialize the human body model with `clsHM.clsHumanModel("bodyParams.xml", gender, weight, height)`. The argument `gender` (string) can be either "male" or "female". The argument `weight` (real, >0) set the weight  and `height` (real, >0) sets the height of the human body. The first argument value "bodyParams.xml" should not be changed.

2. [optional] Initialize the human posture model with `clsHMU.clsHMBodyPoseReader( "posture.xml" )`. The first argument value "posture.xml" refers to an XML file (part of the project) that includes predefined body postures. Note this step is only necessary if the joint configuration `q` is required for predefined body postures. The method `mybodyposture.getPostureByName( bodyPosture )` of the object created from class `clsHMBodyPoseReader` returns the joint configuration `q` (i.e., a vector of joint angles) for the body posture given by argument `bodyPosture` (string). The name of the body posture must match the name of a body posture that is defined in the XML file.  

3. [optional] Initialize the human body location model with `clsHMU.clsHMBodyLocationReader( "body.xml" )`. The first argument value "body.xml" refers to an XML file (part of the project) that includes the position of the body locations listed in ISO/TS 15066. Note this step is only necessary if the exact positions of the body locations listed in ISO/TS 15066 are needed. The method `GetBodyLocationById( id )` of the object created from class `clsHMBodyLocationReader` returns an object (e.g., `mybodyloc`) that includes the description of the body location with ID give by `id`. The IDs used in "body.xml" are exactly the same as used for the body locations listed in ISO/TS 15066. The object returned (e.g., `mybodyloc`) and the joint configuration `q` can then be used as input for method `GetBodyPartPosition(mybodyloc, q)` of the object created from class `clsHumanModel`. The method will return the output arguments `pn`, `pi`, and `frameid` while `pn` is the position of the body locations w.r.t. the world frame, `pi` is the position of the body location w.r.t. the local frame (to which `frameid` refers), and `frameid` the I

4. [optional] The method `PlotModel(q, u, pi, frameid)` of the object created from the class `clsHumanModel` can be used to visualize the human body model. The visualization will show the body posture given by the joint configuration `q`, the direction of the collision `u` (i.e., a normalized vector of 3x1 size) and the position of the body location `pi` w.r.t. the frame with ID `frameid`.

5. The method `EffectiveMass(q, u, pi, frameid)` of the object created from the class `clsHumanModel` can be used to visualize the human body model. The input arguments are the same as for the visualization method (see previous step). The method will return two arguments. The first is the effective mass of the body locations specified by `pi` and `frameid`. The second can include further values for all body part lie in the line formed by the collision vector `u`. This is specifically important if the body part with the body locations selected lies against other body parts (e.g., the arm lies against the trunk). If the body locations selected is not obstructed by other body parts, the second argument will be empty. 

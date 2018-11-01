<!-- do not modify - autogenerated -->
 
# AI Driving Olympics

<a href="http://aido.duckietown.org"><img width="200" src="https://www.duckietown.org/wp-content/uploads/2018/07/AIDO-768x512.png"/></a>


## "ROS-based Lane Following" for challenge `aido1_LF1-v3`

This is a template for one of the challenges in the [the AI Driving Olympics](http://aido.duckietown.org/).

The [online description of this challenge is here][online].

For submitting, please follow [the instructions available in the book][book].
 
[book]: http://docs.duckietown.org/DT18/AIDO/out/

[online]: https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1-v3

## Description

This is a simple template for an ROS agent that does Lane Following with the Duckietown stack.

[This code is documented here](https://docs.duckietown.org/DT18/AIDO/out/ros_baseline.html).

## Running Locally

To run locally:

1. `pip2 install --user -r requirements.txt`
2. `pip2 install --user -e git://github.com/duckietown/gym-duckietown.git#egg=gym-duckietown`
3. `python2 local_experiment.py`

If you are looking to use the old Duckietown stack (as is the case, running it out of the box), you will need to follow these steps in addition to 1 and 2 above:

2a. Clone this [repository](https://github.com/duckietown/Software)
2b. `cd Software/catkin_ws && catkin_make`
2c. `source devel/setup.bash`

And then, you can run step 3.

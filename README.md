# reinforcement-_learning_racing_quad
This repository is an implmentation of a reinforcement algorithm which can teach itself to pass throuhg gates. The implmentation uses python, keras and the flightgoggles simulation environment.

## Dependencies
In order to run this project you will have to install the follwing respositories
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
* [Keras](https://keras.io/#installation)
* [FlightGoggles](https://github.com/mit-fast/FlightGoggles/wiki/installation-local)

## Instructions
### Training and testing the supervised learning neural network
Note: The training data is stored as rosbag files in the training data folder directory
* Run the 'extract_rosbag.py' file to extract the rosbag information and store it as a pickle file
* Run 'train_nn.py' file to train the supervised neural neural network. This will save the trained model to oa file.
* Start the flightgoggles simulator by running the 'start_nn.sh' file
* Test the supervised learning model bu running the 'supervised_rate_thrust_publisher.py' file.

### Training and testing the supervised learning neural network
* Run the 'dqn.py' file to iteratively train the supervised model using reinforcement learning
* Start the flightgoggles simulator by running the 'start_nn.sh' file
* Test the supervised learning model bu running the 'reinforcement_learning_rate_thrust_publisher.py' file.
 


# Human Activity Recognition for Roboy

There are two primary ways to run and use the application: with Ravestate dialog as Charades game, and without the dialog (see below).  
The implementation requires GPU.

## Continuous human activity recognition without the Ravestate dialog

### Description

This version of the implementation runs continuously in the following way: 

1. records 3 seconds of an action
2. processes and outputs the prediction
3. goes to step 1.  

voicing of predictions uses /roboy/cognition/speech/synthesis/talk service of Roboy.  
assumes ROS_MASTER is at http://192.168.0.105:11311/    

### Installation and Running

1. clone this repository

2. run 
```make build```
to build the container 

3. to run the container
```make run```  
When running, container starts the Human Activity Recognition server.  
Server outputs "STARTED<<<<<<<<<<<<<<<<<<<
ready to recognize" on the standard output when it's ready

4. To run the application w/out the dialog, run
```make pipeline```

5. to connect to the container in case you want to manually run any of the components or kill the server node, run 
```make connect```

6. running a container starts a GUI at http://localhost:8000/

7. if the camera that you want to use is not /dev/video0, then you can change this line ```--device=/dev/video0:/dev/video0 \``` in Makefile.


## Charades with Ravestate dialog

### Description:

This version of the implementation is a Charades game with natural language interface. To initiate playing Charades with Roboy, say "charades" or "play."

### Editing the configuration file for Ravestate

1. edit the charades.yml in ravestate/config/:
	* enter your password to neo4j
	* if you have access to Roboy and his speech services, add ravestate_roboyio to the core config; otherwise, use ravestate_conio to communicate on console
	* if you have access to Roboy and his face/emotion service, change 'emotions: False' to 'emotions: True'
2. now run (in ravestate directory):
```python3 -m ravestate -f config/charades.yml```

3. ravestate dialog controls the game flow and does the calls to the recognition component

* NOTE: To use Ravestate with ROS without ravestate_roboyio, but using ravestate_conio, currently the following workaround is needed:
file ravestate/modules/ravestate_ros1/ros1_properties.py line 54 
```rospy.init_node(node_name, disable_signals=True)```
should be uncommented.  
Until this is fixed in Ravestate, this should be done manually to run charades with console io.  
This is the [link](https://github.com/Roboy/ravestate/issues/152) to the issue. Check it to know if the issue was fixed.

### Installation and Running

1. clone this repository

2. run
```
./compose.sh up
```
This command builds and runs this container and neo4j container, which is required to run Ravestate.  
Human activity recognition server will start upon the start of the container.

3. to  start the dialog, run
```
make ravestate
```
4. to connect to the container in case you want to manually run any of the components or edit the config file, run 
```make connect```

5. running a container starts a GUI at http://localhost:8000/

6. if the camera that you want to use is not /dev/video0, then you can change this line ```--device=/dev/video0:/dev/video0 \``` in the compose.sh script.

7. to stop the containers
```
./compose.sh stop
```
8. to stop and delete the containers
```
./compose.sh down
```  

* NOTE: compose.sh script is used instead of docker-compose due to absence of support of GPU in the current compose version.   
[Issue Tracker](https://github.com/docker/compose/issues/6691) - once the support is in place, normal docker-compose can be used instead 


# Acknoweldgements

This work uses the pre-trained I3D model of [Joao Carreira and Andrew Zisserman](https://github.com/deepmind/kinetics-i3d).  
Their code from the above repository was used for i3d.py and predictor.py






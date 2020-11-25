# GaitDynamicsIMU
 

Contents:

1. ShankMotionCal: 
    - files are used to estimate shank motion from IMU raw outputs

2. EXPData_Process: 
    - files are used to do inverse dynamics with motion data from VICON systems
    - requires the results from the folder "ShankMotionCal"
    - results together with shank motion are stored together estimated shank motion

3. ContactIdentify: 
    - files are used to identify foot-ground contact parameters
    - requires the results from the folder "EXPData_Process"

4. TrackShank
    - files are used to identify foot-ground contact parameters
    - requires the results from the folder "EXPData_Process" and "ContactIdentify"



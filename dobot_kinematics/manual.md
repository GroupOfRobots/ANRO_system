### **Install**  
sudo pip3 install pybullet  
sudo apt install python3-pykdl  

### **PyKDL**
* http://docs.ros.org/en/diamondback/api/kdl/html/python/kinematic_chains.html  
* http://wiki.icub.org/wiki/KDL-simple
* Kinematic Chain https://www.orocos.org/book/export/html/800.html


### **PyBullet Quickstart Guide**  
https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html?fbclid=IwAR1nc_f9d5cFmVUpwV7hbaQoRze5bFiu9VAAk7kpkW3aH8enho1wgDZp8fw

### **Manual**  
https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?fbclid=IwAR0xoPAwDT2UmnWPVPSZTJBtVy1562FuemTGgpav1Ac2gD0SAOBMONJwy_A#

### **Collision utils source:**
* https://github.com/adamheins/pyb_utils/blob/main/pyb_utils/collision.py  
* https://adamheins.com/blog/collision-detection-pybullet?fbclid=IwAR1P69kYmPCN_yQEQ8ZqTSciODNjY43dEZepNyT2dC3LUL9Wggk1PrqLhz0


### **Use case - predicted collision between gripper and robot base**
TARGET: x, y, z, r = 65.1431884765625, -79.33975219726562, -2.8165130615234375, 16.598087310791016  
**Response:** [PTP_server-1] [WARN] [1668080416.844542343] [dobot_PTP_server]: Goal rejected: dobot_msgs.srv.EvaluatePTPTrajectory_Response(is_valid=False, message='A collision was detected during trajectory validation. The movement cannot be executed.')


### **Use case - predicted violation of joint limits while executing linear trajectory**  
START: x, y, z, r =  -6.363799095153809, -127.4368133544922, 78.73419189453125, 0  
TARGET: x, y, z, r = 200.0, 0.0, 100.0, 0.0    
MOTION_TYPE: uint8 MOTION_TYPE_MOVL_XYZ = 2    
**Response:** [PTP_server-1] [WARN] [1668081940.281544573] [dobot_PTP_server]: Goal rejected: dobot_msgs.srv.EvaluatePTPTrajectory_Response(is_valid=False, message='Joint limits violated')

### **use_ground_collision_detection - explaination**  
When newcomers start working with Dobot Magician, it is common for them to bump the end effector on the table. For example, students, while working with the robot and the attached camera, may make a mistake in interpreting the data about the position of the object to be moved as a result of which the end effector will come into collision with the table top. For this reason, a parameter _**use_ground_collision_detection**_ has been added. When its value is True, the physics simulator checks whether a collision with the ground will occur during movement. If it does, the movement to the collision point will be **rejected by the action server**. 

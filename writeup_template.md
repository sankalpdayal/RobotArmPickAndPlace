## Project: Kinematics Pick & Place

---


**I followed following steps to complete the project:**  


1. Set up your ROS Workspace.
2. Cloned the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experimented with the forward_kinematics environment and got familiar with the robot.
4. Launched in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Performed Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Filled in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./Image1.JPG
[image2]: ./Image2.JPG
[image3]: ./Image3.JPG
[image4]: ./Image4.JPG
[image5]: ./Image5.JPG
[image6]: ./Image6.JPG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  

You're reading it!

### Kinematic Analysis
#### 1. DH parameter assignment table

I ran the forward_kinematics demo and evaluated the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot. I used the diagrams in the project videos and checked them against the values in kr210.urdf.xacro file
to derive its DH parameters. The diagrams I used are as follows

DH Parameter assignment

![alt text][image4]

Link information available in urdf file.

![alt text][image5]

Using the information from urdf file and DH parameter assignment following table is obtained.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 |0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 2. Homogeneous transformation matrices creation

Using the DH parameter table, individual transformation matrices about each joint are created in the code using `sympy`. To this I implemented following code in `IK_server.py`. Method to create a homogeneous transformation matrix

```python
def create_transformation_matrix(alpha, a, d, q):  
trans_mat = Matrix([[ 				cos(q),				-sin(q),			0,			    a],  
					[ 	 sin(q)*cos(alpha),	  cos(q)*cos(alpha),  -sin(alpha),	-sin(alpha)*d],  
					[ 	 sin(q)*sin(alpha),	  cos(q)*sin(alpha),   cos(alpha),	 cos(alpha)*d],  
					[ 					 0,					  0,			0,			    1]])  
return trans_mat
```
	
Code responsible for creation of different homogeneous transformation matrices between different links

```python
T0_1 = create_transformation_matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = create_transformation_matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = create_transformation_matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = create_transformation_matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = create_transformation_matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = create_transformation_matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = create_transformation_matrix(alpha6, a6, d7, q7).subs(DH_Table)
```

In addition, I multiplied all the individual homogeneous transformation matrices to get a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose. Code responsible for this is as follows

```python
T0_EE =  T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE`
```

In order to compensate for the difference between URDF file convecntion and DH convention which I followed, I created a transformation matrix that has to be applied to EE. This is done by rotating around Z by 180 degrees and then around y by -90 degrees.
Code responsible for creation of this matrix is 

```python
rot_mat_error = rot_mat_z.subs(yw, radians(180)) * rot_mat_y.subs(ph, radians(-90))`
```

#### 3. Solving Inverse Kinematics problem

To solve Inverse Kinematics problem, as suggested in lectures, I decoupled it into Inverse Position Kinematics and inverse Orientation Kinematics; doing so I derived the equations to calculate all individual joint angles. 

##### Inverse Position Kinematics:

First step was to obtain the position WC (O4,O5, O6). This is obtained by subtracting the vector of link from WC to EE from position of EE. the vector of link from WC to EE is 0.33 mutliplied by the unit vector in Z direction. 
 
##### Inverse Orientation Kinematics :

Since now we know the position WC, the angle theta1 can be obtained by simlpy taking tan inverse of position y by position X. 

Also using the position WC, angle phi can be obtained by following the geometry. A triangle can be formed with side between WC and O2 as hypotenuse. Since we know lengths d1 and a1 and position WC, the length of sides of this trainagle can be obtianed. In the side view 1, labelled angle phi can be obtained using the ratio of these sides.

![alt text][image1]

A triangle can be formed between WC, O2 and O3. Using the position WC and lengths a1, d1, a2, a3 and d4 the length of sides of this triangle can be obtained. Further using the cosine law, the angle a, angle between sides WC and O2 and  WC and O3 can be obtained. Now using the angle phi obtained in previous step and angle a, theta 2 can be obtained. Please 
refer Side View 2 for this derivation.

![alt text][image2]

To get theta 3, angle b, angle between sides WC and O3 and O3 and O2 can be obtained using cosine law. This angle can be used to get theta 3. There is a constant correction that needs to be done because of sag in the link. This angle called angle psi can be obtained using length a3 and d4. Please 
refer Side View 3 for this derivation.

![alt text][image3]

Using theta 1,2 and 3 rotation matrix can be obtained between link between 0 to 3. Since rotation between link 0 to EE is known, inverse of rotation matrix can be applied to get rotation matrix between link 4 to 6. From this result To get the angles theta 4, 5 and 6 can be obtained.

### Project Implementation

The implementation for calculating Inverse Kinematics based on previously performed Kinematic Analysis was first done in  done in `IK_debug.py`. Once the code resulted in successfully having almost zero error for all 3 test cases, the same code was ported to `IK_server.py` file. Code was properly commented for ease of understanding. Implementation made sure that most of symbolic math
 and constant calculations was done out of the for loop. My implementation was as follows.

#####  Out of FOR loop:

1. Created symbols for lengths, offset and twist angles to be used in DH table.
2. Created symbols for joining angles
3. Created DH table using symbols and values.
4. Using the DH table, created individual homogenous transformation matrices and final rotation matrix from the homogenous transformation matrices
5. Created symbolic rotation matrices using roll, pitch and yaw.
6. Created symbolic rotation matrix for EE.
7. Obtained rotation matrix to compenstate for error between DH convection and urdf file and applied to symbolic rotation matrix for EE
8. Obtained constants like angle psi and length of side A and side C.

##### Inside FOR loop:

1. Obtained rotation matrix using the values of roll, pitch and yaw. 
2. Obtained the position of WC using the position EE and vector length of link between EE and WC. Unit vector in Z direction is obtained using last column of rotation matrix obtained in previous step.
3. Obtained theta 1, theta 2 and theta 3 using the position WC and link lenghts and using the cosine law.
4. Obtained rotation matrix between link 0 and link 3 using theta 1, theta 2 and theta 3.
5. Obtained inverse of this matrix by taking a transpose.
6. Using the inverse of rotation matrix between link 0 and link 3 and rotation matrix using the values of roll, pitch and yaw; obtained rotation matrix between link 3 and link 6.
7. Obtained theta4, theta5, theta6 by using the math to get euler angles from rotation matrix.

The code was able to guide the robot to successfully complete multiple pick and place cycles. 

Before the implementation deriving correct inverse kinematics equation was very important. I spent majority of time of my project getting this right. During the implementation I found using the debug file very helpful. 
Since the implementation had many mathematical elements it was critical to get them right and using the debug file I was able to identify bugs in the impelementation. Also I found the taking a transpose instead of inverse 
was better as it is more numerically stable. Lastly I made sure that only the parameters that need to be computed every time should be inside the for loop so that unnecessary computations are not done again and again.

Following image shows the inital and final position of the robot for different pick positions.

![alt text][image6]


Following video captures the run from start to stop for a successful pick and drop.

[![Results of Robot Run](http://img.youtube.com/vi/CzpICfFCbXw/0.jpg)](http://www.youtube.com/watch?v=CzpICfFCbXw)
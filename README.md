# cvchallenge_task2_jms
This package provides a very simple baseline solution to Task 2 of the catvehicle challenge.

# Install
Create or go into your catvehicle_ws, and make sure you have cloned the catvehicle and obstaclestopper packages into this workspace. 
```
cd catvehicle_ws/src
git clone https://github.com/sprinkjm/catvehicle.git
git clone https://github.com/sprinkjm/obstaclestopper.git
git clone https://github.com/sprinkjm/cvchallenge_task2.git
cd ..
catkin_make
```
Now you're ready to run 

# Running
```
cd catvehicle_ws
source devel/setup.bash
```
Run each of the following in its own tab (don't forget to source devel/setup.bash for each):
```
roslaunch cvchallenge_task2 catvehicle_custom.launch worldfile:=world1.world
rosrun rviz rviz
rosrun cvchallenge_task2_jms cvchallenge_task2_jms_node
```

Then monitor the /detections topic either from the cmd line or in rviz.

## License

Copyright (c) 2016-2017 Arizona Board of Regents All rights reserved

Permission is hereby granted, without written agreement and without license or royalty fees, to use, copy, modify, and distribute this software and its documentation for any purpose, provided that the above copyright notice and the following two paragraphs appear in all copies of this software.

IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

## Authors

* Jonathan Sprinkle (sprinkjm@email.arizona.edu)

## Support

This work was supported by the project "Cyber-Physical Systems Virtual Organization: Active Resources" NSF 1521617. Any opinions, findings, and conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the National Science Foundation. Additional support for the CAT Vehicle Challenge is provided by MathWorks.

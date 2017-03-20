import ~/catkin_ws/src/labtwo/experiment_images/test.png &
roslaunch dead_reckoning.launch xPos:=2.0 yPos:=2.0 angle:=1.57 kp:=0.5 kb:=-0.5 ka:=10
xclip -selection clipboard -t image/png -i experiment_images/test.png

# import ~/catkin_ws/src/labtwo/experiment_images/2.png &
# roslaunch dead_reckoning.launch xPos:=1.0 yPos:=1.0 angle:=1.57 kp:=0.5 kb:=-0.5 ka:=10 
# import ~/catkin_ws/src/labtwo/experiment_images/3.png &
# roslaunch dead_reckoning.launch xPos:=1.0 yPos:=1.0 angle:=1.57 kp:=0.5 kb:=-10 ka:=1.5 
# import ~/catkin_ws/src/labtwo/experiment_images/4.png &
# roslaunch dead_reckoning.launch xPos:=1.0 yPos:=1.0 angle:=1.57 kp:=10 kb:=-0.5 ka:=1.5 
# import ~/catkin_ws/src/labtwo/experiment_images/5.png &
# roslaunch dead_reckoning.launch xPos:=-1.0 yPos:=1.0 angle:=-3.14 kp:=0.5 kb:=-0.5 ka:=1.5 
# import ~/catkin_ws/src/labtwo/experiment_images/6.png &
# roslaunch dead_reckoning.launch xPos:=-1.0 yPos:=1.0 angle:=-3.14 kp:=0.5 kb:=-0.5 ka:=10 
# import ~/catkin_ws/src/labtwo/experiment_images/7.png &
# roslaunch dead_reckoning.launch xPos:=-1.0 yPos:=1.0 angle:=-3.14 kp:=0.5 kb:=-10 ka:=1.5 
# import ~/catkin_ws/src/labtwo/experiment_images/8.png &
# roslaunch dead_reckoning.launch xPos:=-1.0 yPos:=1.0 angle:=-3.14 kp:=10 kb:=-0.5 ka:=1.5 
# import ~/catkin_ws/src/labtwo/experiment_images/9.png &
# roslaunch dead_reckoning.launch xPos:=-1.0 yPos:=-1.0 angle:=-1.57 kp:=0.5 kb:=-0.5 ka:=1.5 
# import ~/catkin_ws/src/labtwo/experiment_images/10.png &
# roslaunch dead_reckoning.launch xPos:=-1.0 yPos:=-1.0 angle:=-1.57 kp:=0.5 kb:=-0.5 ka:=10 
# import ~/catkin_ws/src/labtwo/experiment_images/11.png &
# roslaunch dead_reckoning.launch xPos:=-1.0 yPos:=-1.0 angle:=-1.57 kp:=0.5 kb:=-10 ka:=1.5 
# import ~/catkin_ws/src/labtwo/experiment_images/11.png &
# roslaunch dead_reckoning.launch xPos:=-1.0 yPos:=-1.0 angle:=-1.57 kp:=10 kb:=-0.5 ka:=1.5
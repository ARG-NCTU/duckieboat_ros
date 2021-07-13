#!/bin/bash



if [ ! "$1" ]; then
    echo "commit detail please"
    return
fi
echo "commit: $1"

COMMIT=$1
BRANCH=master

if [ ! -z "$2" ]; then
    echo "operator on branch: $2"
    BRANCH=$2
fi

source git_pull.sh $BRANCH
PULLSTAT=$?
if [ "$PULLSTAT" -gt 0 ] ; then
   echo "There is conflict. Aborting"
   cd ~/duckiepond-nctu/
   return
fi
echo "-------------------------pull success----------------------------------"

# push xbee_communication
echo "-----------------------------------------------------------------------"
echo "-------------------------push xbee_communication-----------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/communication/xbee_communication/
git add -A
git commit -m "$1 on xbee_communication"
git push

# push lora_communication
echo "-----------------------------------------------------------------------"
echo "-------------------------push lora_communication-----------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/communication/lora_communication/
git add -A
git commit -m "$1 on lora_communication"
git push

# push seadrone_base
echo "-----------------------------------------------------------------------"
echo "---------------------------pull seadrone_base--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/seadrone_base/
git add -A
git commit -m "$1 on seadrone_base"
git push

# push maritime_gazebo
echo "-----------------------------------------------------------------------"
echo "-------------------------push maritime_gazebo--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/maritime_gazebo/
git add -A
git commit -m "$1 on maritime_gazebo"
git push

# push moos-ros-bridge
echo "-----------------------------------------------------------------------"
echo "-------------------------push moos-ros-bridge--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/moos-ros-bridge/
git add -A
git commit -m "$1 on moos-ros-bridge"
git push

# push bridge
echo "-----------------------------------------------------------------------"
echo "-------------------------push bridge-----------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/bridge/
git add -A
git commit -m "$1 on bridge"
git push

# push mqtt_bridge
echo "-----------------------------------------------------------------------"
echo "-------------------------push mqtt_bridge------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/mqtt_bridge/
git add -A
git commit -m "$1 on mqtt_bridge"
git push

# push hrvo
echo "-----------------------------------------------------------------------"
echo "-------------------------push hrvo-------------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/hrvo/
git add -A
git commit -m "$1 on hrvo"
git push

# push duckieboat_ros
echo "-----------------------------------------------------------------------"
echo "-------------------------push duckieboat_ros---------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/duckieboat_ros/
git add -A
git commit -m "$1 on duckieboat_ros"
git push

# push wamv_ros
echo "-----------------------------------------------------------------------"
echo "-------------------------push wamv_ros---------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/wamv_ros/
git add -A
git commit -m "$1 on wamv_ros"
git push

# push moos-ivp-taiwanMoos
echo "-----------------------------------------------------------------------"
echo "-------------------------push moos-ivp-taiwanMoos----------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/moos-ivp-taiwanMoos/
git add -A
git commit -m "$1 on moos-ivp-taiwanMoos"
git push

# push moos-ivp
echo "-----------------------------------------------------------------------"
echo "-------------------------push moos-ivp---------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/moos-ivp/
git add -A
git commit -m "$1 on moos-ivp"
git push

# push duckiepond-nctu
echo "-----------------------------------------------------------------------"
echo "-------------------------push duckiepond-nctu--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/
git add -A
git commit -m "$1"
git push 

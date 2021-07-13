#! /bin/bash

# echo "password: $2"
BRANCH=master
if [ ! -z "$1" ]; then
    echo "pull branch: $1"
    BRANCH=$1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull duckiepond-nctu--------------------------"
echo "-----------------------------------------------------------------------"
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in duckiepond-nctu. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull moos-ivp---------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/moos-ivp/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in moos-ivp-taiwanMoos. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull moos-ivp-taiwanMoos----------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/moos-ivp-taiwanMoos/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in moos-ivp-taiwanMoos. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull wamv_ros---------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/wamv_ros/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in wamv_ros. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull duckieboat_ros---------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/duckieboat_ros/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in duckieboat_ros. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull moos-ros-bridge--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/moos-ros-bridge/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in moos-ros-bridge. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull bridge-----------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/bridge/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in bridge. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull mqtt_bridge------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/mqtt_bridge/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in mqtt_bridge. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull hrvo-------------------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/hrvo/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in hrvo. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull maritime_gazebo--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/maritime_gazebo/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in hrvo. Aborting"
   return 1
fi


echo "-----------------------------------------------------------------------"
echo "---------------------------pull seadrone_base--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/seadrone_base/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in seadrone_base. Aborting"
   return 1
fi


echo "-----------------------------------------------------------------------"
echo "-------------------------pull lora_communication-----------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/communication/lora_communication/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in lora_communication. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull xbee_communication-----------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/communication/xbee_communication/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in seadrone_base. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "---------------------------pull vision_opencv--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/seadrone_base/sensors/vision_opencv
git checkout melodic
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in vision_opencv. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "---------------------------pull apriltags_ros--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/seadrone_base/sensors/apriltags_ros
git checkout indigo-devel
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in apriltags_ros. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "---------------------------pull realsense-ros--------------------------"
echo "-----------------------------------------------------------------------"
cd ~/duckiepond-nctu/catkin_ws/src/seadrone_base/sensors/realsense-ros
git checkout 2.2.15
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in realsense-ros. Aborting"
   return 1
fi

cd ~/duckiepond-nctu/
return 0
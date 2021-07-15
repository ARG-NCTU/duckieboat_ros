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

# push low_cost_sensors
echo "-----------------------------------------------------------------------"
echo "-------------------------push low_cost_sensors-------------------------"
echo "-----------------------------------------------------------------------"
cd ./low_cost_sensors/
git add -A
git commit -m "$1 on low_cost_sensors"
git push

# push duckieboat_ros
echo "-----------------------------------------------------------------------"
echo "-------------------------push duckieboat_ros---------------------------"
echo "-----------------------------------------------------------------------"
cd ../
git add -A
git commit -m "$1"
git push 

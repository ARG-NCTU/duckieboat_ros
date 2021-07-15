#! /bin/bash

# echo "password: $2"
BRANCH=master
if [ ! -z "$1" ]; then
    echo "pull branch: $1"
    BRANCH=$1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull duckieboat_ros---------------------------"
echo "-----------------------------------------------------------------------"
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in duckieboat_ros. Aborting"
   return 1
fi

echo "-----------------------------------------------------------------------"
echo "-------------------------pull low_cost_sensors-------------------------"
echo "-----------------------------------------------------------------------"
cd ./low_cost_sensors/
git checkout $BRANCH
git pull

CONFLICTS=$(git ls-files -u | wc -l)
if [ "$CONFLICTS" -gt 0 ] ; then
   echo "There is conflict in low_cost_sensors. Aborting"
   return 1
fi


cd ../
return 0
#!/bin/bash
echo "Current location is: $(pwd)"
while true; do
    read -p "Do you want to delete all img data? y or n:" yn
    case $yn in
        [Yy]* ) echo "Clearing ..."; rm -rf DepthIMG/*; rm -rf LidarIMG/*; rm -rf TrainingIMG/*; echo "Finish"; break;;
        [Nn]* ) exit;;
        * ) echo "Please answer y or n.";;
    esac
done

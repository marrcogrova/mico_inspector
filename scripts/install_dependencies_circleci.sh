#!/bin/sh

mkdir build
cd build

install_git_repo () {

	if [ -d "./$1" ] 
	then
		echo "Library $1 already installed" 
	else
		git clone $2
		cd $1
		if [ -z "$3" ]   # Is parameter #1 zero length?
		then
			git checkout "$3"
		fi
		
		mkdir build ; cd build
		cmake ..
		make -j$(nproc)
		sudo make install 
		cd ../..
	fi
}

sudo apt-get update && sudo apt-get install -y cmake

#install ROS to install OPENCV AND PCL
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full

sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install -y ros-kinetic-opencv3


###################################################################
###########				INSTALL SLAM DEPS				###########
###################################################################
sudo apt-get install -y liblapack-dev libblas-dev
sudo apt-get install -y libomp-dev libsuitesparse-dev libcholmod3

install_git_repo "eigen-git-mirror" "https://github.com/eigenteam/eigen-git-mirror" "3.3.7"

install_git_repo "DBoW2" "https://github.com/dorian3d/DBoW2"

install_git_repo "DLoopDetector" "https://github.com/Bardo91/DLoopDetector"

install_git_repo "g2o" "https://github.com/RainerKuemmerle/g2o"


sudo apt-get install -y qt5-default
install_git_repo "nodeeditor" "https://github.com/paceholder/nodeeditor"
 
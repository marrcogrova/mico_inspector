#!/bin/bash



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

###################################################################
###########		INSTALL OPENCV and OPENCV contrib		###########
###################################################################

if [ -d "opencv" ] 
then
	echo "Library $1 already installed" 
else
	git clone "https://github.com/opencv/opencv_contrib"
	git clone "https://github.com/opencv/opencv"
	cd opencv
	mkdir build; cd build
	cmake .. -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules
	make -j$(nproc)
	sudo make install 
	cd ../..
fi


###################################################################
###########					INSTALL PCL	 				###########
###################################################################

sudo apt-get install -y libeigen3-dev libflann-dev libvtk6-dev libboost1.65-dev libqhull-dev
install_git_repo "pcl" "https://github.com/PointCloudLibrary/pcl"


###################################################################
###########				INSTALL SLAM DEPS				###########
###################################################################
sudo apt-get install -y liblapack-dev libblas-dev
sudo apt-get install -y libomp-dev libsuitesparse-dev libcholmod3

install_git_repo "eigen-git-mirror" "https://github.com/eigenteam/eigen-git-mirror" "3.3.7"

install_git_repo "DBoW2" "https://github.com/dorian3d/DBoW2"

install_git_repo "DLoopDetector" "https://github.com/Bardo91/DLoopDetector"

install_git_repo "g2o" "https://github.com/RainerKuemmerle/g2o"

install_git_repo "nodeeditor" "https://github.com/paceholder/nodeeditor"
 
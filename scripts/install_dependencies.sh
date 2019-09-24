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

read -r -p "Do you want to install latest version of OpenCV [y/N] " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
then
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

fi

###################################################################
###########					INSTALL PCL	 				###########
###################################################################

read -r -p "Do you want to install latest version of PCL [y/N] " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
then
	sudo apt-get install libeigen3-dev libflann-dev libvtk7-dev libboost1.65-dev libqhull-dev libomp-dev libsuitesparse-dev
    install_git_repo "pcl" "https://github.com/PointCloudLibrary/pcl"
fi


###################################################################
###########				INSTALL SLAM DEPS				###########
###################################################################
install_git_repo "DBoW2" "https://github.com/dorian3d/DBoW2"
install_git_repo "DLoopDetector" "https://github.com/Bardo91/DLoopDetector"
install_git_repo "g2o" "https://github.com/RainerKuemmerle/g2o"

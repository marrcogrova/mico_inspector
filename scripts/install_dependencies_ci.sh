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

###################################################################
###########				INSTALL SLAM DEPS				###########
###################################################################

install_git_repo "DBoW2" "https://github.com/dorian3d/DBoW2"

install_git_repo "DLoopDetector" "https://github.com/Bardo91/DLoopDetector"

install_git_repo "g2o" "https://github.com/RainerKuemmerle/g2o"

sudo apt-get install -y qt5-default
sudo apt-get install -y libqt5opengl5 libqt5opengl5-dev
install_git_repo "nodeeditor" "https://github.com/paceholder/nodeeditor"
 
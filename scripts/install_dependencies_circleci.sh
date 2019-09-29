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
	cmake .. -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DWITH_TBB=OFF -DWITH_OPENMP=OFF -DWITH_IPP=OFF -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_EXAMPLES=OFF -DWITH_NVCUVID=OFF -DWITH_CUDA=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DWITH_CSTRIPES=OFF -DWITH_OPENCL=OFF -DBUILD_JAVA=OFF -DBUILD_opencv_apps=OFF -DBUILD_opencv_js=OFF -DBUILD_opencv_gapi=OFF -DBUILD_opencv_dnn=OFF -DBUILD_opencv_ml=OFF -DBUILD_opencv_stitching=OFF -DBUILD_opencv_objdetect=OFF
	make -j$(nproc)
	sudo make install 
	cd ../..
fi

sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'

sudo ldconfig

###################################################################
###########				INSTALL SLAM DEPS				###########
###################################################################

install_git_repo "eigen-git-mirror" "https://github.com/eigenteam/eigen-git-mirror" "3.3.7"

install_git_repo "DBoW2" "https://github.com/dorian3d/DBoW2"

install_git_repo "DLoopDetector" "https://github.com/Bardo91/DLoopDetector"

install_git_repo "g2o" "https://github.com/RainerKuemmerle/g2o"


sudo apt-get install -y qt5-default
sudo apt-get install -y libqt5opengl5 libqt5opengl5-dev
install_git_repo "nodeeditor" "https://github.com/paceholder/nodeeditor"
 
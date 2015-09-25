1. Install Coin3D:

	sudo apt-get install libcoin80-dev
	
2. Install Soqt4:

	sudo apt-get install libsoqt4-dev
	
3. Download OpenRAVE. Unpack it and install it using

	mkdir build && cd build && cmake ../ && make -j4 && sudo make install && sudo ldconfig

4. Clonde the openrave_catkin repository to your catkin_ws/src older:	

	git clone https://github.com/personalrobotics/openrave_catkin.git
	
5. Clone the or_urdf package into your catkin_ws/src folder:

    git clone https://github.com/hoergems/or_urdf-1.git
    
6. Build your catkin workspace

7. Compile the package:

	mkdir build
	cd build && cmake ../ && make -j8 && cd ..

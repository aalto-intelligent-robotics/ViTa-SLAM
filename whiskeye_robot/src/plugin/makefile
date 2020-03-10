FILE_OUT = ../../libwhiskeye_gazebo.so

# special for mitch's machine
FLAGS += -I$(HOME)/mdk/src/libmiro_gazebo

# for Gazebo 7
ifeq ($(wildcard /usr/share/gazebo-9),)

FLAGS += -I/usr/include/gazebo-7
FLAGS += -I/usr/include/ignition/math2
FLAGS += -I/usr/include/ignition/msgs1
FLAGS += -I/usr/include/ignition/transport4
FLAGS += -I/usr/include/sdformat-4.4
FLAGS += -I/opt/ros/kinetic/include
FLAGS += -I../../..

LINK += -lboost_system
LINK += -L/opt/ros/kinetic/lib
LINK += -lroscpp
LINK += -limage_transport

# for Gazebo 9
else

FLAGS += -I/usr/include/gazebo-9
FLAGS += -I/usr/include/ignition/math4
FLAGS += -I/usr/include/ignition/msgs1
FLAGS += -I/usr/include/ignition/transport4
FLAGS += -I/usr/include/sdformat-6.0
FLAGS += -I/opt/ros/melodic/include
FLAGS += -I../../..

LINK += -lboost_system
LINK += -L/opt/ros/melodic/lib
LINK += -lroscpp
LINK += -limage_transport

endif



#### default target

$(FILE_OUT) : libwhiskeye_gazebo.cpp *.h makefile
	@echo "Deleting legacy shared libwhiskeye_gazebo.so ........."
	rm -f $(FILE_OUT)
	@echo $@...
	g++ -std=gnu++11 -shared -fPIC $< -o $@ $(FLAGS) $(LINK)




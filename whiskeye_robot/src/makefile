.PHONY : default
default : ../whiskeye_robot.sdf $(DAE)
	#make -C mesh # uncommented this because it is not needed (it causes an error because mesh/makefile would rebuild the .sdf form the raw stl files
	make -C sdf
	make -C plugin

.PHONY : test
test : default
	cd ../test && ./launch_sim.sh


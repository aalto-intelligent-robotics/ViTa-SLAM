
DIR_RESOURCE=$(shell pwd | sed s!device/nrp.*!resource!)
SRC=$(DIR_WHISKY_SOURCE)/cad/181116_WhiskeyeSTL.zip
TMP=/tmp/whiskeyestl
DST=.

.PHONY : default
default : $(TMP)
	@./make_collision_mesh.py "$(TMP)" "$(DST)"

$(TMP) : $(SRC)
	unzip -d /tmp $(SRC)
	touch $(TMP)	

clean :
	rm $(TMP) -rf
	rm $(DST)/hi/*
	rm $(DST)/lo/*


#!/usr/bin/python

# this script will import STL files from CAD into DAEs (hi for visual
# and lo for collision) and also rotaxes for constructing the SDF.
import os
import sys
import subprocess

# get argument, source folder with STLs
SRC=sys.argv[1]
DST=sys.argv[2]

# if not exist, skip
if not os.path.isdir(SRC):
	print "SRC not found, exiting:", SRC
	exit()

# get a list of all STL files
from os import listdir
from os.path import isfile, join
from shutil import copyfile
stls = [f for f in listdir(SRC) if isfile(join(SRC, f))]

# for each
x = []
for stl in stls:
	if not stl.endswith(".STL"):
		continue
	if "RotAxe" in stl:
		src = SRC + "/" + stl
		dst = "rotaxe/" + stl.replace(" ", "_").lower().replace("_render", "").replace("_-_", "_")
		if not os.path.isfile(dst):
			print "-->", dst
			copyfile(src, dst)
	else:
		x.append(stl)

# for each
for stl in x:

	# if not recognised, do nothing
	COUNT=0

	# specials
	reorient = "reorient_faces_coherently"
	if "Head_barebones" in stl:
		reorient = ""
	if "PearNana" in stl:
		reorient = ""

	# parse SRC to get type
	if "Whisker" in stl:
		COUNT=50
		PRE="convexhull"
		POST=reorient
		POST += " flipfaces_if"
		POST += " recompute_normals"

		# for whiskers, we also need the STL for whisker frame measurement
		src = SRC + "/" + stl
		dst = "whisker/" + stl.replace(" ", "_").lower().replace("_render", "").replace("_-_", "_")
		if not os.path.isfile(dst):
			print "-->", dst
			copyfile(src, dst)

	elif "Row" in stl:
		# these, as it happens, are not used - the rows do not have collision data
		COUNT=100
		PRE="convexhull"
		POST=reorient
		POST += " flipfaces_if"
		POST += " recompute_normals"
	else:
		COUNT=100
		PRE="convexhull"
		POST=reorient
		POST += " flipfaces_if"
		POST += " recompute_normals"

	# if recognised
	if COUNT > 0:

		# STL -> DAE
		stl_ = SRC + "/" + stl
		dae = stl.replace(".STL", ".dae").replace(" ", "_").lower().replace("_render", "").replace("_-_", "_")
		dae_ = DST + "/hi/" + dae
		if not os.path.isfile(dae_):
			print dae_
			cmd = "./meshlab_script_run.sh \"" + stl_ + "\" \"" + dae_ + "\" decimate=5000 " + reorient + " recompute_normals"
			output = subprocess.check_output(cmd, shell=True)

		# DAE HI -> DAE LO
		dae_2 = DST + "/lo/" + dae
		if not os.path.isfile(dae_2):
			print dae_2
			cmd = "./meshlab_script_run.sh \"" + dae_ + "\" \"" + dae_2 + "\" " + PRE + " decimate=" + str(COUNT) + " " + POST
			output = subprocess.check_output(cmd, shell=True)



		#cmd = "./meshlab_script_run.sh " + SRC + " " + DST + " " + PRE + " decimate=" + str(COUNT) + " " + POST


		#cmd = "./meshlab_script_run.sh " + SRC + " " + DST + " " + PRE + " decimate=" + str(COUNT) + " " + POST
		#output = subprocess.check_output(cmd, shell=True)
		#print output


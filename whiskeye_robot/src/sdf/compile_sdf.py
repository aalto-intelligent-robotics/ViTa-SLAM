#!/usr/bin/python

import numpy as np
from stl import mesh
import os.path
import copy
import math
import re

from compile_sdf_pars import *

#datum_i = 0



################################################################
##  COMMON
################################################################

def error(msg):
	raise ValueError(msg)



################################################################
##  READ ROTAXE
################################################################

print "importing rotaxes..."

# function to get whisker tip from STL file
def measure_whisker_tip_direction(stl, cen):

	# convert to m
	cenm = cen * 0.001

	# find vertex that is furthest from joint centre "cen"
	l = 0.0
	sh = stl.vectors.shape
	for tri in range(0, sh[0]):
		for ver in range(0, 3):
			v = stl.vectors[tri, ver]
			d = np.linalg.norm(v - cenm)
			if d > l:
				l = d
				tip = v

	# get direction
	d = tip - cenm

	# scale to 100mm
	d /= np.linalg.norm(d)
	d *= 0.1

	# reset
	tip = cenm + d

	# ok
	return tip

# function to get rotaxe from STL file
def import_rotaxe(stl, hint=None):

	# assume both ends are at first listed vertex
	a = stl.vectors[0, 0]
	b = a
	l = np.linalg.norm(b - a)

	# find ends iteratively by maximizing distance between a and b, "l"
	sh = stl.vectors.shape
	for tri in range(0, sh[0]):
		for ver in range(0, 3):
			v = stl.vectors[tri, ver]
			la = np.linalg.norm(v - a)
			lb = np.linalg.norm(v - b)
			if la > lb:
				if la > l:
					b = v
					l = la
			else:
				if lb > l:
					a = v
					l = lb

	# convert to mm
	a *= 1000.0
	b *= 1000.0

	# rotational centre is midway between a and b
	cen = (a + b) / 2.0

	# axis of rotation is vector towards one end - if no
	# hint is given, we'll go towards a
	use_a = True

	# if a hint is given
	if not hint is None:

		# hints help to disambiguate which way round the axis is
		if hint == "n3":
			# point towards -ve x will end up as +ve z in correct configuration
			use_a = a[0] < b[0]
		elif hint == "n1" or hint == "n2":
			# point towards +ve y
			use_a = a[1] > b[1]
		elif hint == "w":
			# for whiskers, the hint is that the axis should point counter-clockwise
			# round the z axis so that the end result is "+ve theta is protraction"
			ang_a = math.atan2(a[1], a[0]-175.0)
			ang_b = math.atan2(b[1], b[0]-175.0)
			while (ang_b - ang_a) > 1.5:
				ang_b -= 3.14
			while (ang_a - ang_b) > 1.5:
				ang_a -= 3.14
			use_a = ang_a < ang_b
		else:
			error("hint not recognised")

	# use_a or b
	if use_a:
		ax = a - cen
	else:
		ax = b - cen

	# normalise ax - don't think this is necessary/helpful since it is
	# later converted to metres and is thus no longer unitary...
	ax /= np.linalg.norm(ax)

	# ok
	return (cen, ax)

rotaxe_neck = []
rotaxe_whiskers = []

# do neck
pat = "../mesh/rotaxe/whiskeye_rotaxeneck-{}.stl"
for dof in range(1, 4):

	# read stl
	filename = pat.format(dof)
	stl = mesh.Mesh.from_file(filename)

	# import rotaxe
	rotaxe = import_rotaxe(stl, "n" + str(dof))

	# store rotaxe
	rotaxe_neck.append(rotaxe)

# do whiskers
pat = "../mesh/rotaxe/whiskeye_head-1_row_mini-{}_rotaxe-{}.stl"
for row in range(1, 7):
	for col in range(1, 5):

		# read stl
		filename = pat.format(row, col)
		stl = mesh.Mesh.from_file(filename)

		# import rotaxe
		rotaxe = import_rotaxe(stl, "w")

		# store rotaxe
		rotaxe_whiskers.append(rotaxe)

print "---- OK"



################################################################
##  READ MESH
################################################################

def get_files(path):
	from os import listdir
	from os.path import isfile, join
	files = [f for f in listdir(path) if isfile(join(path, f))]
	return files

dae = []
files = get_files("../../mesh/hi")
for file in files:
	if "whisker" in file:
		continue
	dae.append(file)



################################################################
##  READ MATERIALS
################################################################

materials = {}
with open('materials.txt', 'r') as file:
	lines = file.readlines()
	for line in lines:
		f = line.find('=')
		if f > 0:
			key = line[0:f].strip()
			val = line[f+1:].strip()
			materials[key] = val
			#print "[" + key + "]", "[" + val + "]"



################################################################
##  TEMPLATES
################################################################

def decorate_mesh(name, collision):

	if collision:
		res = "lo"
	else:
		res = "hi"
	return "model://whiskeye_robot/mesh/" + res + "/" + name

def detoken(txt, dic):

	# for each dictionary entry
	for key, val in dic.iteritems():
		key = "$(" + key.upper() + ")"
		txt = txt.replace(key, val)

	return txt

def get_color(tag):
	for k, v in materials.items():
		if re.match(k, tag):
			return v
	print "no color specified for", tag
	return "Red"

def use_for_visual(mesh):

	# for now
	return use_for_collision(mesh)

def use_for_collision(mesh):

	# determine which meshes are included for collision checking
	inc = [
		"whiskerasm",
		"wheel",
		"body-3_body-1",
		"host_pc"
		]
	for i in inc:
		if i in mesh:
			return True

	# for now, include all the missing ones too - optimise this
	# at some point!
	print mesh
	return True

def detoken_link(dic, dae=None):

	# expand name
	if not "ref" in dic:
		dic["ref"] = dic["name"]

	# expand inertia
	if not "ixx" in dic:
		dic["ixx"] = dic["i"]
		dic["iyy"] = dic["i"]
		dic["izz"] = dic["i"]

	# expand collide_without_contact
	if not "collide_without_contact" in dic:
		dic["collide_without_contact"] = "0"

	# expand mu
	if not "mu2" in dic:
		dic["mu2"] = dic["mu"]

	# expand visual/dae
	if not "meshes" in dic:
		dic["meshes"] = []
	meshes = dic["meshes"]
	del dic["meshes"]
	if not dae is None:
		for file in dae:
			if dic["ref"] in file:
				meshes.append(file)

	# expand visuals
	dic["visuals"] = ""
	dic2 = {}
	for v in meshes:
		if not use_for_visual(v):
			continue
		dic2["color"] = get_color(v[0:-4])
		dic2["name"] = v
		dic2["uri_mesh_visual"] = decorate_mesh(v, False)
		dic["visuals"] += detoken(template_visual, dic2)

	# expand collisions
	dic["collisions"] = ""
	dic2 = {}
	for v in meshes:
		if not use_for_collision(v):
			continue
		dic2["name"] = v
		dic2["uri_mesh_collision"] = decorate_mesh(v, True)
		dic2["mu"] = dic["mu"]
		dic2["mu2"] = dic["mu2"]
		dic["collisions"] += detoken(template_collision, dic2)

	# expand sensors
	if not "sensors" in dic:
		dic["sensors"] = ""
		
	# expand datums
	#if not "datums" in dic:
	#	dic["datums"] = ""
		
	return detoken(template_link, dic)

def detoken_joint(txt, dic):

	# expand name
	dic["name"] = dic["parent"] + "_" + dic["child"]

	# expand pose
	dic["pose"] = dic["pose"] + " 0 0 0"

	# expand range
	if "range" in dic:
		(lo, hi) = dic["range"]
		dic["limit_lo"] = str(np.deg2rad(lo))
		dic["limit_hi"] = str(np.deg2rad(hi))
		del dic["range"]

	return detoken(txt, dic)

def add_sensor(txt, sens):

	if visualize_sensors:
		sens["visualize"] = "true"
	elif "visualize" in sens:
		pass
	else:
		sens["visualize"] = "false"

	if sens["type"] == "imu":
		txt += detoken(template_imu, sens)
	elif sens["type"] == "camera":
		txt += detoken(template_camera, sens)
	elif sens["type"] == "contact":
		txt += detoken(template_contact, sens)

	return txt

#def add_datum(txt, x, r):
#
#	global datum_i
#	
#	dic = {}
#	dic["POS"] = "{:.6f}".format(x[0]) + " {:.6f}".format(x[1]) + " {:.6f}".format(x[2])
#	dic["NAME"] = "datum_" + str(datum_i)
#	dic["RADIUS"] = str(r)
#	datum_i += 1
#
#	txt += detoken(template_datum, dic)
#
#	return txt

# read template
with open('template_link.sdf', 'r') as file:
    template_link = file.read()

# read template (for additional visual elements)
with open('template_visual.sdf', 'r') as file:
    template_visual = file.read()

# read template (for additional collision elements)
with open('template_collision.sdf', 'r') as file:
    template_collision = file.read()

# read template
with open('template_joint.sdf', 'r') as file:
    template_joint = file.read()

# read template
with open('template_robot.sdf', 'r') as file:
    template_robot = file.read()

# read template
with open('template_imu.sdf', 'r') as file:
    template_imu = file.read()

# read template
with open('template_camera.sdf', 'r') as file:
    template_camera = file.read()

# read template
with open('template_contact.sdf', 'r') as file:
    template_contact = file.read()

# read template
#with open('template_datum.sdf', 'r') as file:
#    template_datum = file.read()



################################################################
##  BUILD
################################################################

def format_vector(vec):
	x = vec * 0.001
	return "{} {} {}".format(x[0], x[1], x[2])

def format_vector_unit(vec):
	x = vec * (1.0 / np.linalg.norm(vec))
	x *= 0.001
	return "{} {} {}".format(x[0], x[1], x[2])

def rotaxe_cen(rotaxe):
	return rotaxe[0]

def rotaxe_axis(rotaxe):
	return rotaxe[1]

# start content
content = "";

# add link
data = {}
data["name"] = "body"
data["centre"] = "0 0 0.1"
data["mass"] = str(phys_mass_body)
data["i"] = str(phys_mass_body * 0.1)
data["mu"] = "0.1"
data["sensors"] = ""
sens = {}
sens["type"] = "imu"
sens["name"] = "imu_body"
data["sensors"] = add_sensor(data["sensors"], sens)
sens = {}
sens["type"] = "camera"
sens["name"] = "cam2"
sens["pose"] = "0.0 0.0 0.5 0 0 0"
#sens["visualize"] = "true"
data["sensors"] = add_sensor(data["sensors"], sens)
content += detoken_link(data, dae)

# add joint
data = {}
data["parent"] = "body"
data["child"] = "neck"
data["axis"] = format_vector_unit(rotaxe_axis(rotaxe_neck[0]))
data["pose"] = format_vector(rotaxe_cen(rotaxe_neck[0]))
data["range"] = (0, 70)
data["friction"] = str(neck_friction)
data["damping"] = str(neck_damping)
content += detoken_joint(template_joint, data)

# add link
data = {}
data["name"] = "neck"
data["centre"] = format_vector((rotaxe_cen(rotaxe_neck[0]) + rotaxe_cen(rotaxe_neck[1])) / 2)
data["mass"] = str(phys_mass_neck)
data["i"] = str(phys_mass_neck * 0.1)
data["mu"] = "0.1"
content += detoken_link(data, dae)

# add joint
data = {}
data["parent"] = "neck"
data["child"] = "gmbl"
data["axis"] = format_vector_unit(rotaxe_axis(rotaxe_neck[1]))
data["pose"] = format_vector(rotaxe_cen(rotaxe_neck[1]))
data["range"] = (0, 120)
data["friction"] = str(neck_friction)
data["damping"] = str(neck_damping)
content += detoken_joint(template_joint, data)

# add link
data = {}
data["name"] = "gmbl"
data["ref"] = "gimbal"
data["centre"] = format_vector((rotaxe_cen(rotaxe_neck[1]) + rotaxe_cen(rotaxe_neck[2])) / 2)
data["mass"] = str(phys_mass_gmbl)
data["i"] = str(phys_mass_gmbl * 0.1)
data["mu"] = "0.1"
content += detoken_link(data, dae)

# add joint
data = {}
data["parent"] = "gmbl"
data["child"] = "head"
data["axis"] = format_vector_unit(rotaxe_axis(rotaxe_neck[2]))
data["pose"] = format_vector(rotaxe_cen(rotaxe_neck[2]))
data["range"] = (-90, 90)
data["friction"] = str(neck_friction)
data["damping"] = str(neck_damping)
content += detoken_joint(template_joint, data)

# add link
data = {}
data["name"] = "head"
data["centre"] = format_vector(rotaxe_cen(rotaxe_neck[2]) + np.array([0.0, 0.0, 100.0]))
data["mass"] = str(phys_mass_head)
data["i"] = str(phys_mass_head * 0.1)
data["mu"] = "0.1"
data["sensors"] = ""
#data["datums"] = ""
x = rotaxe_cen(rotaxe_neck[2]) * 0.001
#data["datums"] = add_datum(data["datums"], x + np.array([0.040, 0.0, 0.220]), 0.005)
#data["datums"] = add_datum(data["datums"], x + np.array([0.040, 0.0, 0.070]), 0.130) # DOCLINK HEAD SPHERE
sens = {}
sens["type"] = "camera"
sens["name"] = "cam0"
sens["pose"] = "0.0778 0.0573 0.5166 0 -1.5708 0"
data["sensors"] = add_sensor(data["sensors"], sens)
sens["name"] = "cam1"
sens["pose"] = "0.0778 -0.0573 0.5166 0 -1.5708 0"
data["sensors"] = add_sensor(data["sensors"], sens)
content += detoken_link(data, dae)

# rotaxe counter
rotaxe_i = 0

# posefile
posefile = "/*this file is automatically generated*/\n"
posefile += "double neck_pose[] = {\n"
for i in range(0, len(rotaxe_neck)):
	rotaxe = rotaxe_neck[i]
	o = rotaxe_cen(rotaxe) * 0.001 # origin (centre of joint)
	y = o + 0.1 * rotaxe_axis(rotaxe) # 100mm in +ve y
	z = o + 0.1 * np.array([0, 0, 1.0]) # 100mm in +ve z
	pose = np.concatenate((o, y, z))
	for q in range(0, 9):
		posefile += str(pose[q]) + ", "
	posefile += "\n"
posefile += "};\n"
posefile += "double whisker_pose[] = {\n"

# for each whisker row
for row in range(1, 7):

	# for each whisker
	for col in range(1, 5):

		# tokens
		lngs = [50, 70, 100, 160]
		lng = lngs[col-1]
		name = "whisker" + str(row) + "_" + str(col)
		whsk = "whiskeye_head-1_row_mini-{}_whiskerasm_{}mm-1".format(row, lng)

		# get rotaxe from array
		rotaxe = rotaxe_whiskers[rotaxe_i]
		rotaxe_i += 1

		# read STL to recover the whisker tip
		stl = mesh.Mesh.from_file("../mesh/whisker/" + whsk + ".stl")
		tip = measure_whisker_tip_direction(stl, rotaxe_cen(rotaxe))

		# write whisker pose file
		o = rotaxe_cen(rotaxe) * 0.001 # origin (centre of joint)
		y = o + 0.1 * rotaxe_axis(rotaxe) # 100mm in +ve y
		z = tip # 100mm in +ve z
		pose = np.concatenate((o, y, z))
		for q in range(0, 9):
			posefile += str(pose[q]) + ", "
		posefile += "\n"

		# add joint
		data = {}
		data["parent"] = "head"
		data["child"] = name
		data["axis"] = format_vector_unit(rotaxe_axis(rotaxe))
		data["pose"] = format_vector(rotaxe_cen(rotaxe))
		data["range"] = (-60, 60)
		data["friction"] = "0.1"
		data["damping"] = "0.1"
		content += detoken_joint(template_joint, data)

		# add link
		data = {}
		data["name"] = name
		data["centre"] = format_vector(rotaxe_cen(rotaxe))
		data["mass"] = str(phys_mass_whisker)
		data["i"] = str(phys_mass_whisker * 0.1)
		data["mu"] = "0.1"
		data["meshes"] = [whsk + ".dae"]
		data["sensors"] = ""
		data["collide_without_contact"] = "1"
		sens = {}
		sens["type"] = "contact"
		sens["name"] = name
		sens["target"] = data["meshes"][0] + "_collision"
		data["sensors"] = add_sensor(data["sensors"], sens)
		content += detoken_link(data)

# install content
data = {}
data["content"] = content
sdf = detoken(template_robot, data)

# write sdf
with open('../../whiskeye_robot.sdf', 'w') as file:
	file.write(sdf)

# write posefile
posefile += "};\n"
with open('../plugin/whisker_pose.h', 'w') as file:
	file.write(posefile)



################################################################
##  FIN
################################################################

print "(build complete)"


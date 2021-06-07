import numpy as np
from stl import mesh
import pybullet as p

# fly = p.loadURDF("fly.urdf", [0,0,0], p.getQuaternionFromEuler([0,0,0]))

def get_link_cog(file):
	stl = mesh.Mesh.from_file(file)
	_, cog, _ = stl.get_mass_properties()
	return cog

# print(get_link_cog("thorax.stl"))

# Inertail matrix calculation is also available:

# your_mesh = mesh.Mesh.from_file('some_file.stl')
# volume, cog, inertia = your_mesh.get_mass_properties()
# print("Volume = {0}".format(volume))
# print("Position of the center of gravity (COG) = {0}".format(cog))
# print("Inertia matrix at expressed at the COG = {0}".format(inertia[0,:]))
# print(" {0}".format(inertia[1,:]))
# print(" {0}".format(inertia[2,:]))
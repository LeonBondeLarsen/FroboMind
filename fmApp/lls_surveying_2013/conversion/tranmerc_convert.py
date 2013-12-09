#!/usr/bin/env python
#*****************************************************************************
# KP2000 to UTM projection conversion
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************
"""
This file contains a Python script to convert coordinates in a file between
KP2000, UTM32 and latitude/longitude.

Revision
2013-12-04 KJ First version
"""
# import kp2000conv class
from transverse_mercator import tranmerc
from kp2000 import kp2000conv
from math import pi, cos
from sys import argv

# general defines
deg_to_rad = pi/180.0
# WGS-84 defines
wgs84_a = 6378137.0 # WGS84 semi-major axis of ellipsoid [m]
wgs84_f = 1/298.257223563 # WGS84 flattening of ellipsoid
# UTM defines
utm_false_easting = 500000.0
utm_scale_factor = 0.9996
utm_origin_latitude = 0.0 * deg_to_rad
# UTM32 defines
utm32_central_meridian = 9.0 * deg_to_rad
utm32_false_northing = 0.0 * deg_to_rad


# instantiate kp2000conv class
kp = kp2000conv()

# setup utm32 conversion
tm = tranmerc()


def load_from_csv (filename):
	print 'Loading waypoints from: %s' % filename
	wpts = []
	lines = [line.rstrip('\n') for line in open(filename)] # read the file and strip \n
	wpt_num = 0
	for i in xrange(len(lines)): # for all lines
		if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
			data = lines[i].split (',') # split into comma separated list
			if len(data) >= 2 and data[0] != '' and data[1] != '':
				wpt_num += 1
				e = float (data[0])
				n = float (data[1])

				wpts.append([e, n])
			else:
				print '  Erroneous waypoint: %s' % lines[i]
	print '  Total %d waypoints loaded.' % wpt_num
	return wpts

def save_to_csv(filename):
	f = open(filename, 'w')
		for i in xrange(len(

argc = len(argv)
if argc != 4:
	print 'Usage: tranmerc_convert.py conversion infile outfile'
	print 'Conversion: ll_utm32
else:
	conv =  argv[1:][0]
	inf =  argv[1:][1]
	outf =  argv[1:][2]

	in_wpts = load_from_csv(inf)
	out_wpts = []

	if conv == 'll_utm32':
		print 'Convertion from geographical coordinates to UTM32...'
		tm.set_params (wgs84_a, wgs84_f, utm_origin_latitude, \
    		utm32_central_meridian, utm_false_easting, utm32_false_northing, utm_scale_factor)
		for i in xrange(len(in_wpts)):
			(e, n) = tm.geodetic_to_tranmerc (in_wpts[i][1]*deg_to_rad, in_wpts[i][2]*deg_to_rad)
			out_wpts.append ([e,n])



	save_to_csv(out_wpts)
	print 'Quit'







# convert from geodetic to KP2000
(easting, northing) = kp.geodetic_to_kp2000 (test_lat, test_lon, kp.kp2000j)
print '\nConverted from geodetic to KP2000 [m]'
print '  %.5fe %.5fn' % (easting, northing)

# convert back from KP2000 to geodetic
(lat, lon) = kp.kp2000_to_geodetic (easting, northing, kp.kp2000j)
print '\nConverted back to geodetic [deg]:'
print '  latitude:  %.10f'  % (lat)
print '  longitude: %.10f'  % (lon)

# detrmine conversion position error [m]
lat_err = abs(lat-test_lat)
lon_err = abs(lon-test_lon)
earth_radius = 6378137.0 # [m]
lat_pos_err = lat_err/360.0 * 2*pi*earth_radius
lon_pos_err = lon_err/360.0 * 2*pi*(cos(lat)*earth_radius)
print '\nPositional error from the two conversions [m]:'
print '  latitude:  %.9f'  % (lat_pos_err)
print '  longitude: %.9f'  % (lon_pos_err)



import socket
import sys
from ctypes import *
from getkey import getkey, keys

vel = 0
phi = 0

UDP_IP = "192.168.1.1" # feather
UDP_PORT_CMD = 4241
UDP_PORT_RTN = 4242

sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_cmd.connect((UDP_IP, UDP_PORT_CMD))

sock_rtn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rtn.connect((UDP_IP, UDP_PORT_RTN))

class cmdPacket(Structure):
	_fields_ = [ ( "velocity", c_double ) , ( "theta", c_double ) , ( "mode", c_int ) ]
	
class rtnPacket(Structure):
	_fields_ = [ ( "x", c_double ), ( "y", c_double ), ( "head", c_double ) ]

# 	_fields_ = [ ( "odo", c_double * 3 ) , ( "imu", c_double * 6 ) , ( "heading", c_double ) ]

# mode 0 = return x,y,heading
# mode 1 = driving command
# mode 2 = cardinal command
# mode 3 = stop
# mode 4 = reset

while True:
	key = getkey()
	
	if ( keys.UP == key ) : #forward
		print("fwd")
		
		vel += 10
		if ( vel > 100 ):
			vel = 100
		
		sock_cmd.send( cmdPacket( vel, phi, 1 ) )

	elif ( keys.DOWN == key ):
		print("back")

		vel -= 10
		
		if ( vel < 0 ):
			vel = 0
		
		sock_cmd.send( cmdPacket( vel, phi, 1 ) )
		
	elif ( keys.LEFT == key ):
		print("left")
		
		phi -= 15
		if ( phi < 0 ):
			phi += 360
		
		sock_cmd.send( cmdPacket( vel, phi, 1 ) )
		
	elif ( keys.RIGHT == key ):
		print("right")
		
		phi += 15
		if ( phi >= 360 ):
			phi -= 360
		
		sock_cmd.send( cmdPacket( vel, phi, 1 ) )
		
	elif ( 'w' == key ):
		print("north")
		
		vel = 0
		phi = 0
		
		sock_cmd.send( cmdPacket( vel, phi, 2 ) )
		
	elif ( 's' == key ):
		print("south")
		
		vel = 0
		phi = 180
		
		sock_cmd.send( cmdPacket( vel, phi, 2 ) )
		
	elif ( 'a' == key ):
		print("west")
		
		vel = 0
		phi = 270
		
		sock_cmd.send( cmdPacket( vel, phi, 2 ) )
		
	elif ( 'd' == key ):
		print("east")
		
		vel = 0
		phi = 90
		
		sock_cmd.send( cmdPacket( vel, phi, 2 ) )
		
	elif ( keys.SPACE == key ):
		print("all stop")
		
		vel = 0
		
		sock_cmd.send( cmdPacket( vel, phi, 3 ) )
			
	elif ( 'q' == key ):
		print("response")
				
		sock_cmd.send( cmdPacket( vel, phi, 0 ) )

		buffer = sock_cmd.recv( sizeof( rtnPacket ) )
		newRtnPacket = rtnPacket.from_buffer_copy( buffer )

		for field_name, field_type in newRtnPacket._fields_:
			print field_name, getattr(newRtnPacket, field_name)

	elif ( 'r' == key ):
		print("reset")

		vel = 0
		
		sock_cmd.send( cmdPacket( vel, phi, 4 ) )
		
	else:
		print("unrecognized")
	
		# print "odo %d %d %d" % ( newRtnPacket.odo[ 0 ], newRtnPacket.odo[ 1 ], newRtnPacket.odo[ 2 ] )
		# print "imu %d %d %d -- %d %d %d" % ( newRtnPacket.imu[ 0 ], newRtnPacket.imu[ 1 ], newRtnPacket.imu[ 2 ], newRtnPacket.imu[ 3 ], newRtnPacket.imu[ 4 ], newRtnPacket.imu[ 5 ] )
		# print "hdg %d" % ( newRtnPacket.heading )
	# time.sleep( 0.1 )

import socket, serial, time

ard = serial.Serial('/dev/ttyUSB0', 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
UDP_IP = "127.0.0.1"
UDP_PORT = 30303
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('8.8.8.8', 0))  # connecting to a UDP address doesn't send packets
local_ip_address = s.getsockname()[0]
print "IP is ", local_ip_address
sock = socket.socket(socket.AF_INET, # Internet
		socket.SOCK_DGRAM) # UDP
sock.bind((local_ip_address, UDP_PORT))
print "Startup"
while True:
	sock.settimeout(.3)
	try:
		data, addr = sock.recvfrom(2) # buffer size is 1024 bytes
	except Exception:
		data = chr(0)
	sock.settimeout(None)
	#data = bytearray()  # New empty byte array
	# Append data to the array
	#data = 9 
	print "received message:", data
	ard.write(data)
ard.write(chr(0))

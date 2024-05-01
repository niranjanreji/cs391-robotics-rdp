import socket
import select
import json
import time
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool

class Pub(Node):
	status = True
	def __init__(self):
		super().__init__('lidar_out')
		self.lidar_out = self.create_publisher(Int32MultiArray, 'lidar_data',10)
		self.flight_status = self.create_subscription(Bool, 'flight_status', self.status_update, 10)
   
	def status_update(self, msg):
		if msg.data != self.status:
			self.status = msg.data
	
	def send_data(self, data):
		if self.status:
			msg = Int32MultiArray()
			msg.data = data
			self.lidar_out.publish(msg)
			self.get_logger().info(f"Publishing {msg} with {data}")

class PicoListener:
	def __init__(self, HOST='0.0.0.0', PORT=8000):
		self.data_buffer = []
		self.setup_socket_server(HOST, PORT)
		self.inputs = [self.listening_socket]

	def setup_socket_server(self, host, port, max_waiting=5):
		self.listening_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.listening_socket.bind((host, port))
		self.listening_socket.listen(max_waiting)
		print(f"Server started, listening on IP: {host}, Port: {port}")

	def listen_for_pico(self):
		while self.inputs:
			readable, _, _ = select.select(self.inputs, [], [])
			self.handle_input(readable)
			time.sleep(0.05)

	def handle_input(self, readable):
		for s in readable:
			if s == self.listening_socket:
				pico_connection, pico_addr = s.accept()
				print(f"Connected to {pico_addr}")  
				self.inputs.append(pico_connection)
			else:
				data = s.recv(1024)  # Read a chunk of data (up to 1024 bytes)
				if data is None:
					self.shut_socket(s)
					continue
				distances = struct.unpack(f'>{len(data)//4}I', data)  # Unpack into list of ints
				self.update_plot(distances)  # Update the plot with new data
				self.data_buffer.extend(distances)
				#print(self.data_buffer)
				print(f"Received data from {s.getpeername()[0]}: {distances}")

	def recv_all(self, sock, length):
		data = b''
		while len(data) < length:
			more = sock.recv(length - len(data))
			if not more:
				print(f"Connection hung up or incomplete data. Expected {length}, received {len(data)}")
				return None
			data += more
		return data

	def shutdown_sockets(self):
		for s in self.inputs:
			s.close()

	def shut_socket(self, s): #For shutting down individual sockets
		s.close()
		self.inputs.remove(s)
		print(f"Disconnected from {s.getpeername()}")

def main():
	print('Hi from dronecomm.')
	print("Starting the server...")
	pico_listener = PicoListener()
	try:
		pico_listener.listen_for_pico()
	except KeyboardInterrupt:
		for s in pico_listener.inputs[:]: # Iterate over a copy of the list
			if s is not pico_listener.listening_socket:
				pico_listener.shut_socket(s)
	finally:
		print("Data Buffer:", pico_listener.data_buffer) # Print all collected data
		rclpy.init()
		p = Pub()
		p.send_data(pico_listener.data_buffer)
		pico_listener.shutdown_sockets() # Makes sure all sockets are closed


if __name__ == '__main__':
	main()

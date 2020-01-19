import socket

BUFFER_SIZE = 50000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.50.33', 3883))

try:
    while True:
        data = sock.recv(BUFFER_SIZE)
except:
    sock.close()

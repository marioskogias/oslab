#!/usr/bin/env python 

import socket
import sys
import os
from multiprocessing import Process
import signal
import subprocess
import time 
import select
 
def server():
        port = 50000
        backlog = 5
        size = 1024
        host = ""
        signal.signal(signal.SIGPIPE,signal.SIG_IGN)

        try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error, e:
                print "Error %d: %s" % (e.args[0], e.args[1])
                sys.exit(1)

        sys.stderr.write("Created TCP socket\n")

        try:
                s.bind((host,port))
                s.listen(backlog)
        except socket.error:
                print "Error %d: %s" % (e.args[0],e.args[1])
                sys.exit(1)

        sys.stderr.write("Bound TCP socket to port " + str(port) + "\n")

        client,address=s.accept()
        print "connection from "+str(address)
        return client

def client():
        host = sys.argv[1]
        port = int(sys.argv[2])

        try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error, e:
                print "Error %d: %s" % (e.args[0], e.args[1])
                sys.exit(1)

        sys.stderr.write("Created TCP socket\n")


        try:
                hp = socket.gethostbyname(host)
        except socket.error, e:
                print "Error %d: %s" % (e.args[0], e.args[1])
                sys.exit(1)

        sys.stderr.write("Connecting to remote host ...\n")

        try:
                s.connect((host,port))
        except socket.gaierror, e:
                print "Address-related error connecting to server: %s" % e
                sys.exit(1)

        return s


def send(so):
	text = raw_input()
        text = text+"\0"
        en_sock.send(text)
        text = en_sock.recv(1024)
        so.sendall(text)

def receive(so):
	count = 0
	data = so.recv(256)
	count = sys.getsizeof(data)
	while (count<256):        # prepei na diavasei 256
		temp = so.recv(256)
		count = count + sys.getsizeof(temp)
		data = data + temp
        if data:
        	dec_sock.send(data)
               	data = dec_sock.recv(1024)
                out = "                                           "+data
		print out
        else:
        	print "Peer went away"
                os.abort()

if ((len(sys.argv) != 3) and  (len(sys.argv) != 2)):
        print "Usage: "+ sys.argv[0] + " <hostname> <port> to join a conversation"
        print "Usage: "+ sys.argv[0] + "server to initianlize a conversation"
        sys.exit(1)

en = subprocess.Popen([os.getcwd()+"/cryptosock","123"])
time.sleep(0.5)
en_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
en_sock.connect("encrypt.socket")

dec = subprocess.Popen([os.getcwd()+"/decryptosock","123"])
time.sleep(0.5)
dec_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
dec_sock.connect("decrypt.socket")

if (sys.argv[1]=="server"):
        sock = server()
else:
        sock=client()

print "I say",
print "The remote says".rjust(50)
print "---------------------------------------------------------"

while True:
	readable, writable, exceptional = select.select([sock,sys.stdin], [],[])
	for s in readable:
		if s is sock:
                        receive(sock)
                else:
                        send(sock)

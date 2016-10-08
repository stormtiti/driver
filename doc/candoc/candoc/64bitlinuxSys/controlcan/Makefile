
all:
	#gcc -o hello -L. -lcontrolcan -lpthread -lusb main.c
	#arm-none-linux-gnueabi-gcc -o hello -L. -L /home/caidunqing/controlcan -lcontrolcan -lpthread main.c
	#gcc -o hello -L. -L /home/harry/cdq/controlcan -lcontrolcan -lpthread main.c
	gcc -o hello main.c /home/ttc/Desktop/controlcan_x64/libcontrolcan.so -lpthread

clean:
	rm -f *.o hello

TARGET = src/Main.cpp src/Chassis.cpp
BUILD = build/
SRC = src/

WALL = -Wall -o $(BUILD)Square-One-Autonomous-Car.obj


teleport : 
	cd .. && scp -r Square-One-Autonomous-Car pi@192.168.4.1:~/Square-One-Autonomous-Car

compile : 
	gcc $(WALL) $(TARGET)

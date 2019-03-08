#ifndef CHASSIS_H
#define CHASSIS_H

class Chassis {
private:
    int port;
    float output;
    float input;

public:
    Chassis(int myPort);
    ~Chassis();
    void set();
}; 

#endif
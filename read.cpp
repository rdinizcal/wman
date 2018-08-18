#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <string>    /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

using namespace std;

int main(int argc, char *argv[]){

    int pid, serial_fd; //process ID info for forking and file descriptor for serial stream
    struct termios port_config; //sets up termios configuration structure for the serial port
    char buffer[sizeof(int)];

    const char *device = "/dev/ttyACM1";
    serial_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY); 
    if(serial_fd == -1) { 
        fprintf(stdout, "failed to open port\n");
    }

    tcgetattr(serial_fd, &port_config);

    cfsetispeed(&port_config, B9600); //set baud input to 9600 (might be wrong?)
    cfsetospeed(&port_config, B9600); //set baud output to 9600 (might be wrong?)

    port_config.c_cflag         &=  ~PARENB;            // Make 8n1
    port_config.c_cflag         &=  ~CSTOPB;
    port_config.c_cflag         &=  ~CSIZE;
    port_config.c_cflag         |=  CS8;
    port_config.c_cflag         &=  ~CRTSCTS;           // no flow control
    port_config.c_lflag         =   0;                  // no signaling chars, no echo, no canonical processing
    port_config.c_oflag         =   0;                  // no remapping, no delays

    port_config.c_cc[VMIN]      =   0;                  // read doesn't block
    port_config.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

    port_config.c_cflag         |=  CREAD | CLOCAL;                  // turn on READ & ignore ctrl lines
    port_config.c_iflag         &=  ~(IXON | IXOFF | IXANY);         // turn off s/w flow ctrl
    port_config.c_lflag         &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    port_config.c_oflag         &=  ~OPOST;                          // make raw

    tcsetattr(serial_fd, TCSAFLUSH, &port_config); //Sets the termios struct of the file handle fd from the options defined in options. TCSAFLUSH performs the change as soon as possible.

    cout << "Receiving Process Started...." << endl;
    while(true){
        if (read(serial_fd, &buffer, sizeof(buffer)) > 0 ){
            cout << "Read: " << atoi(buffer) << endl;
        }
    }

    close(serial_fd); //close serial stream

    return 0;
}
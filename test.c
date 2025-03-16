#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <stdint.h>


extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[], int telemetryMotor) ;


#define MAXN    40
#define TELEM_LEN 10
#define TELEM_POLES 14

int motorPins[MAXN];
double throttles[MAXN];
uint8_t telem_buf[TELEM_LEN];
int telem_pos;
int telem_fd;
int telem_rpm;
int telem_current;

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
    uint8_t crc = 0, i;
    for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
    return (crc);
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                perror("error from tcgetattr");
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSAFLUSH, &tty) != 0)
        {
                perror("error from tcsetattr");
                return -1;
        }
        return 0;
}

int
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                perror("error from tggetattr");
                return -1;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSAFLUSH, &tty) != 0) {
                perror("error setting term attributes");
                return -1;
        }

        return 0;
}

void handle_telem(void) {
    if (get_crc8(telem_buf, TELEM_LEN-1) == telem_buf[TELEM_LEN-1]) {
        uint32_t rpm = telem_buf[7];
        rpm <<= 8;
        rpm |= telem_buf[8];
        rpm = rpm * 100 / (TELEM_POLES / 2);
        telem_rpm = rpm;
        uint32_t current = telem_buf[3];
        current <<= 8;
        current |= telem_buf[4];
        telem_current = current;
        //printf("rpm = %d\n", rpm);
    } else {
        printf("bad crc\n");
    }
}

void check_telem(void) {
    ssize_t r = read(telem_fd, telem_buf + telem_pos, TELEM_LEN - telem_pos);
    if (r < 0) {
        if (errno == EAGAIN) {
            return;
        }
        perror("error reading telem tty");
        exit(1);
    }
    telem_pos += r;
    if (telem_pos == TELEM_LEN) {
        handle_telem();
        telem_pos = 0;
        check_telem();
    }
}

int main(int argc, char **argv) {
    int i, n, j;

    if (argc < 3) {
        printf("usage:   ./test <telem_tty> <gpio_pin_1> ... <gpio_pin_n>\n");
        printf("example: sudo ./test 16 19 20 21\n");
        exit(1);
    }

    telem_fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
    if (telem_fd < 0) {
        perror("error opening telem tty");
        exit(1);
    }
    if (set_interface_attribs(telem_fd, B115200, 0)) {
        exit(1);
    }
    /*if (set_blocking(telem_fd, 0)) {
        exit(1);
    }*/

    int flags = fcntl(telem_fd, F_GETFL, 0);
    fcntl(telem_fd, F_SETFL, flags | O_NONBLOCK);

    n = 0;
    for(i=2; i<argc; i++) {
        motorPins[n] = atoi(argv[i]);
        if (motorPins[n] >= 8 && motorPins[n] <= 25) {
            n ++;
        } else {
            printf("pin %d out of range 8..25\n", motorPins[n]);
        }
    }

    motorImplementationInitialize(motorPins, n);

    printf("Initializing ESC / Arm, waiting 5 seconds.\n");
    fflush(stdout);
    // send 0 throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = 0;
    for(i=0; i<5000; i++) {
        motorImplementationSendThrottles(motorPins, n, throttles, -1);
        usleep(1000);
    }

    printf("Spinning.\n");
    fflush(stdout);
    // make motors spinning on 15% throttle during 5 seconds
    for(j = 0; j < 100; j++) {
        throttles[0] = 0.01 * j;
        int rpm_sum = 0;
        int rpm_count = 0;
        int current_sum = 0;
        int current_count = 0;
        for(i=0; i<1000; i++) {
            int send_telemetry = (i % 10) == 0;
            motorImplementationSendThrottles(motorPins, n, throttles, send_telemetry ? 0 : -1);
            check_telem();
            usleep(1000);

            if (i > 500) {
                if (telem_rpm > 0) {
                    rpm_sum += telem_rpm;
                    rpm_count++;
                    telem_rpm = 0;
                }
                if (telem_current > 0) {
                    current_sum += telem_current;
                    current_count++;
                    telem_current = 0;
                }
            }
        }
        printf("result,%lf,%lf,%lf\n", throttles[0], rpm_sum/(double)rpm_count,
                current_sum/(double)current_count/100.0);
    }

    printf("Stop.\n");
    fflush(stdout);
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles, -1);

    // finalize
    motorImplementationFinalize(motorPins, n);

    return(0);
}

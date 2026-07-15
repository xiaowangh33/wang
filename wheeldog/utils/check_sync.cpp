#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

int main() {
    const char *port = "/dev/ttyUSB0";
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("打开串口失败");
        return -1;
    }
    
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    tcsetattr(fd, TCSANOW, &tty);
    
    write(fd, "LOG ENABLE\r\n", 12);
    usleep(500000);
    
    printf("查找同步码 0x5A 0xA5...\n");
    unsigned char buf[4096];
    int total = 0;
    int sync_count = 0;
    
    for (int i = 0; i < 100; i++) {
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            total += n;
            for (int j = 0; j < n - 1; j++) {
                if (buf[j] == 0x5A && buf[j+1] == 0xA5) {
                    sync_count++;
                    printf("找到同步码 #%d 在位置 %d, 后续16字节: ", sync_count, total - n + j);
                    for (int k = 0; k < 16 && j+k < n; k++) {
                        printf("%02X ", buf[j+k]);
                    }
                    printf("\n");
                }
            }
        }
        usleep(100000);
    }
    
    printf("\n总共读取 %d 字节，找到 %d 个同步码\n", total, sync_count);
    close(fd);
    return 0;
}

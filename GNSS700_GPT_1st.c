#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_DEVICE "/dev/ttyUSB0"
#define BAUDRATE B115200

typedef struct {
    char utc_time[16];
    double latitude;
    char lat_dir;
    double longitude;
    char lon_dir;
    int fix_quality;
    int satellites;
    float altitude;
} GPGGA_Data;

double nmea_to_decimal(const char* nmea_str) {
    double val = atof(nmea_str);
    int deg = (int)(val / 100);
    double min = val - (deg * 100);
    return deg + min / 60.0;
}

int parse_gpgga(const char* nmea, GPGGA_Data* data) {
    if (strncmp(nmea, "$GPGGA,", 7) != 0) return -1;

    char buf[128];
    strncpy(buf, nmea, sizeof(buf));
    buf[sizeof(buf)-1] = '\0';

    char* token;
    int field = 0;

    token = strtok(buf, ",");
    while (token) {
        switch (field) {
            case 1: strncpy(data->utc_time, token, sizeof(data->utc_time)); break;
            case 2: data->latitude = nmea_to_decimal(token); break;
            case 3: data->lat_dir = token[0]; break;
            case 4: data->longitude = nmea_to_decimal(token); break;
            case 5: data->lon_dir = token[0]; break;
            case 6: data->fix_quality = atoi(token); break;
            case 7: data->satellites = atoi(token); break;
            case 9: data->altitude = atof(token); break;
        }
        token = strtok(NULL, ",");
        field++;
    }
    return 0;
}

int open_serial_port(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);

    options.c_cflag |= (CLOCAL | CREAD);    // Enable receiver
    options.c_cflag &= ~PARENB;             // No parity
    options.c_cflag &= ~CSTOPB;             // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;                 // 8 data bits

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_oflag &= ~OPOST;              // Raw output

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int read_line(int fd, char* buffer, size_t max_len) {
    size_t i = 0;
    char ch;

    while (i < max_len - 1) {
        if (read(fd, &ch, 1) == 1) {
            if (ch == '\n') {
                buffer[i] = '\0';
                return i;
            } else if (ch != '\r') {
                buffer[i++] = ch;
            }
        }
    }
    buffer[i] = '\0';
    return i;
}

int main() {
    int fd = open_serial_port(SERIAL_DEVICE);
    if (fd < 0) return 1;

    char line[256];
    GPGGA_Data gga;

    printf("Listening for GPGGA sentences from GNSS700...\n");

    while (1) {
        if (read_line(fd, line, sizeof(line)) > 0) {
            if (strncmp(line, "$GPGGA,", 7) == 0) {
                if (parse_gpgga(line, &gga) == 0) {
                    printf("\n--- GPGGA ---\n");
                    printf("UTC Time: %s\n", gga.utc_time);
                    printf("Latitude: %.6f %c\n", gga.latitude, gga.lat_dir);
                    printf("Longitude: %.6f %c\n", gga.longitude, gga.lon_dir);
                    printf("Fix Quality: %d\n", gga.fix_quality);
                    printf("Satellites: %d\n", gga.satellites);
                    printf("Altitude: %.1f m\n", gga.altitude);
                }
            }
        }
    }

    close(fd);
    return 0;
}

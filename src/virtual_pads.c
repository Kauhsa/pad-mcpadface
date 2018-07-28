#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <libevdev/libevdev.h>
#include <libevdev/libevdev-uinput.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

// sensors
#define UP_1 0
#define UP_2 1
#define RIGHT_1 2
#define RIGHT_2 3
#define DOWN_1 4
#define DOWN_2 5
#define LEFT_1 6
#define LEFT_2 7

#define SENSORS_LEN 8

int THRESHOLD = 50;

struct pad_state {
    int current_value[SENSORS_LEN];
};

int create_uinput_device(struct libevdev_uinput **uidev) {
    struct libevdev *dev = libevdev_new();
    libevdev_set_name(dev, "Pad McPadFace");
    libevdev_enable_event_type(dev, EV_KEY);
    libevdev_enable_event_code(dev, EV_KEY, BTN_NORTH, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_SOUTH, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_EAST, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_WEST, NULL);

    return libevdev_uinput_create_from_device(
        dev,
        LIBEVDEV_UINPUT_OPEN_MANAGED,
        uidev
    );
}

int create_serial_device(char* port) {
    int fd = open(port, O_RDONLY | O_NOCTTY);

    if (fd < 0) {
       return fd; // error
    }

    struct termios SerialPortSettings;

    tcgetattr(fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings, B2000000);
    cfsetospeed(&SerialPortSettings, B2000000);

    /* no idea what most of this is – or do we care */

    /* 8N1 Mode */
    SerialPortSettings.c_cflag &= ~PARENB; /* Disables the Parity Enable bit (PARENB), So No Parity */
    SerialPortSettings.c_cflag &= ~CSTOPB; /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE; /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8; /* Set the data bits = 8                                 */
    SerialPortSettings.c_cflag &= ~CRTSCTS; /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver, Ignore Modem Control lines       */ 
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
    SerialPortSettings.c_oflag &= ~OPOST; /* No Output Processing */

    /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 1 characters */
    SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */

    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) {
        return -1;
    }

    return fd;
}

int close_serial(int fd) {
    close(fd);
}

int read_serial_byte(int fd) {
    unsigned char read_buffer;
    while (read(fd, &read_buffer, 1) < 1) {}
    return read_buffer;
}

bool is_pressed(int sensor_value_1, int sensor_value_2) {
    return sensor_value_1 > THRESHOLD || sensor_value_2 > THRESHOLD;
}

void process_data(int serial_fd, struct pad_state *pad_state, struct libevdev_uinput *uidev) {
    // wait for the message start byte
    while (read_serial_byte(serial_fd) != 255) {}
    
    int new_values[SENSORS_LEN];
    new_values[UP_1] = read_serial_byte(serial_fd);
    new_values[UP_2] = read_serial_byte(serial_fd);
    new_values[RIGHT_1] = read_serial_byte(serial_fd);
    new_values[RIGHT_2] = read_serial_byte(serial_fd);
    new_values[DOWN_1] = read_serial_byte(serial_fd);
    new_values[DOWN_2] = read_serial_byte(serial_fd);
    new_values[LEFT_1] = read_serial_byte(serial_fd);
    new_values[LEFT_2] = read_serial_byte(serial_fd);

    int up_pressed = is_pressed(new_values[UP_1], new_values[UP_2]);
    int down_pressed = is_pressed(new_values[DOWN_1], new_values[DOWN_2]);
    int left_pressed = is_pressed(new_values[LEFT_1], new_values[LEFT_2]);
    int right_pressed = is_pressed(new_values[RIGHT_1], new_values[RIGHT_2]);

    if (up_pressed != is_pressed(pad_state->current_value[UP_1], pad_state->current_value[UP_2])) {    
        libevdev_uinput_write_event(uidev, EV_KEY, BTN_NORTH, up_pressed ? 1 : 0);
    }

    if (right_pressed != is_pressed(pad_state->current_value[RIGHT_1], pad_state->current_value[RIGHT_2])) {
        libevdev_uinput_write_event(uidev, EV_KEY, BTN_EAST, right_pressed ? 1 : 0);
    }

    if (down_pressed != is_pressed(pad_state->current_value[DOWN_1], pad_state->current_value[DOWN_2])) {
        libevdev_uinput_write_event(uidev, EV_KEY, BTN_SOUTH, down_pressed ? 1 : 0);
    }

    if (left_pressed != is_pressed(pad_state->current_value[LEFT_1], pad_state->current_value[LEFT_2])) {
        libevdev_uinput_write_event(uidev, EV_KEY, BTN_WEST, left_pressed ? 1 : 0);
    }

    libevdev_uinput_write_event(uidev, EV_SYN, SYN_REPORT, 0);
    memcpy(&pad_state->current_value, new_values, sizeof (new_values));
}

int main(int argc, char **argv)
{
    struct libevdev_uinput *uidev;

    struct pad_state pad_state = {
        .current_value = {0, 0, 0, 0, 0, 0, 0, 0}
    };

    int err = create_uinput_device(&uidev);
    int serial_fd = create_serial_device("/dev/ttyACM0");

    if (err != 0) {
        printf("Error creating uinput device\n");
        return -1;
    }

    if (serial_fd < 0) {
        printf("Error creating serial device\n");
        return -1;
    }

    printf("Start\n");
    
    tcflush(serial_fd, TCIFLUSH);

    while (1) {
        process_data(serial_fd, &pad_state, uidev);
    }

    libevdev_uinput_destroy(uidev);
    close_serial(serial_fd);
    printf("Complete\n");
}

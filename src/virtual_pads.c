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
#include <linux/serial.h>

#define LEFT 0
#define UP 1
#define DOWN 2
#define RIGHT 3
#define PANEL_COUNT 4
const unsigned int PANEL_TO_JOY_BUTTON_MAP[] = { BTN_WEST, BTN_NORTH, BTN_SOUTH, BTN_EAST };

#define SENSORS_PER_PANEL_COUNT 2

struct sensor {
    unsigned int value;
};

struct panel {
    struct sensor sensors[SENSORS_PER_PANEL_COUNT];
};

struct pad_state {
    struct panel panels[PANEL_COUNT];
};


int create_uinput_device(struct libevdev_uinput **uidev) {
    struct libevdev *dev = libevdev_new();
    libevdev_set_name(dev, "Pad McPadFace");
    libevdev_enable_event_type(dev, EV_KEY);
    libevdev_enable_event_code(dev, EV_KEY, BTN_WEST, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_SOUTH, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_NORTH, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_EAST, NULL);

    return libevdev_uinput_create_from_device(
        dev,
        LIBEVDEV_UINPUT_OPEN_MANAGED,
        uidev
    );
}

int create_serial_device(const char* port) {
    int fd = open(port, O_RDONLY | O_NOCTTY);

    if (fd < 0) {
       return fd; // error
    }

    /* set serial port to low_latency mode */
    struct serial_struct serial;
    ioctl(fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &serial);

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

bool is_pressed(const struct panel *panel) {
    int THRESHOLD = 50;
    return panel->sensors[0].value > THRESHOLD || panel->sensors[1].value > THRESHOLD;
}

void process_data(int serial_fd, struct pad_state *pad_state, const struct libevdev_uinput *uidev) {
    // wait for the message start byte
    while (read_serial_byte(serial_fd) != 255) {}
    
    struct panel panels[PANEL_COUNT];

    for (int i = 0; i < PANEL_COUNT; i++) {
        for (int j = 0; i < SENSORS_PER_PANEL_COUNT; j++) {
            panels[i].sensors[j].value = read_serial_byte(serial_fd);
        }
    }

    for (int i = 0; i < PANEL_COUNT; i++) {
        bool is_new_state_pressed = is_pressed(&panels[i]);
        if (is_pressed(&pad_state->panels[i]) != is_new_state_pressed) {    
            libevdev_uinput_write_event(uidev, EV_KEY, PANEL_TO_JOY_BUTTON_MAP[i], is_new_state_pressed ? 1 : 0);
        }
    }

    libevdev_uinput_write_event(uidev, EV_SYN, SYN_REPORT, 0);
    memcpy(&pad_state->panels, panels, sizeof (panels));
}

int main(int argc, char **argv)
{
    struct libevdev_uinput *uidev;
    struct pad_state pad_state = { 0 }; // all bytes inside to a 0

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

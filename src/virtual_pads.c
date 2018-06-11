#include <stdio.h>
#include <unistd.h>
#include <libevdev/libevdev.h>
#include <libevdev/libevdev-uinput.h>

int create_uinput_device(struct libevdev_uinput **uidev) {
    struct libevdev *dev = libevdev_new();
    libevdev_set_name(dev, "GREAT DANCING PLATFORM");
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

int main(int argc, char **argv)
{
    struct libevdev_uinput *uidev;
    int err = create_uinput_device(&uidev);

    if (err != 0) {
        return err;
    }

    printf("Start\n");
    
    while (1) {
        libevdev_uinput_write_event(uidev, EV_KEY, BTN_NORTH, 1);
        libevdev_uinput_write_event(uidev, EV_SYN, SYN_REPORT, 0);
        sleep(1);
        libevdev_uinput_write_event(uidev, EV_KEY, BTN_NORTH, 0);
        libevdev_uinput_write_event(uidev, EV_SYN, SYN_REPORT, 0);
        sleep(1);
    }

    libevdev_uinput_destroy(uidev);
    printf("Complete\n");
}

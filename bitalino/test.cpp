
#include "bitalino.h"

#include <thread>
#include <deque>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <mutex>

#include <time.h>
#include <stdio.h>
#include <sys/select.h>

bool keypressed(void)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(0, &readfds);

    timeval readtimeout;
    readtimeout.tv_sec = 0;
    readtimeout.tv_usec = 0;

    return (select(FD_SETSIZE, &readfds, NULL, NULL, &readtimeout) == 1);
}

//#define SAMPLES 300000 // 5 min
#define SAMPLES 95000

int run = 0;

BITalino::VFrame frames(SAMPLES); // initialize the frames vector with 3000000 frames (5min = 300 sec)

//==============================================================================
char* get_filename(const char* suffix)
{
    time_t curr_time = time(NULL);
    struct tm start_time = *localtime(&curr_time);
    char* filename = NULL;
    filename = (char*)calloc(64, sizeof(char));
    if (filename == NULL) {
        fprintf(stderr, "get_filename\n");
        exit(EXIT_FAILURE);
    }

    sprintf(filename, "%4d-%2d-%2d_%2d-%2d-%2d-%s.log",
        start_time.tm_year + 1900, start_time.tm_mon + 1, start_time.tm_mday,
        start_time.tm_hour, start_time.tm_min, start_time.tm_sec,
        suffix);

    const int length = 5;
    int pos[] = { 5, 8, 11, 14, 17 };
    for (int i = 0; i < length; ++i) {
        if (filename[pos[i]] == ' ')
            filename[pos[i]] = '0';
    }

    return filename;
}
//==============================================================================
// TODO re-implement it since it was taken from stackoverflow.com
// took from here
// https://stackoverflow.com/questions/8304259/formatting-struct-timespec/14746954#14746954
int timespec2str(char* buf, struct timespec* ts)
{
    int ret;
    struct tm t;
    const size_t len = sizeof("2015-12-31 12:59:59.123456789") + 1;
    const size_t len_nano = sizeof(".123456789") + 1;

    if (ts->tv_nsec > 1000000000L) {
        ts->tv_sec = ts->tv_nsec / 1000000000L;
        ts->tv_nsec = ts->tv_nsec % 1000000000L;
    }

    tzset();
    if (localtime_r(&(ts->tv_sec), &t) == NULL)
        return 1;

    ret = strftime(buf, len, "%F %T", &t);
    if (ret == 0)
        return 2;

    ret = snprintf(&buf[len - len_nano], len_nano, ".%09ld", ts->tv_nsec);
    if (ret >= len_nano)
        return 3;

    return 0;
}

//==============================================================================
void log2File()
{

    while (run != 1)
        ;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    char* filename = get_filename("EMG");
    FILE* fp = fopen(filename, "w");
    // Open file for writing, no need to fclose, OS will do it
    if (fp == NULL) {
        fprintf(stderr, "Cannot open current_file.txt for writing\n");
        exit(EXIT_FAILURE); // TODO send sigint to main()
    }

    struct timespec now;

    if (clock_gettime(CLOCK_REALTIME, &now) != 0) { // is it a good practice ?
        fprintf(stderr, "clock_gettime, energy");
        exit(EXIT_FAILURE);
    }
    char buf[] = "2015-12-31 12:59:59.123456789";
    timespec2str(buf, &now);
    fprintf(fp, "now: %s\n", buf);
    fprintf(fp, "timestamp\tEMG\n");

    for (uint64_t i = 0; i < SAMPLES; i++) { // 5 min

        struct timespec toc;
        clock_gettime(CLOCK_REALTIME, &toc);

        toc.tv_sec = toc.tv_sec - now.tv_sec;
        toc.tv_nsec = toc.tv_nsec - now.tv_nsec;
        if (toc.tv_nsec < 0) {
            toc.tv_nsec += 1000000000L;
            toc.tv_sec--;
        }

        const BITalino::Frame& f = frames[i];

        fprintf(fp, "%d.%09d %d %d %d\n", toc.tv_sec, toc.tv_nsec,
            f.seq, f.analog[0], f.analog[1]);

        if (i % 20 == 0) {
            fflush(fp);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "EMG DONE";
}

int main()
{
    run = 0;
    std::thread logger(log2File);

    try {
        // uncomment this block to search for Bluetooth devices (Windows and Linux)
        /*
      BITalino::VDevInfo devs = BITalino::find();
      for(int i = 0; i < devs.size(); i++)
      	printf("%s - %s\n", devs[i].macAddr.c_str(), devs[i].name.c_str());
      return 0;
      */

        puts("Connecting to device...");

        // use one of the lines below
        BITalino dev("98:D3:31:70:3E:06"); // device MAC address (Windows and Linux)

        //BITalino dev("COM5");  // Bluetooth virtual COM port or USB-UART COM port (Windows)

        //BITalino dev("/dev/ttyUSB0");  // USB-UART device (Linux)
        //BITalino dev("/dev/rfcomm0");  // Bluetooth virtual serial port (Linux)

        //BITalino dev("/dev/tty.usbserial-A1000QIz");  // USB-UART device (Mac OS)
        //BITalino dev("/dev/tty.bitalino-DevB");  // Bluetooth virtual serial port (Mac OS)

        puts("Connected to device. Press Enter to exit.");

        std::string ver = dev.version(); // get device version string
        printf("BITalino version: %s\n", ver.c_str());

        dev.battery(10); // set battery threshold (optional)

        dev.start(1000, { 0, 1 }); // start acquisition of all channels at 1000 Hz

        dev.trigger({ false, false, true, false });
        // use block below if your compiler doesn't support vector initializer lists
        run = 1;
        do {

            dev.read(frames); // get frames from device
            break;

        } while (!keypressed()); // until a key is pressed

        dev.stop(); // stop acquisition
    } // dev is destroyed here (it goes out of scope)
    catch (BITalino::Exception& e) {
        printf("BITalino exception: %s\n", e.getDescription());
    }

    logger.join();

    return 0;
}

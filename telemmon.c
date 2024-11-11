#include <iostream>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

struct Telemetry {
    double latitude;
    double longitude;
    double altitude;
    double airspeed;
};

void cleanConsole() {
    system("clear");
}

int main() {
    int shm_fd = shm_open("/telemetry", O_RDONLY, 0666);
    if (shm_fd == -1) {
        std::cerr << "Shared memory open failed\n";
        return 1;
    }

    Telemetry* telemetry = static_cast<Telemetry*>(mmap(0, sizeof(Telemetry), PROT_READ, MAP_SHARED, shm_fd, 0));
    if (telemetry == MAP_FAILED) {
        std::cerr << "Memory mapping failed\n";
        close(shm_fd);
        return 1;
    }

    Telemetry previousTelemetry;
    memset(&previousTelemetry, 0, sizeof(Telemetry));

    while (true) {
        if (std::memcmp(telemetry, &previousTelemetry, sizeof(Telemetry)) != 0) {
            cleanConsole();
            std::cout << "Latitude: " << telemetry->latitude
                      << ", Longitude: " << telemetry->longitude
                      << ", Altitude: " << telemetry->altitude
                      << ", Airspeed: " << telemetry->airspeed << " knots\r";
            std::cout.flush();
            std::memcpy(&previousTelemetry, telemetry, sizeof(Telemetry));
        }
        //usleep(2000); // Sleep for 200ms
    }

    munmap(telemetry, sizeof(Telemetry));
    close(shm_fd);
    return 0;
}


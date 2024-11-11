#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

enum FlapSetting {
    UP = 0,
    TEN_DEGREES = 10,
    TWENTY_DEGREES = 20,
    THIRTY_DEGREES = 30,
    FORTY_DEGREES = 40
};

class Waypoint {
public:
    double latitude, longitude, altitude;
    double airspeed; // in knots

    Waypoint(double lat, double lon, double alt, double speed) :
        latitude(lat), longitude(lon), altitude(alt), airspeed(speed) {}
};

class FlightPlan {
    std::vector<Waypoint> waypoints;
public:
    void loadFromCSV(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string latStr, lonStr, altStr, speedStr;
            if (std::getline(ss, latStr, ',') &&
                std::getline(ss, lonStr, ',') &&
                std::getline(ss, altStr, ',') &&
                std::getline(ss, speedStr, ',')) {
                double lat = std::stod(latStr);
                double lon = std::stod(lonStr);
                double alt = std::stod(altStr);
                double speed = std::stod(speedStr);
                waypoints.emplace_back(lat, lon, alt, speed);
            }
        }
    }

    const std::vector<Waypoint>& getWaypoints() const {
        return waypoints;
    }
};

struct Telemetry {
    double latitude;
    double longitude;
    double altitude;
    double airspeed;
    double pitchAngle; // Add this line
    double rollAngle;  // Add this line
    double heading;
    
};


class Aircraft {
    double currentLatitude, currentLongitude, currentAltitude;
    double weight, payloadCapacity, payloadWeight;
    double maxAirspeed; // in knots
    double stallSpeed; // in knots
    double throttle; // 0 to 100 %
    FlapSetting flaps;
    bool landingGearDeployed;
    double waypointTolerance; // in meters

    double previousHeading; // Declare this variable

    
    int shm_fd;
    Telemetry* telemetry;

public:
    Aircraft(double w, double pc, double pw) :
        weight(w), payloadCapacity(pc), payloadWeight(pw),
        maxAirspeed(420), stallSpeed(115), throttle(100),
        flaps(UP), landingGearDeployed(false), waypointTolerance(100.0) {
        shm_fd = shm_open("/telemetry", O_CREAT | O_RDWR, 0666);
        ftruncate(shm_fd, sizeof(Telemetry));
        telemetry = static_cast<Telemetry*>(mmap(0, sizeof(Telemetry), PROT_WRITE, MAP_SHARED, shm_fd, 0));
    }

    ~Aircraft() {
        munmap(telemetry, sizeof(Telemetry));
        close(shm_fd);
        shm_unlink("/telemetry");
    }

    void setFlaps(FlapSetting setting) { flaps = setting; }
    void deployLandingGear() { landingGearDeployed = true; }
    void retractLandingGear() { landingGearDeployed = false; }

    void fly(const FlightPlan& flightPlan) {
        for (size_t i = 0; i < flightPlan.getWaypoints().size() - 1; ++i) {
            auto& wp1 = flightPlan.getWaypoints()[i];
            auto& wp2 = flightPlan.getWaypoints()[i + 1];
            moveBetweenWaypoints(wp1, wp2);
        }
    }
// Function to smooth pitch angle transition
double calculateSmoothPitchAngle(double currentAltitude, double targetAltitude, double currentDistance, double totalDistance) {
    double altitudeDifference = targetAltitude - currentAltitude;
    double smoothDistance = currentDistance / totalDistance;
    double smoothAltitude = altitudeDifference * smoothDistance;
    double pitchAngle = std::atan2(smoothAltitude, currentDistance) * 180 / M_PI; // in degrees
    return pitchAngle;
}

void moveBetweenWaypoints(const Waypoint& wp1, const Waypoint& wp2) {
    static double previousHeading = 0; // Ensure previousHeading is declared

    double distance = calculateDistance(wp1, wp2);
    std::cout << "Starting waypoint: " << wp1.latitude << ", " << wp1.longitude << " at altitude: " << wp1.altitude << "\n";
    std::cout << "Target waypoint: " << wp2.latitude << ", " << wp2.longitude << " at altitude: " << wp2.altitude << "\n";
    std::cout << "Initial distance to target: " << distance << " meters\n";

    double airspeedStart = std::max(stallSpeed, std::min(maxAirspeed, wp1.airspeed * 0.514444));
    double airspeedEnd = std::max(stallSpeed, std::min(maxAirspeed, wp2.airspeed * 0.514444));
    double duration = distance / ((airspeedStart + airspeedEnd) / 2);

    auto startTime = std::chrono::system_clock::now();
    auto endTime = startTime + std::chrono::seconds(static_cast<int>(duration));

    while (std::chrono::system_clock::now() < endTime) {
        double elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - startTime).count();

        double fraction = elapsed / duration;
        currentLatitude = wp1.latitude + fraction * (wp2.latitude - wp1.latitude);
        currentLongitude = wp1.longitude + fraction * (wp2.longitude - wp1.longitude);
        currentAltitude = wp1.altitude + fraction * (wp2.altitude - wp1.altitude);
        
        double currentAirspeed = airspeedStart + fraction * (airspeedEnd - airspeedStart);
        currentAirspeed = std::max(stallSpeed, std::min(maxAirspeed, currentAirspeed));
        
        double dragFactor = calculateDragFactor();
        currentAirspeed /= dragFactor;

        // Calculate heading
        double heading = calculateHeading(wp1, wp2);

        // Calculate roll angle conditionally
        double rollAngle = (heading != previousHeading) ? calculateRollAngle(wp1, wp2) : 0;
        previousHeading = heading;

        // Calculate smooth pitch angle using the smooth transition formula
        double distanceToNextWaypoint = calculateDistanceToWaypoint(currentLatitude, currentLongitude, currentAltitude, wp2);
        double pitchAngle = calculateSmoothPitchAngle(currentAltitude, wp2.altitude, distanceToNextWaypoint, distance);

        telemetry->latitude = currentLatitude;
        telemetry->longitude = currentLongitude;
        telemetry->altitude = currentAltitude;
        telemetry->airspeed = currentAirspeed / 0.514444;
        telemetry->pitchAngle = pitchAngle;
        telemetry->rollAngle = rollAngle;
        telemetry->heading = heading;

        std::cout << "Current location: " << telemetry->latitude << ", " << telemetry->longitude
            << " at altitude: " << telemetry->altitude << " with speed: " << telemetry->airspeed << " knots"
            << ", pitch angle: " << telemetry->pitchAngle << " degrees"
            << ", roll angle: " << telemetry->rollAngle << " degrees"
            << ", heading: " << telemetry->heading << " degrees\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        distanceToNextWaypoint = calculateDistanceToWaypoint(currentLatitude, currentLongitude, currentAltitude, wp2);
        std::cout << "Distance to next waypoint: " << distanceToNextWaypoint << " meters\n";
        
        if (distanceToNextWaypoint <= 100) { // 100 meters tolerance
            std::cout << "Waypoint achieved: " << wp2.latitude << ", " << wp2.longitude << " at altitude: " << wp2.altitude << "\n";
            break; // Move to the next waypoint
        }
    }
    std::cout << "Finished moving to waypoint.\n";
}

// New method to calculate heading
double calculateHeading(const Waypoint& wp1, const Waypoint& wp2) {
    double y = std::sin(wp2.longitude - wp1.longitude) * std::cos(wp2.latitude);
    double x = std::cos(wp1.latitude) * std::sin(wp2.latitude) - std::sin(wp1.latitude) * std::cos(wp2.latitude) * std::cos(wp2.longitude - wp1.longitude);
    double heading = std::atan2(y, x) * 180 / M_PI;
    return heading < 0 ? heading + 360 : heading;
}

// New method to calculate roll angle
double calculateRollAngle(const Waypoint& wp1, const Waypoint& wp2) {
    double latDifference = wp2.latitude - wp1.latitude;
    double lonDifference = wp2.longitude - wp1.longitude;
    double rollAngle = std::atan2(latDifference, lonDifference) * 180 / M_PI;
    if (rollAngle > 180) {
        rollAngle -= 360;
    } else if (rollAngle < -180) {
        rollAngle += 360;
    }
    return rollAngle;
}

    double calculateDragFactor() {
        double dragFactor = 1.0;

        switch (flaps) {
            case TEN_DEGREES: dragFactor += 0.10; break; // 10% drag increase
            case TWENTY_DEGREES: dragFactor += 0.20; break; // 20% drag increase
            case THIRTY_DEGREES: dragFactor += 0.35; break; // 35% drag increase
            case FORTY_DEGREES: dragFactor += 0.50; break; // 50% drag increase
            default: break;
        }

        if (landingGearDeployed) {
            dragFactor += 0.20; // 20% drag increase due to landing gear
        }

        return dragFactor;
    }

    double calculateDistance(const Waypoint& wp1, const Waypoint& wp2) {
        double lat1 = wp1.latitude * M_PI / 180.0;
        double lon1 = wp1.longitude * M_PI / 180.0;
        double lat2 = wp2.latitude * M_PI / 180.0;
        double lon2 = wp2.longitude * M_PI / 180.0;

        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;

        double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                   std::cos(lat1) * std::cos(lat2) *
                   std::sin(dLon / 2) * std::sin(dLon / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        const double R = 6371e3; // Earth's radius in meters
        double distance2D = R * c;

        double dAlt = wp2.altitude - wp1.altitude;
        return std::sqrt(distance2D * distance2D + dAlt * dAlt);
    }

    

    double calculateDistanceToWaypoint(double lat, double lon, double alt, const Waypoint& wp) {
        double lat1 = lat * M_PI / 180.0;
        double lon1 = lon * M_PI / 180.0;
        double lat2 = wp.latitude * M_PI / 180.0;
        double lon2 = wp.longitude * M_PI / 180.0;

        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;

        double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                   std::cos(lat1) * std::cos(lat2) *
                   std::sin(dLon / 2) * std::sin(dLon / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        const double R = 6371e3; // Earth's radius in meters
        double distance2D = R * c;

        double dAlt = wp.altitude - alt;
        return std::sqrt(distance2D * distance2D + dAlt * dAlt);
    }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <flight_plan_file>\n";
        return 1;
    }

    std::string filename = argv[1];

    FlightPlan flightPlan;
    flightPlan.loadFromCSV(filename);

    Aircraft aircraft(1000, 500, 300); // Example values: weight, payloadCapacity, payloadWeight
    aircraft.setFlaps(TWENTY_DEGREES); // For testing, set flaps
    aircraft.deployLandingGear(); // For testing, deploy landing gear

    aircraft.fly(flightPlan);

    return 0;
}


#include <iostream>
#include <signal.h>
#include <fstream>

#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>

#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <framework/ProgramArguments.hpp>
#include <framework/SamplesDataPath.hpp>
#include <framework/Log.hpp>
#include <framework/Checks.hpp>

#include <dw/core/logger/Logger.h>
#include <dw/core/context/Context.h>
#include <dw/core/VersionCurrent.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/gps/GPS.h>

#include <include/curl/curl.h>


static bool gRun = true;

// Mengukur Jarak dua titik
using namespace std;

double distance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371e3; // earth radius in meters
    double phi1 = lat1 * M_PI / 180; // convert to radians
    double phi2 = lat2 * M_PI / 180;
    double dphi = (lat2 - lat1) * M_PI / 180;
    double dlambda = (lon2 - lon1) * M_PI / 180;
    double aa = sin(dphi/2) * sin(dphi/2) +
               cos(phi1) * cos(phi2) *
               sin(dlambda/2) * sin(dlambda/2);
    double cc = 2 * atan2(sqrt(aa), sqrt(1-aa));
    return R * cc;
}

struct Station{
    std::string nameSign;
    double latitudeSign;
    double longitudeSign;
};

void sig_int_handler(int)
{
    gRun = false;
}

int main(int argc, const char** argv)
{
	
    std::cout<<"BEGINNING OF THE PROGRAM"<<std::endl;

#ifndef WINDOWS
    struct sigaction action = {};
    action.sa_handler       = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D untuk menutup terminal
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\ untuk keluar dengan core dump
    sigaction(SIGABRT, &action, NULL); // Memanggil fungsi abort()
    sigaction(SIGTERM, &action, NULL); // Menghentikan command
#endif

    // Baca Data Rambu
    ifstream datafileSign("RambuITB.txt"); // Membaca data Rambu
    if (!datafileSign.is_open()) { // Mengecek apakah file dapat dibuka
        cout << "Tidak ada data rambu" << endl;
        return 1;
    }

    std::vector<Station> stations;
    std::string lineSign;
    while (std::getline(datafileSign, lineSign)) {
        std::stringstream aa(lineSign);
        std::string name, latStr, lonStr;
        std::getline(aa, name, ',');
        std::getline(aa, latStr, ',');
        std::getline(aa, lonStr, ',');
        
        double latSign = std::stod(latStr);
        double lonSign = std::stod(lonStr);
        
        stations.push_back({name, latSign, lonSign});
    }

    gRun = true;

    ProgramArguments arguments(
        {ProgramArguments::Option_t("driver", "gps.novatel"),
         ProgramArguments::Option_t("params", (std::string("file=") + dw_samples::SamplesDataPath::get() + "/samples/sensors/gps/1.gps").c_str())});

    if (!arguments.parse(argc, argv) || (!arguments.has("driver") && !arguments.has("params")))
    {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << "\t--driver=gps.virtual \t\t\t: one of the available GPS drivers "
                  << "(see sample_sensors_info)\n";
        std::cout << "\t--params=file=file.gps,arg2=value \t: comma separated "
                  << "key=value parameters for the sensor "
                  << "(see sample_sensor_info for a set of supported parameters)\n";

        return -1;
    }
    dwContextHandle_t sdk = DW_NULL_HANDLE;
    dwSALHandle_t hal     = DW_NULL_HANDLE;

	// Membuat logger ke console log
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // Inisialisasi Driveworks SDK
    dwContextParameters sdkParams = {};

    CHECK_DW_ERROR(dwInitialize(&sdk, DW_VERSION, &sdkParams));

    // Membuat modul HAL dari SDK
    dwSAL_initialize(&hal, sdk);

	// Membuka sensor GPS dua kali untuk membuktikan kapabilitas pengiriman data sensor
    dwSensorHandle_t gpsSensor[2] = {DW_NULL_HANDLE, DW_NULL_HANDLE};
    for (int32_t i = 0; i < 2; i++)
    {
        dwSensorParams params{};
        std::string parameterString = arguments.get("params");
        params.parameters           = parameterString.c_str();
        params.protocol             = arguments.get("driver").c_str();
        if (dwSAL_createSensor(&gpsSensor[i], params, hal) != DW_SUCCESS)
        {
            std::cout << "Cannot create sensor " << params.protocol
                      << " with " << params.parameters << std::endl;

            dwSAL_release(hal);
            dwRelease(sdk);
            dwLogger_release();

            return -1;
        }
    }

    gRun = gRun && dwSensor_start(gpsSensor[0]) == DW_SUCCESS;
    gRun = gRun && dwSensor_start(gpsSensor[1]) == DW_SUCCESS;

    // Pesan yang dibuat
    bool sensorRun[2] = {gRun, gRun};
    while (gRun)
    {

        if (!sensorRun[0] && !sensorRun[1]){
            break;
        }

        for (int i = 0; i < 2; i++)
        {
            if (!sensorRun[i]){
                continue;
            }
            dwGPSFrame frame;
            dwStatus status = DW_FAILURE;
            status = dwSensorGPS_readFrame(&frame, 50000, gpsSensor[i]);
            if (status == DW_END_OF_STREAM)
            {
                std::cout << "GPS[" << i << "] selesai" << std::endl;
                sensorRun[i] = false;
                break;
            }
           
            // Pesan keluaran log
            std::cout << "GPS[" << i << "] - " << frame.timestamp_us;
            {
				
				std::cout << std::setprecision(10); 
				
				if (frame.flags & DW_GPS_LAT){
                    std::cout << " latitude: " << frame.latitude << std::endl; 
                }
                    

                if (frame.flags & DW_GPS_LON){
                    std::cout << " longitude: " << frame.longitude << std::endl; 
                }
                
				std::vector<Station> nearbyStations;
                for (const auto& station : stations) {
                    double dist_sign = distance(frame.latitude, frame.longitude, station.latitudeSign, station.longitudeSign);
                    // Jika jarak Rambu di bawah 50 meter
					if (dist_sign <= 50.0) {
                        nearbyStations.push_back(station);
                        if (frame.flags & DW_GPS_LAT){ // Print hasil pengukuran jarak dengan rambu
                            std::cout << station.nameSign << " berada sejauh " << dist_sign << " meter" << endl;
                        }
                    }
                }
                    
            }
            std::cout << std::endl;
        }
    }

    dwSensor_stop(gpsSensor[0]);
    dwSensor_stop(gpsSensor[1]);
    dwSAL_releaseSensor(gpsSensor[0]);
    dwSAL_releaseSensor(gpsSensor[1]);

    // Melepas komponen yang telah dipakai
    dwSAL_release(hal);
    dwRelease(sdk);
    dwLogger_release();

    return 0;
}

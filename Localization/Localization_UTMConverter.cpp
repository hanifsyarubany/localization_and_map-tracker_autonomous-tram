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
            
            // UTM Converter
            double sa = 6378137.000000, sb = 6356752.314245;
            double e2 = ( ( ( sa * sa ) - ( sb * sb ) ) / ( sb * sb ) );
            double e2square = e2 * e2;
            double c = ( sa * sa ) / sb;
            double latX = frame.latitude * ( M_PI / 180.0 );
            double lonY = frame.longitude * ( M_PI / 180.0 );
            int zone = static_cast<int>( ( frame.longitude / 6.0 ) + 31 );
            double S = ( ( zone * 6.0 ) - 183.0 );
            double deltaS = lonY - ( S * ( M_PI / 180.0 ) );
            double a = cos(latX) * sin(deltaS);
            double epsilon = 0.5 * log( ( 1 +  a) / ( 1 - a ) );
            double nu = atan( tan(latX) / cos(deltaS) ) - latX;
            double v = ( c / sqrt( ( 1 + ( e2square * ( cos(latX) * cos(latX) ) ) ) ) ) * 0.9996;
            double ta = ( e2square / 2 ) * epsilon * epsilon * ( cos(latX) * cos(latX) );
            double a1 = sin( 2 * latX );
            double a2 = a1 * ( cos(latX) * cos(latX) );
            double j2 = latX + ( a1 / 2.0 );
            double j4 = ( ( 3 * j2 ) + a2 ) / 4.0;
            double j6 = ( ( 5 * j4 ) + ( a2 * ( cos(latX) ) * ( cos(latX) ) ) ) / 3;
            double alpha = ( 3 / 4 ) * e2square;
            double beta = ( 5 / 3 ) * alpha * alpha;
            double gama = ( 35 / 27 ) * alpha* alpha * alpha;
            double Bm = 0.9996 * c * ( latX - alpha * j2 + beta * j4 - gama * j6 );
            double east = epsilon * v * ( 1 + ( ta / 3 ) ) + 500000;
            double north = nu * v * ( 1 + ta ) + Bm;
            if (north<0){
                north = 9999999+ north;
            }
            // end UTM Converter, hasil berupa nilai koordinat easting dan northing

            {
                std::cout << std::setprecision(10);     
				
				if (frame.flags & DW_GPS_LAT){
                    std::cout << " latitude: " << frame.latitude << std::endl; 
                }
                    

                if (frame.flags & DW_GPS_LON){
                    std::cout << " longitude: " << frame.longitude << std::endl; 
                }

                if (frame.flags & DW_GPS_LAT){
                    std::cout << "easting: " << east << std::endl; // Print Hasil Koordinat Easting
                }

                if (frame.flags & DW_GPS_LON){
                    std::cout << "northing: " << north << std::endl; // Print Hasil Koordinat Northing
                }

                if (frame.flags & DW_GPS_LON){
                    std::cout << "zone: " << zone << std::endl; // Print Hasil Zona Koordinat UTM
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

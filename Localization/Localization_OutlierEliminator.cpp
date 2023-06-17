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

// Data data
double koordinat [15][2]= {
            {788654.9599202528,9237513.835627956},
            {788656.5028385923,9237513.953669308},
            {788657.9954732872,9237514.054600881},
            {788659.3857368545,9237514.134638747},
            {788660.6578986855,9237514.184087623},
            {788661.7998847761,9237514.197574578},
            {788662.8219115194,9237514.19973123},
            {788663.7649673435,9237514.190295609},
            {788659.3857368545,9237514.134638747},
            {788660.6578986855,9237514.184087623},
            {788661.7998847761,9237514.197574578},
            {788662.8219115194,9237514.19973123},
            {788663.7649673435,9237514.190295609},
            {788664.7079729761,9237514.156939602},
            {788665.6282399793,9237514.113721922}};
double sigma_y =    0.0;     //ini udah bener
double east = 0.0;           // Ini masukan easting dari program zidan
double north = 0.0 ;         // ini masukan north dari zidan
const int n = 14;            //Banyak koordinat yang diolah
float waktu = 0.1;           //Ini periode sampling 
double jarak[n];             //Untuk masukin jumlah jarak di koordinat
float dat[n][2] = {
  { 0, 2.1 },
  { (waktu*1), 7.7 },
  { (waktu*2), 13.6 },
  { (waktu*3), 27.2 },
  { (waktu*4), 40.9 },
  { (waktu*5), 61.1 },
  { (waktu*6), 61.1 },
  { (waktu*7), 61.1 },
  { (waktu*8), 61.1 },
  { (waktu*9), 40.9 },
  { (waktu*10), 61.1 },
  { (waktu*11), 61.1 },
  { (waktu*12), 61.1 },
  { (waktu*13), 61.1 }};
float predsat = 0.0;         //Inisial prediksi
int parameter_start = 12;    // Jumlah data untuk prediksi = n - parameter_start


void puter (double eastingValues, double northingValues) {
    
  for (int i = 0; i < 14; ++i) {
    koordinat[i][0] = koordinat[i+1][0];
    koordinat[i][1] = koordinat[i+1][1];
    
  }
  koordinat[14][0]= eastingValues;
  koordinat[14][1] = northingValues;
}

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

            //Memasukkan koordinat tangkapan GNSS baru ke daftar        
            puter(east,north); 

            //Dari data koordinat baru yang masuk dideteksi apakah outlier dengan prediksi sebelumnya, output: prediksi & flag
            float satu = sqrt(pow((koordinat[n][0]-koordinat[n-1][0]), 2)+pow((koordinat[n][1]-koordinat[n-1][1]), 2));
            cout << "\n";
            cout  << "Prediksi : " << predsat << " kenyataan: "<< satu << "\n" ;

            if (satu-predsat > 5)
            { cout << "outlier\n";}
            else
            { cout << "Bukan outlier\n";}

            //Koordinat => dat, ya sama aja cuma pas dikopi dari internet gajalan
            sigma_y = 0.0;
            for (int i = 0; i < n; i++) {
                    jarak[i] = sigma_y + sqrt(pow((koordinat[i+1][0]-koordinat[i][0]), 2)+pow((koordinat[i+1][1]-koordinat[i][1]), 2));
                    sigma_y = jarak[i];
                    dat[i][1] = jarak[i];
                }

            //Memasukkan data kordinat ke matriks least square gaussian
            cout.precision(2);
            cout.setf(ios::fixed);
            float sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
            float sumY = 0, sumXY = 0, sumX2Y = 0;

            for (int i = parameter_start; i < n; ++i) {
                sumX += dat[i][0];
                sumX2 += pow(dat[i][0], 2);
                sumX3 += pow(dat[i][0], 3);
                sumX4 += pow(dat[i][0], 4);
                sumY += dat[i][1];
                sumXY += dat[i][0] * dat[i][1];
                sumX2Y += pow(dat[i][0], 2) * dat[i][1];
            }

            float X[3][3] = {
                n,      sumX,   sumX2,
                sumX,   sumX2,  sumX3,
                sumX2,  sumX3,  sumX4
            };

            float Y[3] = {
                sumY,
                sumXY,
                sumX2Y
            };

           // printGaussMatrix(X, Y);
            
            // Melakukan eliminasi gauss sehingga didapat A[0], A[1] dan A[2]
            const int XROWS = 3;
            const int XCOLS = 3;
            //const int YROWS = 3;
            for (int i = 0; i < XCOLS - 1; ++i) {
                for (int j = i + 1; j < XROWS; ++j) {
                float d = X[j][i] / X[i][i];

                for (int k = 0; k < XCOLS; ++k) {
                    X[j][k] = X[j][k] - X[i][k] * d;
                }

                Y[j] = Y[j] - Y[i] * d;


                //printGaussMatrix(X, Y);
                }
            }

            float A[3]; // i => 0, 1, 2
            float s;

            for (int i = 2; i >= 0; i--) {
                s = 0;
                if (i < 2) {
                for (int j = i + 1; j <= 2; ++j) {
                    s += X[i][j] * A[j];
                }
                }
                A[i] = (Y[i] - s) / X[i][i];

            }



            //Hasil prediksi berupa jarak 
            predsat = A[0]*(n)*waktu + A[1]*(n)*waktu + A[2]*(n)*waktu - A[0]*(n-1)*waktu - A[1]*(n-1)*waktu - A[2]*(n-1)*waktu;

			
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

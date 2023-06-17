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

// Fungsi Pengukuran Jarak 2 titik
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

class DoubleStreamBuf : public std::streambuf {
public:
  DoubleStreamBuf(char *buf, std::size_t size, int precision) : m_buf(buf), m_size(size), m_precision(precision) {}


protected:
  virtual std::streamsize xsputn(const char *s, std::streamsize n) override {
    if (n > m_size) {
      n = m_size;
    }
    std::memcpy(m_buf, s, n);
    m_buf += n;
    m_size -= n;
    return n;
  }


  virtual int overflow(int c) override {
    if (c == EOF) {
      return 0;
    }
    if (m_size <= 1) {
      return EOF;
    }
    *m_buf++ = c;
    *m_buf = '\0';
    --m_size;
    return c;
  }


private:
  char *m_buf;
  std::size_t m_size;
  int m_precision;
};


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

    // OPEN FILE
    std::ofstream outfile;
    outfile.open("execution_times_pp.txt"); // open the file
    if (!outfile.is_open()) { // check if the file is open
        std::cout << "Failed to open file." << std::endl;
        return 1;
    }
// Membaca data waypoint 
    ifstream datafile("DataNovatel.txt"); // Membaca data txt waypoint dengan Novatel
    if (!datafile.is_open()) { // Mengecek apakah file dapat dibuka
        cout << "Tidak ada data waypoint" << endl;
        return 1;
    }

    vector<double> latitudeWay;
    vector<double> longitudeWay;
    string lineWay;
    while (getline(datafileWay, lineWay)) { // read line by line
        stringstream ss(lineWay);
        string lat_str, lon_str;
        getline(ss, lat_str, ','); // split line by comma
        getline(ss, lon_str, ',');
        double latWay = stod(lat_str);
        double lonWay = stod(lon_str);
        latitudeWay.push_back(latWay);
        longitudeWay.push_back(lonWay);
    }
	
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
	
    ifstream datafile("Novatel1.txt"); // change filename to the CSV file
    if (!datafile.is_open()) { // check if file is open
        cout << "Tidak ada data file" << endl;
        return 1;
    }


    vector<double> latitude;
    vector<double> longitude;
    string line;
    while (getline(datafile, line)) { // read line by line
        stringstream ss(line);
        string lat_str, lon_str;
        getline(ss, lat_str, ','); // split line by comma
        getline(ss, lon_str, ',');
        double latWay = stod(lat_str);
        double lonWay = stod(lon_str);
        latitude.push_back(latWay);
        longitude.push_back(lonWay);
    }

    // Initialize libcurl
    curl_global_init(CURL_GLOBAL_ALL);


    // Create a curl instance
    CURL* curl = curl_easy_init();


    // Set the target endpoint
    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:8076/grs");


    // Set the HTTP method to POST
    curl_easy_setopt(curl, CURLOPT_POST, 1L);


    // Set the content-type header
    struct curl_slist* headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

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
			
			// Positioning Correction, mencari titik terdekat di waypoint dari data posisi yang diperoleh
            double nearest_lat = frame.latitude;
			double nearest_lon = frame.longitude;
            double nearest_dist = INFINITY;
            for (size_t i = 0; i < latitudes.size(); i++) {
                double lat = latitudes[i];
                double lon = longitudes[i];
                double dist = distance(frame.latitude, frame.longitude, lat, lon);
                if (dist < nearest_dist) {
                    nearest_lat = lat; // keluaran latitude hasil Positioning Correction
                    nearest_lon = lon; // keluaran longitude hasil Positioning Correction
					nearest_dist = dist;
                }
            }
			// end Positioning Correction (03)


			// convert the latitude and longitude values to strings with guaranteed precision
			char lat_buf[32], lon_buf[32];
			DoubleStreamBuf lat_sbuf(lat_buf, sizeof(lat_buf), 11);
			DoubleStreamBuf lon_sbuf(lon_buf, sizeof(lon_buf), 11);
			std::ostream lat_os(&lat_sbuf), lon_os(&lon_sbuf);
			lat_os << std::fixed << std::setprecision(11) << nearest_lat;
			lon_os << std::fixed << std::setprecision(11) << nearest_lon;
			  // create a JSON payload string with the latitude and longitude values
			std::stringstream ss;
			ss << "{\"latitude\": " << lat_buf << ", \"longitude\": " << lon_buf << "}";
			std::string payload = ss.str();
			// Set the payload data
			curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
			// Send the request and print the response
			CURLcode res = curl_easy_perform(curl);
			if (res != CURLE_OK) {
				cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << endl;
			}
			cerr << payload <<endl;
			}
			
			// UTM Converter
            double sa = 6378137.000000, sb = 6356752.314245;
            double e2 = ( ( ( sa * sa ) - ( sb * sb ) ) / ( sb * sb ) );
            double e2square = e2 * e2;
            double c = ( sa * sa ) / sb;
            double latX = nearest_lat * ( M_PI / 180.0 );
            double lonY = nearest_lon * ( M_PI / 180.0 );
            int zone = static_cast<int>( ( nearest_lon / 6.0 ) + 31 );
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
            // end UTM Converter, mendapatkan hasil berupa nilai easting dan northing

            {
                std::cout << std::setprecision(10);
				
				if (frame.flags & DW_GPS_LAT){
                    std::cout << " latitude: " << frame.latitude; 
                }
                    

                if (frame.flags & DW_GPS_LON){
                    std::cout << " longitude: " << frame.longitude; 
                }

                if (frame.flags & DW_GPS_LAT){
                    std::cout << " latitude (new): " << nearest_lat; // Print Hasil Latitude terbaru
                }
                    

                if (frame.flags & DW_GPS_LON){
                    std::cout << " longitude (new): " << nearest_lon; // Print Hasil Longitude terbaru
                }

                if (frame.flags & DW_GPS_LAT){
                    std::cout << "easting: " << east; // Print Hasil Koordinat Easting
                }

                if (frame.flags & DW_GPS_LON){
                    std::cout << "northing: " << north; // Print Hasil Koordinat Northing
                }

                if (frame.flags & DW_GPS_LON){
                    std::cout << "zone: " << zone; // Print Hasil Zona Koordinat UTM
                }
				
				std::vector<Station> nearbyStations;
                for (const auto& station : stations) {
                    double dist_sign = distance(nearest_lat, nearest_lon, station.latitudeSign, station.longitudeSign);
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
	
	// Cleanup
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    curl_global_cleanup();


    outfile.close(); // close the file
    std::cout << "Execution times written to file." << std::endl;

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

#include <curl/curl.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <fstream>
using namespace std;


// Positioning Correction (01)
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
// end Positioning Correction (01)




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




int main()
{
    // OPEN FILE
    std::ofstream outfile;
    outfile.open("execution_times_pp.txt"); // open the file
    if (!outfile.is_open()) { // check if the file is open
        std::cout << "Failed to open file." << std::endl;
        return 1;
    }
    // Positioning Correction (02)
    ifstream datafileWay("DataNovatel.txt"); // change filename to the CSV file
    if (!datafileWay.is_open()) { // check if file is open
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
    // end Positioning Correction (02)




    // BACA FILE NOVATEL
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




    for (size_t i = 0; i < latitude.size(); i++) { // use size_t for the loop variable
        auto start = std::chrono::high_resolution_clock::now();
        // Positioning Correction (03)
        double nearest_lat = latitude[i];
        double nearest_lon = longitude[i];
        double nearest_dist = INFINITY;
        for (size_t j = 0; j < latitudeWay.size(); j++) { // use size_t for the loop variable
            double lat = latitudeWay[j];
            double lon = longitudeWay[j];
            double dist = distance(latitude[i], longitude[i], lat, lon);
            if (dist < nearest_dist) {
                nearest_lat = lat;
                nearest_lon = lon;
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
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        outfile << duration << std::endl;
    }


    // Cleanup
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    curl_global_cleanup();


    outfile.close(); // close the file
    std::cout << "Execution times written to file." << std::endl;
    return 0;
}

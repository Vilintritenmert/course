#include <iostream>
#include <ostream>
#include <stdexcept>
#include <fstream>
#include <cmath>

using namespace std;

fstream getStream(const char *fileName) {
    fstream streamFile(fileName);
    if (!streamFile.is_open()) {
        throw invalid_argument("`" + string(fileName) +
                               "` file is not available");
    }

    return streamFile;
}

const float ticks_per_revolution = 1024;
const float wheel_radius_m = 0.3;
const float wheelbase_m = 1.0;

struct WheelImpulse {
    float timestamp = 0;
    float fl_ticks = 0;
    float fr_ticks = 0;
    float bl_ticks = 0;
    float br_ticks = 0;

    WheelImpulse operator-(const WheelImpulse& other) {
        return WheelImpulse{
            other.timestamp-timestamp,
            other.fl_ticks - fl_ticks,
            other.fr_ticks - fr_ticks,
            other.bl_ticks - bl_ticks,
            other.br_ticks - br_ticks,
        };
    }
};

struct Nrk {
    float x = 0;
    float y = 0;
    float theta = 0;

    WheelImpulse lastImpulse;

    Nrk& operator+=(const WheelImpulse& newImpulse) {
        const WheelImpulse impulseDifference = lastImpulse-newImpulse;

        const float d_left = (impulseDifference.fl_ticks + impulseDifference.bl_ticks) / 2;
        const float d_right = (impulseDifference.fr_ticks + impulseDifference.br_ticks) / 2;

        const float distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;

        const float dL = d_left * distance_per_tick;
        const float dR = d_right * distance_per_tick;

        const float distance = (dL + dR) / 2;
        const float dTheta = (dR - dL) / wheelbase_m;

        x += distance * cos(theta + dTheta / 2);
        y += distance * sin(theta + dTheta / 2);

        theta +=dTheta;

        lastImpulse = newImpulse;

        return *this;
    }
};

fstream& operator>>(fstream& is, WheelImpulse& impulse) {
    is >> impulse.timestamp 
       >> impulse.fl_ticks
       >> impulse.fr_ticks
       >> impulse.bl_ticks
       >> impulse.br_ticks;
    
    return is;  
}

ostream& operator<<(ostream& os, const Nrk& nrk) {
    os 
     << nrk.lastImpulse.timestamp << " "
     << nrk.x << " "
     << nrk.y << " "
     << nrk.theta;

     return os;
}


int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            throw invalid_argument("usage: ugv_odometry <input_path>\n");
        }

        fstream inputFile = getStream(argv[1]);
        WheelImpulse newImpulse;
        Nrk nrk;

        inputFile >> nrk.lastImpulse;
        
        cout << nrk << endl;

        while(true) {
            inputFile >> newImpulse;

            if (inputFile.eof()) {
                cout << "Data is uploaded" << endl;
                
                break;
            }

            nrk += newImpulse;

            cout << nrk << endl;

        }
        
        inputFile.close();

        return 0;
    } catch (std::invalid_argument e) {
        std::cerr << "Something went wrong: " << e.what() << endl;
        return 1;
    }

}

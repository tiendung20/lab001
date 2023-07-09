//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef VEINS_INET_CONSTANT_H_
#define VEINS_INET_CONSTANT_H_

#include <stdlib.h>
#include <string.h>
#include <string>
#include <stdio.h>
#include <vector>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "lib/vecmath/vecmath.h"
using veins::TraCIMobility;
using namespace veins;

typedef std::pair<std::string, double> Pair;
enum ModeOfLatencyEmergenyTime {
    MIN, MAX, AVERAGE, RAN
};

class Constant {
public:
    static constexpr const char *FIRST = "First vehicle";
    static constexpr const char *RSU_IDENTIFY = "RSU";
    static constexpr const char *CARRY_ON = "CARRY-ON";
    static constexpr const long WANTED_ID = 52;
    static constexpr const double ZONE_LENGTH = 1.1;
    static constexpr const char *NON_CAR = "";
    static constexpr const double GAMMA = 0.3;
    static constexpr const double DELTA_T = 1;
    static constexpr const double EXPIRED_TIME = 2;
    static constexpr const double MAX_SPEED = 4;
    static constexpr const double LENGTH_CROSSING = 6.4;
    Constant();
    virtual ~Constant();
    static TraCIMobility *activation;
    static long TOTAL_WAITING_TIME;
    static double TOTAL_TRAVELLING_TIME;
    static double TOTAL_APE;
    static double GLOBAL_HARMFULNESS;
    static double GLOBAL_SONNER;
    static double GLOBAL_LATER;
    static int TOTAL_AGV;
    static double EMERGENCY_TIME;
    static int edgeIdCnt;

    static constexpr const bool SHORTEST_PATH = false;
    static constexpr const double LENGTH_OF_B_VERTEX = 0.1; //assume that length of b_vertex (for ACO) is 0.1
    static constexpr const bool STOP_AT_STATION = true;
    static constexpr const int PAUSING_TIME = 10;
    static std::map<std::string, std::string> routeDict;
    static constexpr const double THRESHOLD = -2;
    static constexpr const double DELAY = 10; //10 seconds
    static constexpr const int MAX_REQUESTS = 6;
    static constexpr const ModeOfLatencyEmergenyTime STRATEGY =
            ModeOfLatencyEmergenyTime::MIN;

    // For SFM
    static constexpr const float MEAN = 1.2;
    static constexpr const double UPPER_SPEED_LIMIT = 1.8;
    static constexpr const double LOWER_SPEED_LIMIT = 0.3;
    static constexpr const float PI = 3.14159265359F;

    // Speed - color
    // No disability, without overtaking behavior
    static constexpr const float V1 = 1.24;
    // The blind
    static constexpr const float V6 = 0.52;
    // Walking with crutches
    static constexpr const float V3 = 0.94;
    // No disability, with overtaking behavior
    static constexpr const float V2 = 2.28;
    // Wheelchairs
    static constexpr const float V5 = 0.69;
    // Walking with sticks
    static constexpr const float V4 = 0.81;

};

static void timing(double *wcTime, double *cpuTime) {
    struct timeval tp;
    struct rusage ruse;

    gettimeofday(&tp, NULL);
    *wcTime = (double) (tp.tv_sec + tp.tv_usec / 1000000.0);

    getrusage(RUSAGE_SELF, &ruse);
    *cpuTime = (double) (ruse.ru_utime.tv_sec
            + ruse.ru_utime.tv_usec / 1000000.0);
}

static void timing_(double *wcTime, double *cpuTime) {
    timing(wcTime, cpuTime);
}

static std::string twoDecimalString(double x) {
    std::string y = std::to_string(x);
    int len = y.size();
    y = y.substr(0, len - 4);
    return y;
}

static char* mergeContent(long Id) {
    char *cstr = new char[strlen(Constant::RSU_IDENTIFY) + 1];
    strcpy(cstr, Constant::RSU_IDENTIFY);
    std::string str = std::to_string(Id);
    char *new_str = new char[str.length() + 1];
    strcpy(new_str, str.c_str());
    char *ret = new char[strlen(Constant::RSU_IDENTIFY) + strlen(new_str) + 1];
    strcpy(ret, cstr);
    strcat(ret, new_str);
    return ret;
}

static bool checkInvalidRoute(std::string route) {
    if (route.find("-E362 -E419") != std::string::npos
            || route.find("^-E362 ^-E419") != std::string::npos
            || route.find("-E362 ^-E419") != std::string::npos
            || route.find("^-E362 -E419") != std::string::npos) {
        return true;
    }
    return false;
}

static int getIndexInFlow(std::string idOfAGV, std::string routeId) {
    if (Constant::routeDict[routeId].length() == 0) {
        Constant::routeDict[routeId] = "$" + idOfAGV + "$";
        return 0;
    }
    if (Constant::routeDict[routeId].find("$" + idOfAGV + "$")
            == std::string::npos) {
        std::string s = Constant::routeDict[routeId];
        int n = std::count(s.begin(), s.end(), '$');
        Constant::routeDict[routeId] = Constant::routeDict[routeId] + "$"
                + idOfAGV + "$";
        return (n / 2);
    } else {
        int index = Constant::routeDict[routeId].find("$" + idOfAGV + "$");
        std::string s = Constant::routeDict[routeId].substr(0, index);
        int n = std::count(s.begin(), s.end(), '$');
        if (n >= 2) {
            EV << "OK" << endl;
        }
        return (n / 2);
    }
    return -1;
}

static std::vector<std::string> split(const std::string &str,
        const std::string &delim) {
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do {
        pos = str.find(delim, prev);
        if (pos == std::string::npos)
            pos = str.length();
        std::string token = str.substr(prev, pos - prev);
        if (!token.empty())
            tokens.push_back(token);
        prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());
    return tokens;
}

static std::string removeLoop(const std::string &str) {
    std::vector<std::string> tokens = split(str, " ");
    int i = 0, j = 0;
    int found = 0;
    for (i = 0; i < tokens.size(); i++) {
        for (j = i + 1; j < tokens.size(); j++) {
            if (tokens[i].compare(tokens[j]) == 0
                    || ("^" + tokens[i]).compare(tokens[j]) == 0
                    || tokens[i].compare("^" + tokens[j]) == 0) {
                found = 1;
                break;
            }
        }
        if (found)
            break;
    }
    if (found) {
        tokens.erase(tokens.begin() + i, tokens.begin() + j);
        std::string result = tokens[0];
        for (int i = 1; i < tokens.size(); i++) {
            result = result + " " + tokens[i];
        }
        return result;
    }
    return str;
}

static std::string getLane(std::vector<std::string> list, bool wantLast) {
    if (wantLast) {
        for (int i = list.size() - 1; i >= 0; i--) {
            if (list[i][0] != ':' && list[i].find("_") == std::string::npos) {
                if (list[i][0] == '^')
                    return list[i].substr(1);
                return list[i];
            }
        }
    } else {
        for (int i = 1; i < list.size(); i++) {
            if (list[i][0] != ':' && list[i].find("_") == std::string::npos) {
                if (list[i][0] == '^')
                    return list[i].substr(1);
                return list[i];
            }
        }
    }
    return "";
}

static int locateLast(std::string route, std::string trace) {
    std::vector<std::string> tokens = split(route, " ");
    int last = tokens.size() - 1;
    if (last < 0)
        return -1;
    std::string lastLane = tokens[last];

    if (trace.find("$" + lastLane + "$") == std::string::npos) {
        return -1;        //not found
    } else {
        int location = trace.find("$" + lastLane + "$") + lastLane.length();
        return location;
    }
}

static bool willReachExit(std::string route) {
    if (route.length() < 2)
        return false;
    if (route.length() < 5) {
        if (route.find("-E0") != std::string::npos
                || route.find("E92") != std::string::npos
                || route.find("E298") != std::string::npos) {
            return true;
        }
        return false;
    }
    std::string last = route.substr(route.length() - 5);
    if (last.find("-E226") != std::string::npos
            || last.find("-E0") != std::string::npos
            || last.find("E92") != std::string::npos
            || last.find("E298") != std::string::npos) {
        return true;
    }
    return false;
}

static bool goAround(std::vector<std::string> v) {
    std::string temp1, temp2;
    for (int i = 0; i < v.size(); i++) {
        temp1 = v[i];
        if (temp1[0] == '^')
            temp1 = temp1.substr(1);
        for (int j = i + 1; j < v.size(); j++) {
            temp2 = v[j];
            if (temp2[0] == '^')
                temp2 = temp2.substr(1);
            if (temp1.compare(temp2) == 0) {
                return true;
            }
        }
    }
    return false;
}

#endif /* VEINS_INET_CONSTANT_H_ */

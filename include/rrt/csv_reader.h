#ifndef SRC_CSV_READER_H
#define SRC_CSV_READER_H

#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

namespace Global_Plan_Reader
{

/// A Class to read csv data files
class TRJReader
{
    std::string fileName;
    //std::string delimeter;

public:
    explicit TRJReader(std::string filename ) ://, std::string delm = ",") :
            fileName(filename)
    {}

    ///
    /// Function to fetch data from a CSV File
    std::vector<std::array<double, 2>> getData() {
        //std::string filename = "/home/biagio/Scrivania/ifac2020_race-master/src/rrt_star/trajectories/mtl/optimal.trj";
        //std::string filename = "/home/biagio/Scrivania/ifac2020_race-master/src/global_planner/berlin/central_safe.trj";
        std::ifstream file(fileName);

        if (!file) {
            throw std::runtime_error("Invalid Path for csv file.");
        }

        std::vector<std::array<double, 2>> dataList;

        std::string line;
        size_t pos = 0;
        std::string temp;
        int i;

        for (i = 0; std::getline(file, line); i++)//Read useless line
        {
            std::array<double, 2> trackpoint{};


            //read x
            std::getline(file, line);
            pos = line.find(":") + 1;
            trackpoint[0] = (atof(line.substr(pos, std::string::npos).c_str()));

            //read y
            std::getline(file, line);
            pos = line.find(":") + 1;
            trackpoint[1] = (atof(line.substr(pos, std::string::npos).c_str()));

            //read speed
            std::getline(file, line);
            pos = line.find(":") + 1;
            auto c = (atof(line.substr(pos, std::string::npos).c_str()));

            dataList.emplace_back(trackpoint);
        }
        file.close();


        return dataList;
    }
};

}

#endif

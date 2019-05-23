/* Copyright (c) EcarX - All Rights Reserved
 * Author: Ke Ma <ke.ma@ecarx.com.cn>
 * description: KITTI oxts imu operations
 */
#ifndef _COMBDATA_HPP_
#define _COMBDATA_HPP_

#include <fstream> // std::ifstream
#include <ctime>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "oxtsPose.hpp"
#include "kittiVelCamFusion.hpp"
#include "kittiOXTS.hpp"

class CombData
{
public:
    CombData(std::string kittiDir,
             std::string baseDir,
             std::string oxtsPath,
             std::string veloDir,
             std::string imgPath)
        : kittiDir_(kittiDir),
          baseDir_(baseDir),
          oxtsPath_(oxtsPath),
          veloDir_(veloDir),
          imgPath_(imgPath) {}

    CombData()
    {
        kittiDir_ = "/media/mk11/Dataset/KITTI_all";
        baseDir_ = kittiDir_ + "/Raw/2011_09_26_drive_0009/2011_09_26/2011_09_26_drive_0009_sync";
        oxtsPath_ = baseDir_ + "/oxts/data";
        veloDir_ = baseDir_ + "/velodyne_points/data";
        imgPath_ = baseDir_ + "/image_00/data";
        combinedResultsOutDir_ = "/home/mk11/compileWS/pcprojection/output";
    }

    std::string getTimeExt()
    {
        time_t now = time(0);
        struct tm tstruct;
        char buf[80];
        tstruct = *localtime(&now);

        strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &tstruct);
        std::string mytime(buf);

        return mytime;
    }

    long long stringToDatetime(const std::string &str)
    {
        char *cha = (char *)str.data();
        tm timeinfo;
        int year, month, day, hour, minute, second, ns;
        sscanf(cha, "%d-%d-%d %d:%d:%d.%d", &year, &month, &day, &hour, &minute, &second, &ns);
        timeinfo.tm_year = year - 1900; // year, start from 1900
        timeinfo.tm_mon = month - 1;    // month, range: 0-11
        timeinfo.tm_mday = day;
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = minute;
        timeinfo.tm_sec = second;

        timeinfo.tm_isdst = 0;
        time_t t = mktime(&timeinfo); // convert form tm ot time_t

        long long ts = t * 1e6 + ns / 1000;

        return ts;
    }

    bool loadTimestamps(std::vector<long long> &vTimes, std::string timeFilePath)
    {
        vTimes.clear();
        std::ifstream ifs(timeFilePath);
        std::string str;
        int count = 0;
        while (std::getline(ifs, str))
        {
            if (str.empty())
            {
                continue;
            }
            else
            {
                long long t = stringToDatetime(str);
                vTimes.push_back(t);
            }
            // printf("%lld\n", vTimestamps_.at(count));
            count++;
        }
        ifs.close();

        return true;
    }

    bool loadOxtsTimePoses()
    {
        KittiOXTS *ko = new KittiOXTS();

        if (!ko->exec())
        {
            std::cerr << "\t mk11-error: ko->exec()" << std::endl;
        }

        if (!ko->getTimePoseCopy(vOxtsTimePoses_))
        {
            std::cerr << "\t mk11-error: ko->getTimePoseCopy(vOxtsTimePoses_)" << std::endl;
        }

        // if (!ko->printVeloTimePoses())
        // {
        //     std::cerr << "\t mk11-error: ko->printVeloTimePoses())" << std::endl;
        // }

        return true;
    }

    bool loadVeloTimeBinaryPath()
    {
        std::string veloTimesFilePath = baseDir_ + "/velodyne_points/timestamps.txt";
        std::string veloBinPath = baseDir_ + "/velodyne_points/data";

        loadTimestamps(veloTimes_, veloTimesFilePath);

        veloBinPaths_.clear();
        std::map<int, std::string> vbps = had::File::getFileNames(veloBinPath, ".bin", 10, 6, false);
        for (auto p = vbps.begin(); p != vbps.end(); p++)
        {
            veloBinPaths_.push_back(p->second);
        }

        // for (int i = 0; i < veloBinPaths_.size(); ++i)
        // {
        //     printf("%lld \t %s\n", veloTimes_.at(i), veloBinPaths_.at(i).c_str());
        // }

        return true;
    }

    bool loadImgTimePngPath()
    {
        std::string imgTimesFilePath = baseDir_ + "/image_00/timestamps.txt";
        std::string imgPngDir = baseDir_ + "/image_00/data";

        loadTimestamps(imgTimes_, imgTimesFilePath);

        imgPngPaths_.clear();
        std::map<int, std::string> ipps = had::File::getFileNames(imgPngDir, ".png", 10, 6, false);
        for (auto p = ipps.begin(); p != ipps.end(); p++)
        {
            imgPngPaths_.push_back(p->second);
        }

        // for (int i = 0; i < imgPngPaths_.size(); ++i)
        // {
        //     printf("%lld \t %s\n", imgTimes_.at(i), imgPngPaths_.at(i).c_str());
        // }
        return true;
    }

    std::string getFileName(const std::string &str_in)
    {
        std::string s = str_in;

        char sep = '/';
        size_t i = s.rfind(sep, s.length());
        if (i != std::string::npos)
        {
            // std::cout << i << std::endl;
            s = s.substr(i + 1, s.length() - i);
        }

        sep = '.';
        i = s.find(sep);
        if (i != std::string::npos)
        {
            // std::cout << i << std::endl;
            s = s.substr(0, i);
            return s;
        }

        return ("");
    }

    bool writeTimeDataPath2File()
    {

        std::string outFileName = "sequence_txyz_tvel_timg_";
        std::string outFilePath = combinedResultsOutDir_ + "/" + outFileName + getTimeExt() + ".txt";

        std::ofstream ofs;
        ofs.open(outFilePath);

        auto i_oxts = vOxtsTimePoses_.begin();
        auto i_velTime = veloTimes_.begin();
        auto i_veloBin = veloBinPaths_.begin();
        auto i_imgTime = imgTimes_.begin();
        auto i_imgPath = imgPngPaths_.begin();

        int strLength_oxts = 0;
        int strLength_velo = 0;
        int strLength_imag = 0;
        int idx = 0;
        while (i_oxts != vOxtsTimePoses_.end() || i_velTime != veloTimes_.end() || i_veloBin != veloBinPaths_.end() || i_imgTime != imgTimes_.end() || i_imgPath != imgPngPaths_.end())
        {
            char str_oxts[1000] = "";
            if (i_oxts != vOxtsTimePoses_.end())
            {
                long long t = (*i_oxts).getTime();
                double x = (*i_oxts).getx();
                double y = (*i_oxts).gety();
                double z = (*i_oxts).getz();

                sprintf(str_oxts, "%20lld \t %10.6f \t %10.6f \t %10.6f", t, x, y, z);

                ++i_oxts;
            }
            else
            {
                for (int i = 0; i < strLength_oxts; i++)
                {
                    strcat(str_oxts, " ");
                }
            }

            char str_velo[1000] = "";
            if (i_velTime != veloTimes_.end() && i_veloBin != veloBinPaths_.end() && std::stoi(getFileName(*i_veloBin)) == idx)
            {
                sprintf(str_velo, "%20lld \t %s", *i_velTime, (*i_veloBin).c_str());

                ++i_velTime;
                ++i_veloBin;
            }
            else
            {
                for (int i = 0; i < strLength_velo; i++)
                {
                    strcat(str_velo, " ");
                }
            }

            char str_imag[1000] = "";
            if (i_imgTime != imgTimes_.end() && i_imgPath != imgPngPaths_.end() && std::stoi(getFileName(*i_imgPath)) == idx)
            {
                sprintf(str_imag, "%20lld \t %s", *i_imgTime, (*i_imgPath).c_str());

                ++i_imgTime;
                ++i_imgPath;
            }
            else
            {
                for (int i = 0; i < strLength_imag; i++)
                {
                    strcat(str_imag, " ");
                }
            }

            std::string oline = "";
            oline = oline + (std::string)str_oxts + "\t\t";
            oline = oline + (std::string)str_velo + "\t\t";
            oline = oline + (std::string)str_imag + "\t\n";

            strLength_oxts = ((std::string)str_oxts).length();
            strLength_velo = ((std::string)str_velo).length();
            strLength_imag = ((std::string)str_imag).length();

            ofs << oline.c_str();

            if (++idx > 447)
            {
                break;
            }
        }

        ofs.close();

        return true;
    }

    bool exec()
    {
        std::cerr << "\t \t loading loadOxtsTimePoses ... " << std::endl;
        if (!loadOxtsTimePoses())
        {
            std::cerr << "\t \t loading loadOxtsTimePoses ... "
                      << "ERROR" << std::endl;
        }
        else
        {
            std::cerr << "\t \t loading loadOxtsTimePoses ... complete "
                      << "\n"
                      << std::endl;
        }

        std::cerr << "\t \t loading loadVeloTimeBinaryPath ... " << std::endl;
        if (!loadVeloTimeBinaryPath())
        {
            std::cerr << "\t \t loading loadVeloTimeBinaryPath ... "
                      << "ERROR" << std::endl;
        }
        else
        {
            std::cerr << "\t \t loading loadVeloTimeBinaryPath ... complete "
                      << "\n"
                      << std::endl;
        }

        std::cerr << "\t \t loading loadImgTimePngPath ... " << std::endl;
        if (!loadImgTimePngPath())
        {
            std::cerr << "\t \t loading loadImgTimePngPath ... "
                      << "ERROR" << std::endl;
        }
        else
        {
            std::cerr << "\t \t loading loadImgTimePngPath ... complete "
                      << "\n"
                      << std::endl;
        }

        std::cerr << "\t \t loading writeTimeDataPath2File ... " << std::endl;
        if (!writeTimeDataPath2File())
        {
            std::cerr << "\t \t loading writeTimeDataPath2File ... "
                      << "ERROR" << std::endl;
        }
        else
        {
            std::cerr << "\t \t loading writeTimeDataPath2File ... complete "
                      << "\n"
                      << std::endl;
        }

        return true;
    }

private:
    std::string kittiDir_;
    std::string baseDir_;
    std::string oxtsPath_;
    std::string veloDir_;
    std::string imgPath_;
    std::string combinedResultsOutDir_;

    std::vector<OxtsPose> vOxtsTimePoses_;

    std::vector<long long> veloTimes_;
    std::vector<std::string> veloBinPaths_;

    std::vector<long long> imgTimes_;
    std::vector<std::string> imgPngPaths_;
};

#endif // _COMBDATA_HPP_

// #include "kittiVelCamFusion.hpp"
// #include "kittiOXTS.hpp"
// #include "oxtsPose.hpp"
// #include "combData.hpp"
// #include <iostream>

// int main(int argc, char **argv)
// {

//     CombData *cd = new CombData();

//     if (!cd->exec())
//     {
//         std::cerr << "\t mk11-error: something wrong" << std::endl;
//     }

//     return 0;
// }

/*
 * Copyright (C) 2016 Social Robotics Lab, Yale University & 2014 iCub Facility
 * Authors: Alessandro Roncone & Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <speech.h>

using namespace std;

int main(int argc, char ** argv)
{
    string fullpath = argv[0];
    fullpath = fullpath.substr(0, fullpath.find_last_of("\\/")+1);

    ros::init(argc, argv, "svox_speech");

    Speech s(fullpath);

    ros::spin();
    return 0;
}

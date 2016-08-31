/*
 * Copyright (C) 2016 Social Robotics Lab, Yale University & 2014 iCub Facility
 * Authors: Alessandro Roncone & Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <string>
#include <vector>

#include <picoapi.h>
#include <picoapid.h>
#include <picoos.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

/****************************************************************
 * @brief The Speech class
 */
class Speech
{

public:
    Speech(std::string _fullpath);
    ~Speech();

    // Speech_IDL
    virtual bool setLanguage(const std::string& language);
    virtual std::vector<std::string>  getSupportedLang();
    virtual bool say(const std::string& text);
    virtual bool setSpeed(const int16_t speed);
    virtual bool setPitch(const int16_t pitch);
    virtual int16_t getSpeed();
    virtual int16_t getPitch();
    virtual bool play();
    virtual bool pause();
    virtual bool stop();

private:
    /**
     * @brief renderSpeech
     * @param text
     * @return the rendered wave file path
     */
    const std::string renderSpeech(const std::string& text);
    bool playWav(const std::string& filename);
    void releasePico();
private:
    std::string fullpath;
    std::string language;
    std::string pcmDevice;
    int pitch, speed;
    std::vector<std::string> supportedLangs;

private: // ROS Stuff
    ros::NodeHandle nh;

    ros::Subscriber s_sub;  // Subscriber for the speech output

    std::string speech;             // Text to display

    void speechCb(const std_msgs::String& msg);

private: // picotts
    /* adapation layer global variables */
    void *          picoMemArea;
    pico_System     picoSystem;
    pico_Resource   picoTaResource;
    pico_Resource   picoSgResource;
    pico_Resource   picoUtppResource;
    pico_Engine     picoEngine;
    pico_Char *     picoTaFileName;
    pico_Char *     picoSgFileName;
    pico_Char *     picoUtppFileName;
    pico_Char *     picoTaResourceName;
    pico_Char *     picoSgResourceName;
    pico_Char *     picoUtppResourceName;
    int picoSynthAbort;
};

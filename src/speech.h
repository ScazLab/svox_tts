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

#include <svox_tts/Speech.h>

#define DEFAULT_PITCH     55
#define DEFAULT_SPEED    110
#define DEFAULT_LANG  "en-US"

/****************************************************************
 * @brief The Speech class
 */
class Speech
{

public:
    Speech(std::string _fullpath);
    ~Speech();

    // Speech_IDL
    virtual bool setSpeed(const int16_t speed);
    virtual bool setPitch(const int16_t pitch);
    virtual bool setLanguage(const std::string& language);
    virtual int16_t     getSpeed();
    virtual int16_t     getPitch();
    virtual std::string getLanguage();
    virtual std::vector<std::string>  getSupportedLang();

    virtual bool say(const std::string& text);

    bool resetDefaults();

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

    ros::ServiceServer service;  // Service for the speech output and other options
    ros::Publisher     s_output; // Publisher with the speech output (useful to other nodes,
                                 // such as baxter_collaboration/baxter_display)

    /**
     * Callback for the service that interfaces with this class
     * @param  req the speech request
     * @param  res the speech response (res.success either true or false)
     * @return     true always :)
     */
    bool serviceCb(svox_tts::Speech::Request  &req,
                   svox_tts::Speech::Response &res);

    /**
     * Publishes the text that have been said on a topic
     * @param text The text to publish
     */
    void publishSpeechOutput(const std::string &text);

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __timer_hpp__
#define __timer_hpp__

#include <boost/date_time.hpp>

struct timer{
    bool hasEnded = false;
    boost::posix_time::ptime* start_time = NULL;
    boost::posix_time::time_duration* time_diff = NULL;
    timer(){
        timer::start_time = new boost::posix_time::ptime;
        timer::time_diff = new boost::posix_time::time_duration;
    };
    ~timer(){
        if (!timer::hasEnded){
            delete timer::start_time;
            delete timer::time_diff;  
        }
    };

    void start(){
        timer::start_time[0] = boost::posix_time::microsec_clock::local_time();
    };

    void reset(){
        timer::start_time[0] = boost::posix_time::microsec_clock::local_time();
    };

    double elapsed(){
        double elapsed;
        timer::time_diff[0] = boost::posix_time::microsec_clock::local_time() - *timer::start_time;
        elapsed = timer::time_diff->total_nanoseconds() / 1e9;  
        return elapsed;
    };

    void end(){
        timer::hasEnded = true;
        delete timer::start_time;
        delete timer::time_diff;
    };
};


#endif
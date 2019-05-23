/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: time utilities
 */

#ifndef _TIME_HELPER_HPP_ 
#define _TIME_HELPER_HPP_

#include <iostream>
#include <chrono>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#define GPS_OFFSET_1980_1_6       (315964800L)
#define SECONDS_IN_A_WEEK         (604800)
#define GPS_UTC_OFFSET            (18)

static inline int64_t utc_gps2sec(int gps_week, int gps_secs)
{
	int64_t tim_val ;
	tim_val = gps_secs -GPS_UTC_OFFSET + gps_week * SECONDS_IN_A_WEEK + GPS_OFFSET_1980_1_6 ;
	
	return tim_val ;
}

static inline int64_t utc_date2sec(int yy, int mm, int dd, int hh, int min, int sec, bool utc_date = false)
{
	struct tm tm ;
	time_t tim_val ;
	memset(&tm, 0, sizeof(tm)) ;
	tm.tm_year = yy-1900 ;
	tm.tm_mon = mm-1 ;
	tm.tm_mday = dd ;
	tm.tm_hour = hh ;
	tm.tm_min = min  ;
	tm.tm_sec = sec ;

    if(utc_date)
	    tim_val = mktime(&tm) ;
    else
	    tim_val = mktime(&tm) - timezone ;
	
    //printf("%ld\n", tim_val) ;
	
	return (int64_t)tim_val ;
}


/**
 * Stopwatch helps to measure time intervals
 * for instance to evaluate performance of a piece of code
 */
class Stopwatch {
public:
    enum Type {CHRONO=0, TIMEOFDAY} ;
private:
    Type type ;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_tic;
    int64_t time_tic_tod;
    const std::string description;

public:
    Stopwatch() :
        Stopwatch("") {
    }

public:
    explicit Stopwatch(std::string description_):
        type(CHRONO),
        description(description_) {
        tic();
    }

    explicit Stopwatch(Type t):
        type(t),
        description("") {
        tic();
    }

public:
    /**
     * starts/resets stopwatch from time of call to tic()
     *
     * MATLAB: tic
     */
    void tic() {
        if(type == CHRONO)
            time_tic = std::chrono::high_resolution_clock::now();
        else if(type == TIMEOFDAY)
            time_tic_tod = timeofday_now() ;
    }

    /**
     * return time of day in micro seconds.
     *
     * same as bot_timestamp_now
     */
    static int64_t timeofday_now ()
    {
        struct timeval tv ;
        gettimeofday(&tv, NULL) ;
        return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec ;
    }

    /**
     * return start time in stopwatch
     *
     */
    int64_t get_time_tic_tod() const
    {
        return time_tic_tod ;
    }

public:
    /**
     * returns number of seconds since call of tic(), or Stopwatch instantiation
     *
     * call does not reset the stopwatch, i.e. behaviour as in MATLAB
     *
     * MATLAB: toc
     */
    double toc() const {
        return utoc() * 1e-6;
    }

    /**
     * return number of seconds since start_time
     *
     */
    double toc(int64_t start_time) const
    {
        return (timeofday_now()-start_time) * 1e-6 ;
    }

public:
    /**
     * returns number of micro seconds since call of tic(), or Stopwatch instantiation
     *  1 sec = 1e6 micro seconds
     *
     *  call does not reset the stopwatch
     */
    long utoc() const {
        long us=0;
        if(type == CHRONO)
        {
            auto time_toc = std::chrono::high_resolution_clock::now();
            us = std::chrono::duration_cast<std::chrono::microseconds>(time_toc - time_tic).count();
        }
        else if(type == TIMEOFDAY)
        {
            us = timeofday_now()-time_tic_tod ;
        }

        return us ;
    }

public:
    /**
     * returns number of nano seconds since call of tic(), or Stopwatch instantiation
     *  1 sec = 1e9 nano seconds
     *
     *  call does not reset the stopwatch
     */
    long ntoc() const {
        auto time_toc = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time_toc - time_tic).count();
    }

public:
    /**
     * debugPrint(false) does nothing
     *
     * debugPrint(true) prints out the name of the stopwatch
     * together with the microseconds elapsed.
     *
     * the function exists for convenience to quickly
     * activate and deactivate the feedback for performance timing.
     */
    void debugPrint(bool print = true) const {
        if (print)
            std::cout << description << ": " << (utoc() / 1000.0) << " ms." << std::endl;
    }

};

#endif

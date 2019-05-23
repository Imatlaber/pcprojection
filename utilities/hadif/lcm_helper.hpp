/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: color helper functions
 */

#ifndef _LCM_HELPER_HPP_
#define _LCM_HELPER_HPP_

#include <lcm/lcm-cpp.hpp>
#include <unordered_set>
#include <thread>

template <class MessageHandlerClass, typename MessageType>

class SubscribeMessageType
{
    lcm::LCM *lcm;
    std::unordered_set<std::string> undecodable_channels;
    lcm::Subscription *sub;
    void (MessageHandlerClass::*handlerMethod)(const lcm::ReceiveBuffer* rbuf, const std::string& channel);
    MessageHandlerClass* handler;
public:
    //try to decode data, and update subscriber when undecodable channel received
    bool getMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, MessageType &msg)
    {
        if(msg.decode(rbuf->data, 0, rbuf->data_size)>0)
            return true;
        //Then, form a regex to exclude non-decodable channel, e.g.
        //to unsubscribe from PMD_INFO, VEHICLE_MONITORS and UbloxGPS, the regex would be:
        //^(?!(^$|^PMD_INFO$|^VEHICLE_MONITOR$|^UbloxGPS$)).*$
        //The first symbols ^$ is there so we can have a balanced |
        undecodable_channels.insert(channel);
        std::string regex("(?!(^$");
        for(auto channel : undecodable_channels)
            regex += "|^" + channel + "$";
        regex += ")).*";

        lcm->unsubscribe(sub);

        sub = lcm->subscribe(regex, handlerMethod, handler);

        return false;
    }

    SubscribeMessageType(lcm::LCM *_lcm,
                         void (MessageHandlerClass::*_handlerMethod)
                         (const lcm::ReceiveBuffer* rbuf, const std::string& channel),
                         MessageHandlerClass* _handler
                         ):
        lcm(_lcm), handlerMethod(_handlerMethod), handler(_handler)
    {
        //Start by subscribing to all channels
        sub = lcm->subscribe(".*", handlerMethod, handler);
    }
};

class ThreadedLcm 
{

public:
    // maximum of waiting time between call of shutdown(); and
    constexpr static int TIMEOUT_MS = 100;
	lcm::LCM * lcm_ ;
    std::thread thread;
    // flag that indicates to thread that shutdown has been called
    bool shutdownFlag = false;

public:
	ThreadedLcm(lcm::LCM * lcm) 
	: lcm_(lcm)
	{
	}

	~ThreadedLcm()
	{
		stopThread() ;
	}

    /**
     * shutdown() indicates to the thread that listening for
     * lcm messages is no more required.
     *
     * shutdown() is blocking for at most TIMEOUT_MS.
     *
     * shutdown() should be called right before the main loop exits
     */
    void shutdown() {
        stopThread();
    }

    void startThread() {
        thread = std::thread(&ThreadedLcm::run, this);
        // the thread is not detached, i.e. not thread.detach();
        //  + thread.join() is mandatory
        //  + via shutdown()
    }

private:
    void stopThread() {
        shutdownFlag = true;
        if (thread.joinable()) {
            thread.join();
            printf("ThreadedLcm::thread joined\n");
        }
    }

private:
    /**
     * private function run() implements the logic of the thread
     *
     *  1) listen for incoming lcm messages
     *  2) terminate when shutdownFlag is set
     */
    void run() {
        // nutils::nuLCM::handleTimeout(timeout_millis)
        //   @return >0 if a message was handled,
        //            0 if the function timed out, and
        //           <0 if an error occured.
        while (!shutdownFlag && 0 <= lcm_->handleTimeout(TIMEOUT_MS)) {
            printf("thread lcm: spinonce \n") ;
        }
    }

};
#endif


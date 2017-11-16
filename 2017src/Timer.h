//Timer header file
#ifndef TIMER_H
#define TIMER_H

#include <thread>
#include <chrono>
#include <ctime>

class Timer
{
public:
  
    
    typedef std::chrono::milliseconds Interval;
    typedef std::function<void(void)> Timeout;
    
    int count = 0;

    Timer(const Timeout &timeout);
    Timer(const Timeout &timeout,
          const Interval &interval,
          bool singleShot = true);

    void start(bool multiThread = false);
    void stop();

    bool running() const;

    void setSingleShot(bool singleShot);
    bool isSingleShot() const;

    void setInterval(const Interval &interval);
    const Interval &interval() const;

    void setTimeout(const Timeout &timeout);
    const Timeout &timeout() const;
    
    double getTime();
    
    double getCycleTime();

    unsigned long getTime_helper();
        
    unsigned long getCycleTime_helper();
     
    
    

private:
    std::thread _thread;
    
    unsigned long begTime;

    bool _running = false;
    bool _isSingleShot = true;

    Interval _interval = Interval(0);
    Timeout _timeout = nullptr;

    void _temporize();
    void _sleepThenTimeout();
    
    
};

#endif // TIMER_H
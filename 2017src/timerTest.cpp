#include <iostream>

#include "Timer.h"
#include <unistd.h>

using namespace std;

int main(void)
{
    int sec = 0;
    Timer tHello([&sec]()
    {
        //cout << "Hello!" << endl;
        //cout << sec << endl;
        sec++;
    });

    tHello.setSingleShot(false);
    tHello.setInterval(Timer::Interval(1000));
    tHello.start(true);
    
    while(tHello.getTime() < 10){
        
    }
    //this_thread::sleep_for(chrono::milliseconds(12121));
    //cout << tHello.getTime() << endl;
    cout << tHello.getTime() << endl;
    cout << tHello.getCycleTime() << endl;
    //sleep(5);
    //cout << CLOCKS_PER_SEC << endl;
    //cout << clock() << endl;
    //cout << tHello.getTime() << endl;
    //cout << tHello.getCycleTime() << endl;

    Timer tStop([&]()
    {
        tHello.stop();
    });

    tStop.setSingleShot(true);
    tStop.setInterval(Timer::Interval(3000));
    tStop.start();

    return 0;
}
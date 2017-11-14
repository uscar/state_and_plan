#include <iostream>

#include "Timer.h"

using namespace std;

int main(void)
{
    Timer tHello([]()
    {
        cout << "Hello!" << endl;
    });

    tHello.setSingleShot(false);
    tHello.setInterval(Timer::Interval(1000));
    tHello.start(true);
    
    int limit = std::numeric_limits<int>::max();
    int n = 0;
    while(n < limit-2){
        n++;
    }
    
    cout << CLOCKS_PER_SEC << endl;
    cout << tHello.getTime() << endl;
    cout << tHello.getCycleTime() << endl;

    Timer tStop([&]()
    {
        tHello.stop();
    });

    tStop.setSingleShot(true);
    tStop.setInterval(Timer::Interval(3000));
    tStop.start();

    return 0;
}
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

    Timer tStop([&]()
    {
        tHello.stop();
    });

    tStop.setSingleShot(true);
    tStop.setInterval(Timer::Interval(3000));
    tStop.start();

    return 0;
}
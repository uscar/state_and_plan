CPPFLAGS = g++ -g -Wall -std=c++11

all: clean timerTest
	-@echo "-----BUILT-----"


timerTest: timerTest.cpp Timer.o 
	$(CPPFLAGS) $< Timer.o -o timerTest




Timer.o: Timer.cpp Timer.h
	$(CPPFLAGS) -c $< -o Timer.o




.PHONY: 
	clean

clean:
	rm -rf Timer.o
	rm -rf testTimer
	-@echo "----CLEANED----"

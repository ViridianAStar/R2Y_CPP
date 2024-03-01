#include "vex.h"

class clockD
{
    private:
        // keep track of start time (internal to the object?)
        std::chrono::time_point<std::chrono::high_resolution_clock> globalStart;
    public:
        // final time taken between starts and stops
        std::chrono::microseconds timeTaken;

        void start() {
            // initialize the clock
            auto start = std::chrono::high_resolution_clock::now();
            globalStart = start;
        }

        void stop() {
            // stop the clock
            auto stop = std::chrono::high_resolution_clock::now();
            // cast the clocks time to microseconds
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - globalStart);
            timeTaken = duration;
        }
};
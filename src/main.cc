#include <string>
#include <fstream>
#include <vector>
#include <iostream>

#include "Schedule_state.h"
#include "Schedule.h"

#include "FileReader.h"



int main(int argc, char *argv[])
{
    auto reader = Scheduler::FileReader(std::string("/home/liu/workspace/Scheduler/STG/50/rand0000.stg"));
    Scheduler::Schedule schedule(reader.getNodes(), 4);
    //schedule.solve();
    //auto all_unscheduled = state.get_unscheduled();
}

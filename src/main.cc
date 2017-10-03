#include <string>
#include <fstream>
#include <vector>
#include <iostream>

#include "Schedule_state.h"
#include "Schedule.h"

#include "FileReader.h"



int main(int argc, char *argv[])
{
    auto reader = Scheduler::FileReader(std::string("/home/liu/workspace/Scheduler/STG/50/rand0001.stg"));
    auto schedule_ptr = new Scheduler::Schedule(reader.getNodes(), 4);
    
    schedule_ptr->solve();
    //std::cout<<"OK!"<<std::endl;
    //delete schedule_ptr;
    //auto schedule_ptr2 = new Scheduler::Schedule(reader.getNodes(), 4);
    //schedule_ptr2->solve();
    std::cout<<"OK!"<<std::endl;
    std::this_thread::sleep_for(std::chrono::minutes(100));

    //auto all_unscheduled = state.get_unscheduled();
}

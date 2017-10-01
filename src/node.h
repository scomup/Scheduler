#ifndef NODE_H
#define NODE_H

#include<stdint.h>

namespace Scheduler
{

struct Node
{
    const int16_t iD;
    const int16_t core_num;
    const int16_t time;
    const std::vector<int16_t> sub_nodes;
};

}

#endif
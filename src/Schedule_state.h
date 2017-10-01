#ifndef SCHEDULE_STATE
#define SCHEDULE_STATE

#include <algorithm>
#include <limits>
#include <set>

#include "make_unique.h"
#include "node.h"

namespace Scheduler
{

constexpr int16_t inf = -1;
//constexpr int16_t offset_time = std::numeric_limits<int16_t>::min();
constexpr int16_t offset_time = -10000;

class Schedule_state
{
  public:
    Schedule_state(std::vector<Node> &nodes, int16_t core_num);
    Schedule_state(const Schedule_state &state);
    void update_schedulable_time(std::vector<Node> &nodes);
    std::unique_ptr<std::vector<int16_t>> get_unscheduled();
    void set_finished_node(std::vector<Node> &nodes, int16_t node_id, int16_t schedulabe_time);
    int get_schedulable_time(int id);
    
    std::vector<int16_t> nodes_finish_time_;
    std::vector<int16_t> cores_ocuppied_time_;
};
Schedule_state::Schedule_state(const Schedule_state &state)
    : nodes_finish_time_(state.nodes_finish_time_), cores_ocuppied_time_(state.cores_ocuppied_time_)
{
}

Schedule_state::Schedule_state(std::vector<Node> &nodes, int16_t core_num)
{
    int16_t nodes_num = nodes.size();
    nodes_finish_time_.resize(nodes_num);
    std::fill(nodes_finish_time_.begin(), nodes_finish_time_.end(), inf);
    nodes_finish_time_[0] = 0;
    cores_ocuppied_time_.resize(core_num);
    std::fill(cores_ocuppied_time_.begin(), cores_ocuppied_time_.end(), 0);
}


int Schedule_state::get_schedulable_time(int id)
{

    int16_t res = nodes_finish_time_[id];
    if(res < inf)
        return - res + offset_time;
    else
        return inf;
}

std::unique_ptr<std::vector<int16_t>> Schedule_state::get_unscheduled()
{
    int16_t nodes_num = nodes_finish_time_.size();
    auto p_unscheduled = common::make_unique<std::vector<int16_t>>();

    for (int16_t i = 0; i < nodes_num; i++)
    {
        if (nodes_finish_time_[i] == inf)
        {
            p_unscheduled->push_back(i);
        }
    }
    return p_unscheduled;
}


void Schedule_state::update_schedulable_time(std::vector<Node> &nodes)
{
    std::sort(cores_ocuppied_time_.begin(), cores_ocuppied_time_.end());
    int16_t nodes_num = nodes_finish_time_.size();
    for (int16_t id = 0; id < nodes_num; id++)
    {
        if (nodes_finish_time_[id] >= 0)
            continue;
        int16_t core_num = nodes[id].core_num;
        //int16_t schedulabe_time = std::max(cores_ocuppied_time_[core_num - 1], task_schedulabe_time);
        int16_t schedulabe_time = inf;

        for (auto sub_node_id : nodes[id].sub_nodes)
        {
            schedulabe_time = std::max(nodes_finish_time_[sub_node_id], schedulabe_time);
        }
        if (schedulabe_time != inf)
        {
            schedulabe_time = std::max(cores_ocuppied_time_[core_num - 1], schedulabe_time);
            nodes_finish_time_[id] = offset_time - (schedulabe_time + nodes[id].time);
        }
    }
}

void Schedule_state::set_finished_node(std::vector<Node> &nodes, int16_t node_id, int16_t schedulabe_time)
{
    std::sort(cores_ocuppied_time_.begin(), cores_ocuppied_time_.end());
    int16_t core_num = nodes[node_id].core_num;
    for (int16_t i = 0; i < core_num; i++)
    {
        cores_ocuppied_time_[i] = schedulabe_time + nodes[node_id].time;
    }
    nodes_finish_time_[node_id] = schedulabe_time + nodes[node_id].time;
    update_schedulable_time(nodes);
}
}

#endif

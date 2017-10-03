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
    int16_t get_schedulable_time(int16_t id);
    int16_t get_node_predict_finished_time(std::vector<Node> &nodes, int16_t id);
    void predict_finished_time(std::vector<Node> &nodes);
    int16_t get_finished_node_num();
    
    std::vector<int16_t> nodes_finish_time_;
    std::vector<int16_t> cores_ocuppied_time_;
    int16_t pre_time_;
};
Schedule_state::Schedule_state(const Schedule_state &state)
    : nodes_finish_time_(state.nodes_finish_time_), cores_ocuppied_time_(state.cores_ocuppied_time_),pre_time_(state.pre_time_)
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
    update_schedulable_time(nodes);
    predict_finished_time(nodes);

}


int16_t Schedule_state::get_schedulable_time(int16_t id)
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

int16_t Schedule_state::get_finished_node_num()
{
    int16_t nodes_num = nodes_finish_time_.size();
    int16_t finished_node_num = 0;

    for (int16_t i = 0; i < nodes_num; i++)
    {
        if (nodes_finish_time_[i] > inf)
        {
            finished_node_num++;
        }
    }
    return finished_node_num;
}



void Schedule_state::update_schedulable_time(std::vector<Node> &nodes)
{
    std::sort(cores_ocuppied_time_.begin(), cores_ocuppied_time_.end());
    int16_t nodes_num = nodes_finish_time_.size();
    for (int16_t id = 0; id < nodes_num; id++)
    {
        if (nodes_finish_time_[id] > inf)
            continue;
        int16_t core_num = nodes[id].core_num;
        //int16_t schedulabe_time = std::max(cores_ocuppied_time_[core_num - 1], task_schedulabe_time);
        int16_t schedulabe_time = inf;

        for (auto sub_node_id : nodes[id].sub_nodes)
        {
            if(nodes_finish_time_[sub_node_id] <= inf){
                schedulabe_time = inf;
                break;
            }
            schedulabe_time = std::max(nodes_finish_time_[sub_node_id], schedulabe_time);
        }
        if (schedulabe_time != inf)
        {
            schedulabe_time = std::max(cores_ocuppied_time_[core_num - 1], schedulabe_time);
            nodes_finish_time_[id] = offset_time - schedulabe_time;
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
    predict_finished_time(nodes);
}

void Schedule_state::predict_finished_time(std::vector<Node> &nodes)
{
    pre_time_ = get_node_predict_finished_time(nodes, nodes.size()-1);
}

int16_t Schedule_state::get_node_predict_finished_time(std::vector<Node> &nodes, int16_t id)
{
    int16_t pre_time = nodes_finish_time_[id];
    if (pre_time > inf)
    {
        return pre_time;
    }
    else if(pre_time < inf)
    {
        return get_schedulable_time(id) + nodes[id].time;
    }
    else
    {
        int16_t sub_pre_time = inf;
        for (int16_t sub_id : nodes[id].sub_nodes)
        {
            int16_t tmp = get_node_predict_finished_time(nodes, sub_id);
            sub_pre_time = std::max(sub_pre_time, tmp);
        }
        return sub_pre_time + nodes[id].time;
    }
}
}

#endif

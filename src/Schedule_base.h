#ifndef SCHEDULE_H
#define SCHEDULE_H

#include <algorithm>
#include <limits>
#include <set>
#include <map>
#include <list>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <chrono>
#include <thread>
#include "make_unique.h"
#include "Schedule_state.h"

namespace Scheduler
{
typedef std::vector<std::unique_ptr<Schedule_state>> state_vec;

class Schedule
{

public:
  Schedule(std::vector<Node> Nodes, int16_t core_num);
  void solve();

private:
  void recude(state_vec &sub_states, state_vec &new_sub_states);
  void generate_sub_state(std::unique_ptr<Schedule_state> state_ptr,state_vec& sub_states);
  void generate_next_level_state(state_vec &sub_states, state_vec &next_sub_states_reduce);
  void get_schedulabe_time(int16_t id, std::unique_ptr<Schedule_state> state_ptr_);
  void check_new(std::list<std::unique_ptr<Schedule_state>> &list, std::unique_ptr<Schedule_state> &x);
  int16_t cmapare(std::unique_ptr<Schedule_state> &a, std::unique_ptr<Schedule_state> &b);

  std::vector<Node> Nodes_;
  std::vector<int16_t> cores_ocuppied_time_;
  std::unique_ptr<Schedule_state> init_state_ptr_;
};

Schedule::Schedule(std::vector<Node> Nodes, int16_t core_num)
    : Nodes_(Nodes), init_state_ptr_(common::make_unique<Schedule_state>(Nodes, core_num))
{}

void Schedule::generate_sub_state(std::unique_ptr<Schedule_state> state_ptr,state_vec& sub_states)
{
  
  int16_t nodes_num = Nodes_.size();
  std::vector<std::pair<int16_t,int16_t>> schedulable;
  for (int16_t id = 0; id < nodes_num; id++)
  {
    int16_t schedulabe_time = state_ptr->get_schedulable_time(id);
    if (schedulabe_time == inf)
      continue;
      schedulable.push_back(std::make_pair(id,schedulabe_time));
  }
  std::sort(schedulable.begin(),schedulable.end(),[](const std::pair<int16_t,int16_t>& a, const  std::pair<int16_t,int16_t>& b){return a.second < a.second;});
  int16_t min_schedulabe = schedulable.front().second;
  for (auto &m : schedulable)
  {
    if(min_schedulabe == m.second && Nodes_[m.first].core_num == 4){
      schedulable.clear();
      schedulable.push_back(m);
      break;
    }

  }
  for (auto &m : schedulable)
  {
    auto sub_state_ptr = common::make_unique<Schedule_state>(*state_ptr);
    sub_state_ptr->set_finished_node(Nodes_, m.first, m.second);
    sub_states.emplace_back(std::move(sub_state_ptr));
  }
  state_ptr.reset(nullptr);
}

void Schedule::generate_next_level_state(state_vec &sub_states, state_vec &next_sub_states_reduce)
{
  static int level = 0;

  state_vec next_sub_states;
  auto p0 = std::chrono::system_clock::now();
  for (size_t i = 0; i < sub_states.size(); i++)
  {
    generate_sub_state(std::move(sub_states[i]), next_sub_states);
    //std::move(next_sub_states_part.begin(), next_sub_states_part.end(), std::back_inserter(next_sub_states));
  }
  auto p1 = std::chrono::system_clock::now();
  auto diff1 = p1 - p0;

  recude(next_sub_states, next_sub_states_reduce);
  //next_sub_states_reduce = std::move(next_sub_states);
  auto p2 = std::chrono::system_clock::now();
  auto diff2 = p2 - p1;
  std::cout << level++
            << "   :elapsed time = " << std::chrono::duration_cast<std::chrono::seconds>(diff1).count()
            << " "
            << std::chrono::duration_cast<std::chrono::seconds>(diff2).count() << "   "
            << next_sub_states_reduce.size() << std::endl;
}

void Schedule::solve()
{
  init_state_ptr_->update_schedulable_time(Nodes_);
  state_vec sub_states;
  generate_sub_state(std::move(init_state_ptr_),sub_states);

  for (size_t level = 0; level < Nodes_.size(); level++)
  {
    state_vec next_sub_states;
    generate_next_level_state(sub_states, next_sub_states);
    next_sub_states.swap(sub_states);
    if(level == 49){
      std::cout<<sub_states[0]->nodes_finish_time_[51]<<std::endl;;
    }
  }

}

void Schedule::recude(state_vec &sub_states, state_vec &new_sub_states)
{
  
  std::map<size_t, std::unique_ptr<std::list<std::unique_ptr<Schedule_state>>>> states_map;

  for (size_t i = 0; i < sub_states.size(); i++)
  {
    auto unscheduled = sub_states[i]->get_unscheduled();
    size_t key = boost::hash_range(unscheduled->begin(), unscheduled->end());
    //unscheduled.reset(nullptr);

    auto itr = states_map.find(key);
    if (itr == states_map.end())
    {
      auto v_ptr = common::make_unique<std::list<std::unique_ptr<Schedule_state>>>();
      v_ptr->emplace_back(std::move(sub_states[i]));
      states_map[key] = std::move(v_ptr);
    }
    else
    {
      auto &v = (*states_map[key]);
      check_new(v, sub_states[i]);
    }
  }

  for (auto &m : states_map)
  {
    auto state_list_ptr = std::move(m.second);
    for (auto &state_ptr : *state_list_ptr)
    {
      new_sub_states.emplace_back(std::move(state_ptr));
    }
  }
}

void Schedule::check_new(std::list<std::unique_ptr<Schedule_state>> &list, std::unique_ptr<Schedule_state> &target)
{
  auto i = list.begin();
  while (i != list.end())
  {
    int16_t flag = cmapare((*i), target);
    if (flag == -1)
    {
      target.reset(nullptr);
      return;
    }
    else if (flag == 1)
    {
      (*i).reset(nullptr);
      list.erase(i++);
    }
    else
    {
      i++;
      continue;
    }
  }
  list.emplace_back(std::move(target));
}

int16_t Schedule::cmapare(std::unique_ptr<Schedule_state> &a, std::unique_ptr<Schedule_state> &b)
{
  const size_t node_num = a->nodes_finish_time_.size();
  bool better = false;
  bool worse = false;
  for (size_t i = 0; i < node_num; i++)
  {
    if (a->nodes_finish_time_[i] > inf)
    {
      continue;
    }
    const int16_t schedulable_time_a = -a->nodes_finish_time_[i] + offset_time;
    const int16_t schedulable_time_b = -b->nodes_finish_time_[i] + offset_time;
    bool better_part = schedulable_time_a < schedulable_time_b;
    bool worse_part = schedulable_time_a > schedulable_time_b;
    better |= better_part;
    worse |= worse_part;
  }
  bool select_a = (!worse && better)  || (!worse && !better);
  bool select_b = (worse && !better);
  if (select_a)
    return -1;
  else if (select_b)
    return 1;
  else
    return 0;
}
}

#endif

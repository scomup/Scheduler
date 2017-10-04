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

struct Best_state
{
  bool done;
  int16_t time;
  int16_t bubble;
  std::shared_ptr<Schedule_state> state;
};

class Schedule
{

public:
  Schedule(std::vector<Node> Nodes, int16_t core_num);
  void solve();

private:
  void add_state_by_check_dict(std::shared_ptr<Schedule_state> &new_state);
  void check_new(std::list<std::shared_ptr<Schedule_state>> &list, std::shared_ptr<Schedule_state> &target);
  int16_t cmapare(std::shared_ptr<Schedule_state> &a, std::shared_ptr<Schedule_state> &b);
  void generate_sub_state(std::shared_ptr<Schedule_state> &state_ptr);
  bool generate_from_frontier();
  void add_to_frontier(std::shared_ptr<Schedule_state> &new_state);  

  std::vector<Node> Nodes_;
  std::vector<int16_t> cores_ocuppied_time_;
  std::vector<std::map<size_t, std::list<std::shared_ptr<Schedule_state>>>> dictionary;
  std::map<int16_t,std::list<std::shared_ptr<Schedule_state>>> frontier_;
  Best_state best_state;
};

Schedule::Schedule(std::vector<Node> Nodes, int16_t core_num)
    : Nodes_(Nodes)
{
  auto init_state_ptr = std::make_shared<Schedule_state>(Nodes, core_num);
  dictionary.resize(Nodes.size());
  add_state_by_check_dict(init_state_ptr);
  best_state.done = false;
}

void Schedule::solve()
{
  bool finished = false;
  while (!finished)
    finished = generate_from_frontier();
}

void Schedule::add_to_frontier(std::shared_ptr<Schedule_state> &new_state)
{
  auto itrr = frontier_.find(new_state->pre_time_);
  if (itrr == frontier_.end()){
    std::list<std::shared_ptr<Schedule_state>> state_list;
    state_list.emplace_back(new_state);
    frontier_[new_state->pre_time_] = state_list;
  }
  else{
    frontier_[new_state->pre_time_].emplace_back(new_state);  
  }
}

void Schedule::add_state_by_check_dict(std::shared_ptr<Schedule_state> &new_state)
{
  
  int16_t finished_node_num = new_state->get_finished_node_num();
  auto unscheduled = new_state->get_unscheduled();
  size_t key = boost::hash_range(unscheduled->begin(), unscheduled->end());
  if (new_state->nodes_finish_time_[51] != inf)
  {
    int16_t best_time = new_state->pre_time_;
    if (!best_state.done || (best_state.done && best_state.time > best_time))
    {
      best_state.done = true;
      best_state.time = best_time;
      best_state.bubble = new_state->pre_time_;
      best_state.state = new_state;
    }
    return;
  }

  auto &level_dict = dictionary[finished_node_num];
  auto itr = level_dict.find(key);
  if (itr == level_dict.end())
  {
    std::list<std::shared_ptr<Schedule_state>> state_list;
    level_dict[key].emplace_back(new_state);
    add_to_frontier(new_state);
  }
  else
  {
    auto &v = (level_dict[key]);
    check_new(v, new_state);
  }
  
}

bool Schedule::generate_from_frontier()
{
  auto p0 = std::chrono::system_clock::now();
  static size_t level = 0;
  level++;

  auto& current_frontier = frontier_.begin()->second;
  int16_t best_time = frontier_.begin()->first;
  size_t active_frontier_size = current_frontier.size();

  //std::cout 
  //          << "   :elapsed time = " << std::chrono::duration_cast<std::chrono::milliseconds>(diff1).count()
  //          << std::endl;



  int16_t current_node_num = current_frontier.front()->get_finished_node_num();
  //frontier_.sort([](const std::shared_ptr<Schedule_state> &left, const std::shared_ptr<Schedule_state> &right){return left.get()->x < right.get()->x;});
  if (best_state.done)
  {
    //std::cout << min_frontier_time<<best_state.bubble<<std::endl;
    std::cout 
              << ": OK:"
              << best_state.time
              << std::endl;
    return true;
  }

  auto it = current_frontier.begin();
  for (size_t i = 0; i < active_frontier_size; i++)
  {
    generate_sub_state((*it));
    it++;
  }
  auto b = current_frontier.begin();
  auto e = current_frontier.begin();
  std::advance(e, active_frontier_size);
  current_frontier.erase(b, e);
  if(current_frontier.size() == 0){
    frontier_.erase(best_time);
  }
  size_t frontier_size = 0;
  for (auto &m : frontier_ )
  {
    frontier_size += m.second.size();
    it++;
  }

  auto p2= std::chrono::system_clock::now();
  auto diff2 = p2 - p0;

  std::cout << level
            << ": froniter:"
            << frontier_size
            << "   active froniter:"
            << active_frontier_size
            << "   best:"
            << best_time
            << "   fnum:"
            << current_node_num
            << "   :elapsed time = " << std::chrono::duration_cast<std::chrono::seconds>(diff2).count()
            << std::endl;

          
  return false;
  //frontier_.sort([]( std::shared_ptr<Schedule_state> const& left, std::shared_ptr<Schedule_state> const& right){
  //    return (*left).pre_time_ < (*right).pre_time_;});
}

void Schedule::generate_sub_state(std::shared_ptr<Schedule_state> &state_ptr)
{

  int16_t nodes_num = Nodes_.size();
  std::list<std::pair<int16_t,int16_t>> schedulable;
  int16_t min_finished_time = 10000;
  for (int16_t id = 0; id < nodes_num; id++)
  {
    int16_t schedulabe_time = state_ptr->get_schedulable_time(id);
    if (schedulabe_time == inf){
      continue;
    }
    int16_t tmp = schedulabe_time + Nodes_[id].time;
    min_finished_time = std::min(min_finished_time, tmp);
    schedulable.push_back(std::make_pair(id,schedulabe_time));
  }
  schedulable.sort([](const std::pair<int16_t,int16_t>& a, const  std::pair<int16_t,int16_t>& b){return a.second < a.second;});
  int16_t min_schedulabe_time = schedulable.front().second;
  for (auto it = schedulable.begin(); it != schedulable.end(); it++)
  {
    if(min_schedulabe_time == (*it).second && Nodes_[(*it).first].core_num == 4){
      schedulable.clear();
      schedulable.push_back((*it));
      break;
    }
    if(min_schedulabe_time >= min_finished_time){
      schedulable.erase(it++);
    }
    
  }
  for (auto &m : schedulable)
  {
    auto sub_state_ptr = std::make_shared<Schedule_state>(*state_ptr);
    sub_state_ptr->set_finished_node(Nodes_, m.first, m.second);
    add_state_by_check_dict(sub_state_ptr);
  }
}

void Schedule::check_new(std::list<std::shared_ptr<Schedule_state>> &list, std::shared_ptr<Schedule_state> &target)
{
  auto i = list.begin();
  while (i != list.end())
  {
    int16_t flag = cmapare((*i), target);
    if (flag == -1)
    {
      return;
    }
    else if (flag == 1)
    {
      list.erase(i++);
    }
    else
    {
      i++;
      continue;
    }
  }
  list.emplace_back(target);
  add_to_frontier(target);

}

int16_t Schedule::cmapare(std::shared_ptr<Schedule_state> &a, std::shared_ptr<Schedule_state> &b)
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

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
  state_vec generate_sub_state(std::unique_ptr<Schedule_state> state_ptr_);
  void get_schedulabe_time(int16_t id, std::unique_ptr<Schedule_state> state_ptr_);
  void check_new(std::list<std::unique_ptr<Schedule_state>>& list,std::unique_ptr<Schedule_state>& x);
  int16_t cmapare(std::unique_ptr<Schedule_state>& a,std::unique_ptr<Schedule_state>& b);
    
  std::vector<Node> Nodes_;
  std::vector<int16_t> cores_ocuppied_time_;
  std::unique_ptr<Schedule_state> init_state_ptr_;
};

Schedule::Schedule(std::vector<Node> Nodes, int16_t core_num)
    : Nodes_(Nodes), init_state_ptr_(common::make_unique<Schedule_state>(Nodes, core_num))
{
  std::cout<<sizeof(*init_state_ptr_)<<std::endl;
  state_vec a;
  for(size_t i=0;i<= 50000000;i++){
    a.emplace_back(common::make_unique<Schedule_state>(*init_state_ptr_));
    if(i%1000000 == 0)
    std::cout<<i<<std::endl;
  }

  std::cout<<"Try to release!"<<std::endl;
for(auto &x :a){
  auto *raw = x.release();
  delete raw;
}
std::cout<<"OK!"<<std::endl;
int ddd= a[0]->cores_ocuppied_time_[0];
//std::cout<<ddd<<std::endl;
state_vec n;
a.swap(n);
std::this_thread::sleep_for(std::chrono::minutes(3));

  
}

state_vec Schedule::generate_sub_state(std::unique_ptr<Schedule_state> state_ptr)
{
  state_vec all_sub;
  int16_t nodes_num = Nodes_.size();
  for (int16_t id = 0; id < nodes_num; id++)
  {
    int16_t schedulabe_time = state_ptr->get_schedulable_time(id);
    if (schedulabe_time == inf)
      continue;
    auto sub_state_ptr = common::make_unique<Schedule_state>(*state_ptr);
    sub_state_ptr->set_finished_node(Nodes_, id, schedulabe_time);
    all_sub.emplace_back(std::move(sub_state_ptr));
  }
  state_ptr.reset(nullptr);
  return all_sub;
}

void Schedule::solve()
{
  init_state_ptr_->update_schedulable_time(Nodes_);
  auto sub_states = generate_sub_state(std::move(init_state_ptr_));

  for (size_t level = 0; level < Nodes_.size(); level++)
  {
    auto p0 = std::chrono::system_clock::now();
    
    state_vec next_sub_states;

    std::cout << "generate ..."<<std::endl;

    for (size_t i = 0; i < sub_states.size(); i++)
    {
      auto next_sub_states_part = generate_sub_state(std::move(sub_states[i]));
      std::move(next_sub_states_part.begin(), next_sub_states_part.end(), std::back_inserter(next_sub_states));
      //std::cout
      //<< next_sub_states.size() << std::endl;
    }
    auto p1 = std::chrono::system_clock::now();
    auto diff1 = p1 - p0;
    std::cout << "recude ..."<<std::endl;
    
    state_vec next_sub_states_reduce;
    std::cout << next_sub_states.size() << std::endl;
    recude(next_sub_states, next_sub_states_reduce);
    std::cout << next_sub_states_reduce.size() << std::endl;
    //next_sub_states_reduce = std::move(next_sub_states);
    auto p2 = std::chrono::system_clock::now();
    auto diff2 = p2 - p1;

    sub_states.clear();
    sub_states = std::move(next_sub_states_reduce);
    next_sub_states_reduce.clear();

    std::cout << level
              << "   :elapsed time = " << std::chrono::duration_cast<std::chrono::seconds>(diff1).count()
              << " "
              <<  std::chrono::duration_cast<std::chrono::seconds>(diff2).count()<<"   "
              << sub_states.size() << std::endl;
    
  }
}

void Schedule::recude(state_vec &sub_states, state_vec &new_sub_states)
{
  std::map<size_t, std::unique_ptr<std::list<std::unique_ptr<Schedule_state>>> > states_map;

  sub_states[0]->get_schedulable_time(0);
  for (size_t i = 0; i < sub_states.size(); i++)
  {
    auto unscheduled = sub_states[i]->get_unscheduled();
    size_t key = boost::hash_range(unscheduled->begin(), unscheduled->end());
    unscheduled.reset(nullptr);

    auto itr = states_map.find(key);
    if (itr == states_map.end())
    {
      auto v_ptr = common::make_unique<std::list<std::unique_ptr<Schedule_state>>>();
      //common::make_unique<std::list<std::unique_ptr<Schedule_state> > > data;
      v_ptr->emplace_back(std::move(sub_states[i]));
      states_map[key] = std::move(v_ptr);
    }
    else
    {
      auto& v = (*states_map[key]);
      check_new(v,sub_states[i]);

      //states_map[key]->emplace_back(std::move(sub_states[i]));
    }
  }

  for (auto &m : states_map)
  {
    auto state_list_ptr = std::move(m.second);
    for (auto &state_ptr : *state_list_ptr){
      //state->get_schedulable_time(0);
      new_sub_states.emplace_back(std::move(state_ptr));
      //break;
    }
    //for (auto &state_ptr : *state_list_ptr){
    //  state_ptr.reset(nullptr);
    //}

    //std::cout << it << std::endl;
    //new_sub_states.emplace_back(std::move(it));
  }
  //new_sub_states[0]->get_schedulable_time(0);
  //sub_states.clear();
  //states_map.clear();
  //state_vec a;
  //sub_states.swap(a);
  
}

void Schedule::check_new(std::list<std::unique_ptr<Schedule_state>>& list,std::unique_ptr<Schedule_state>& target)
{
  auto i = list.begin();
  std::vector<int16_t> a;
  while (i != list.end())
  {
    int16_t flag = cmapare((*i),target);
    if(flag == -1)
    {
      //target.reset(nullptr);
      target->nodes_finish_time_.swap(a);
      target->cores_ocuppied_time_.swap(a);
      target.reset(nullptr);
      return;
    }
    else if(flag == 1)
    {
      (*i)->nodes_finish_time_.swap(a);
      (*i)->cores_ocuppied_time_.swap(a);
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

  /*
  for (auto &m : list)
  {
    int16_t flag = cmapare(m,target);
    if(flag == -1)
    {
      return;
    }
    else if(flag == 1)
    {
      list.remove(m); 
    }
    else
    {
      continue;
    }
  }
  list.emplace_back(std::move(target));
  */
}

int16_t Schedule::cmapare(std::unique_ptr<Schedule_state>& a,std::unique_ptr<Schedule_state>& b)
{
  //std::cout<<a->nodes_finish_time_[0]<<b->nodes_finish_time_[0];
  const size_t node_num = a->nodes_finish_time_.size();
  bool better = false;
  bool worse  = false;
  for(size_t i = 0; i < node_num; i++)
  {
    if(a->nodes_finish_time_[i] > inf){
      continue;
    }
    const int16_t schedulable_time_a = -a->nodes_finish_time_[i] + offset_time;
    const int16_t schedulable_time_b = -b->nodes_finish_time_[i] + offset_time;
    bool better_part = schedulable_time_a > schedulable_time_b;
    bool worse_part  = schedulable_time_a < schedulable_time_b;
    better |= better_part;
    worse  |= worse_part;
  }
  bool select_a = !worse && better;
  bool select_b = (worse && !better) || (!worse && !better);
  if(select_a)
    return -1;
  else if(select_b)
    return 1;
  else
    return 0;

}

}

#endif

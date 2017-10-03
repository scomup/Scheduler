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
  void generate_from_frontier();  

private:
  void add_state_by_check_dict(std::shared_ptr<Schedule_state>& new_state);
  void check_new(std::list<std::shared_ptr<Schedule_state>> &list, std::shared_ptr<Schedule_state> &target);  
  int16_t cmapare(std::shared_ptr<Schedule_state> &a, std::shared_ptr<Schedule_state> &b);  
  void generate_sub_state(std::shared_ptr<Schedule_state>& state_ptr);
  //void generate_from_frontier();  

  std::vector<Node> Nodes_;
  std::vector<int16_t> cores_ocuppied_time_;
  std::vector<std::map<size_t,std::list<std::shared_ptr<Schedule_state>>>> dictionary;
  std::list<std::shared_ptr<Schedule_state> > frontier_;
  
};

Schedule::Schedule(std::vector<Node> Nodes, int16_t core_num)
    : Nodes_(Nodes)
{
    auto init_state_ptr = std::make_shared<Schedule_state>(Nodes, core_num);
    dictionary.resize(Nodes.size());
    add_state_by_check_dict(init_state_ptr);
    
}

void Schedule::add_state_by_check_dict(std::shared_ptr<Schedule_state>& new_state)
{
    int16_t finished_node_num = new_state->get_finished_node_num();
    auto unscheduled = new_state->get_unscheduled();
    size_t key = boost::hash_range(unscheduled->begin(), unscheduled->end());
    if(key==0){
        for(auto &a: (*unscheduled))
        std::cout<<a<<std::endl;
    }

    auto& level_dict = dictionary[finished_node_num];
    auto itr = level_dict.find(key);
    if (itr == level_dict.end())
    {
        std::list<std::shared_ptr<Schedule_state>> state_list;
        frontier_.emplace_back(new_state);
        state_list.emplace_back(new_state);
        level_dict[key] = std::move(state_list);
    }
    else
    {
        auto &v = (level_dict[key]);
        check_new(v, new_state);
    }
}

bool myt_lt (const std::shared_ptr<Schedule_state>& left,
    const std::shared_ptr<Schedule_state>& right)
 {
    return (left.get()->pre_time_ < right.get()->pre_time_);
 }

void Schedule::generate_from_frontier()
{
    static size_t level = 0;
    level ++;

    size_t active_frontier_size = 0;
    auto it = frontier_.begin();
    frontier_.sort(myt_lt);
    int16_t finished_node_num = frontier_.front()->get_finished_node_num();
    //frontier_.sort([](const std::shared_ptr<Schedule_state> &left, const std::shared_ptr<Schedule_state> &right){return left.get()->pre_time_ < right.get()->pre_time_;});
    int16_t min_frontier_time = frontier_.front()->pre_time_;
    size_t frontier_size = frontier_.size();

    it = frontier_.begin();
    for(size_t i=0;i<frontier_size;i++){
        if(min_frontier_time != (*it)->pre_time_){
            break;
        }
        generate_sub_state((*it));
        //frontier_.erase(it++);
        
        it++;
        active_frontier_size++;
    }   
    auto b = frontier_.begin();
    auto e = frontier_.begin();
    std::advance(e,active_frontier_size);
    frontier_.erase(b, e);

    std::cout << level
              << ": froniter:"
              << frontier_size
              << "   active froniter:"
              << active_frontier_size 
              << "   best:"
              << min_frontier_time 
              << "   fnum:"
              << finished_node_num
              << std::endl;
  
    //frontier_.sort([]( std::shared_ptr<Schedule_state> const& left, std::shared_ptr<Schedule_state> const& right){
    //    return (*left).pre_time_ < (*right).pre_time_;});
    


}

void Schedule::generate_sub_state(std::shared_ptr<Schedule_state>& state_ptr)
{
  int16_t nodes_num = Nodes_.size();
  for (int16_t id = 0; id < nodes_num; id++)
  {
    int16_t schedulabe_time = state_ptr->get_schedulable_time(id);
    if (schedulabe_time == inf)
      continue;
    auto sub_state_ptr = std::make_shared<Schedule_state>(*state_ptr);
    if(id == 51)
    std::cout<<"Here!"<<schedulabe_time<< std::endl;   
    sub_state_ptr->set_finished_node(Nodes_, id, schedulabe_time);
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
  frontier_.emplace_back(target);
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
    bool better_part = schedulable_time_a > schedulable_time_b;
    bool worse_part = schedulable_time_a < schedulable_time_b;
    better |= better_part;
    worse |= worse_part;
  }
  bool select_a = !worse && better;
  bool select_b = (worse && !better) || (!worse && !better);
  if (select_a)
    return -1;
  else if (select_b)
    return 1;
  else
    return 0;
}



}

#endif

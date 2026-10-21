// Bounded RAM journal: navigation never waits for a browser or writes flash.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
namespace raton {
class EventJournal {
 public:
  static constexpr int kCount=32, kLength=144;
  void push(const char* text,uint32_t ms) {
    Entry& e=entries_[next_];e.id=++sequence_;e.ms=ms;
    snprintf(e.text,sizeof(e.text),"%s",text);
    next_=(next_+1)%kCount;
    if(count_<kCount)++count_;else ++overwritten_;
  }
  bool next(uint32_t after,char* out,size_t size,uint32_t& id) const {
    for(int i=0;i<count_;++i){
      const Entry& e=entries_[(next_-count_+i+kCount)%kCount];
      if(e.id<=after)continue;
      id=e.id;snprintf(out,size,"HISTORY id=%lu ms=%lu %s",(unsigned long)e.id,(unsigned long)e.ms,e.text);
      return true;
    }
    return false;
  }
  uint32_t overwritten() const{return overwritten_;}
 private:
  struct Entry {uint32_t id=0,ms=0;char text[kLength]={};};
  Entry entries_[kCount];int next_=0,count_=0;uint32_t sequence_=0,overwritten_=0;
};
}  // namespace raton

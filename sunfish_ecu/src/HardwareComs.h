#ifndef HARDWARE_COMS_H
#define HARDWARE_COMS_H

#include <string>
#include <sys/epoll.h>

class ComsPacket;

class HardwareComs
{
public:
  HardwareComs();
  ~HardwareComs();

  void setDevName(const std::string & name);
  void connect();
  void disconnect();
  bool isConnected();

  bool run(int wait_time, const ComsPacket * tx, ComsPacket * rx);

private:
  std::string devName_;
  int efd_, fd_;
  struct epoll_event event;
  struct epoll_event events[1];

};

#endif

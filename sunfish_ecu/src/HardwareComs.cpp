
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
//#include <sys/epoll.h>
#include <errno.h>

#include "ros/ros.h"
#include "HardwareComs.h"
#include "ComsPacket.h"

HardwareComs::HardwareComs():
  fd_(-1)
{
  efd_ = epoll_create1 (0);
  ROS_ASSERT(efd_ != -1);
}

HardwareComs::~HardwareComs()
{
  disconnect();
  close(efd_);
}

void HardwareComs::setDevName(const std::string & name)
{
  devName_ = name;
}

void HardwareComs::connect()
{
  if (( devName_.size() != 0   ) &&
      ( isConnected() == false )) {

    fd_ = open(devName_.c_str(), O_RDWR);
    if ( fd_ >= 0 ) {
      event.data.fd = fd_;
      event.events = EPOLLIN | EPOLLET;
      int s = epoll_ctl (efd_, EPOLL_CTL_ADD, fd_, &event);
      ROS_ASSERT(s != -1);
    }
  }
}

void HardwareComs::disconnect()
{
  if ( isConnected() ) {
    close(fd_);
    fd_ = -1;
  }
}

bool HardwareComs::isConnected()
{
  return ( fd_ < 0 )? false: true;
}

bool HardwareComs::run(int wait_time, const ComsPacket * tx, ComsPacket * rx)
{
  ROS_ASSERT(rx != NULL);
  bool rv = false;

  if ( isConnected() == false ) {
    connect();
    return false;
  }

  int n = epoll_wait (efd_, events, 1, wait_time);
  for ( int i = 0; i < n; i++)
  {
    if ((  events[i].events & EPOLLERR ) ||
        (  events[i].events & EPOLLHUP ) ||
        (!(events[i].events & EPOLLIN  )))
    {
      // An error has occured on this fd, or the socket is not
      //   ready for reading (why were we notified then?)
      ROS_INFO("epoll error");
      disconnect();
      return false;
    }
    else
    {
      // We have data on the fd waiting to be read. Read and
      // display it. We must read whatever data is available
      // completely, as we are running in edge-triggered mode
      // and won't get a notification again for the same
      // data.
      uint8_t buffer[ComsPacket::DATA_SIZE];
      ssize_t count = read (events[i].data.fd, buffer, ComsPacket::DATA_SIZE);
      if (count == -1) {
        ROS_INFO("Read Error");
        disconnect();
        break;

      } else {
        rx->setData(buffer);
        rv = true;
      }
      if ( tx != NULL ) {
        int s = write (events[i].data.fd, tx->getData(), ComsPacket::DATA_SIZE);
        ROS_ASSERT(s != -1);
      }
    }
  }
  return rv;
}


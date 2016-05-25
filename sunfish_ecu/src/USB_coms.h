#ifndef USB_COMS_H
#define USB_COMS_H

#include <stdint.h>

#include <string>
#include <map>

class USB_Msg
{
public:

private:
  uint8_t buffer[512];
};

class USB_PktInt
{
public:
  bool msgWaiting(void);
  USB_Msg collectMsg(void);
  void  sendMsg(const USB_Msg & msg);

private:
  bool txReady;
  USB_Msg waitingMsg_;
  USB_Msg receivedMsg_;

};


class USB_Coms
{
public:
  USB_Coms(const std::string & devName);
  virtual ~USB_Coms();

  void connect();
  void disconnect();
  bool isConnected();

  void runCom(int milliseconds);

  bool registerCallback(uint16_t type, USB_PktInt * callback);

private:
  std::string devName_;
  std::map<uint16_t, USB_PktInt *> callbackList_;


};

#endif


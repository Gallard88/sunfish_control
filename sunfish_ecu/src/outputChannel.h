#ifndef OUTPUTCHANNEL_H
#define OUTPUTCHANNEL_H

class OutputChannel
{
public:
  OutputChannel();

  typedef enum {
    vecFwd,
    vecStrafe,
    vecDive,
    vecYaw,
    vecPitch,
    vecRoll,
    vecSize
  } vec_t;

  void setChannel(int channel);
  int  getChannel(void);


  void  setMap(int vect, float value);
  float getMap(int vect);

  float run(const float * power);
  void  setDuty(float duty);
  float getDuty(void);
  bool isDutyUpdated(void);

private:
  int channel_;
  float newDuty_, oldDuty_;
  float map_[vecSize];

};

#endif

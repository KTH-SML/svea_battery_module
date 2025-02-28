#ifndef _ROS_svea_msgs_energy_sensor_readings_h
#define _ROS_svea_msgs_energy_sensor_readings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace svea_msgs
{

  class energy_sensor_readings : public ros::Msg
  {
    public:
      typedef const char* _sensor_id_type;
      _sensor_id_type sensor_id;
      typedef float _current_type;
      _current_type current;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _power_type;
      _power_type power;

    energy_sensor_readings():
      sensor_id(""),
      current(0),
      voltage(0),
      power(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_sensor_id = strlen(this->sensor_id);
      varToArr(outbuffer + offset, length_sensor_id);
      offset += 4;
      memcpy(outbuffer + offset, this->sensor_id, length_sensor_id);
      offset += length_sensor_id;
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_power;
      u_power.real = this->power;
      *(outbuffer + offset + 0) = (u_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_power.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_power.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_power.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->power);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_sensor_id;
      arrToVar(length_sensor_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sensor_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sensor_id-1]=0;
      this->sensor_id = (char *)(inbuffer + offset-1);
      offset += length_sensor_id;
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_power;
      u_power.base = 0;
      u_power.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_power.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_power.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_power.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->power = u_power.real;
      offset += sizeof(this->power);
     return offset;
    }

    virtual const char * getType() override { return "svea_msgs/energy_sensor_readings"; };
    virtual const char * getMD5() override { return "da1657ac1a8b4dc0db5e71ce416725bc"; };

  };

}
#endif

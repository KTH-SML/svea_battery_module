#ifndef _ROS_svea_msgs_energy_sensors_h
#define _ROS_svea_msgs_energy_sensors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "svea_msgs/energy_sensor_readings.h"

namespace svea_msgs
{

  class energy_sensors : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t sensors_length;
      typedef svea_msgs::energy_sensor_readings _sensors_type;
      _sensors_type st_sensors;
      _sensors_type * sensors;

    energy_sensors():
      header(),
      sensors_length(0), st_sensors(), sensors(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->sensors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensors_length);
      for( uint32_t i = 0; i < sensors_length; i++){
      offset += this->sensors[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t sensors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sensors_length);
      if(sensors_lengthT > sensors_length)
        this->sensors = (svea_msgs::energy_sensor_readings*)realloc(this->sensors, sensors_lengthT * sizeof(svea_msgs::energy_sensor_readings));
      sensors_length = sensors_lengthT;
      for( uint32_t i = 0; i < sensors_length; i++){
      offset += this->st_sensors.deserialize(inbuffer + offset);
        memcpy( &(this->sensors[i]), &(this->st_sensors), sizeof(svea_msgs::energy_sensor_readings));
      }
     return offset;
    }

    virtual const char * getType() override { return "svea_msgs/energy_sensors"; };
    virtual const char * getMD5() override { return "e4c2832700fa54c75f466d292d9c065d"; };

  };

}
#endif

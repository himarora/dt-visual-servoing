// Generated by gencpp from file duckietown_msgs/SetCustomLEDPattern.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_SETCUSTOMLEDPATTERN_H
#define DUCKIETOWN_MSGS_MESSAGE_SETCUSTOMLEDPATTERN_H

#include <ros/service_traits.h>


#include <duckietown_msgs/SetCustomLEDPatternRequest.h>
#include <duckietown_msgs/SetCustomLEDPatternResponse.h>


namespace duckietown_msgs
{

struct SetCustomLEDPattern
{

typedef SetCustomLEDPatternRequest Request;
typedef SetCustomLEDPatternResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetCustomLEDPattern
} // namespace duckietown_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::duckietown_msgs::SetCustomLEDPattern > {
  static const char* value()
  {
    return "df549e3242a29ae8542797d8cadb0afe";
  }

  static const char* value(const ::duckietown_msgs::SetCustomLEDPattern&) { return value(); }
};

template<>
struct DataType< ::duckietown_msgs::SetCustomLEDPattern > {
  static const char* value()
  {
    return "duckietown_msgs/SetCustomLEDPattern";
  }

  static const char* value(const ::duckietown_msgs::SetCustomLEDPattern&) { return value(); }
};


// service_traits::MD5Sum< ::duckietown_msgs::SetCustomLEDPatternRequest> should match
// service_traits::MD5Sum< ::duckietown_msgs::SetCustomLEDPattern >
template<>
struct MD5Sum< ::duckietown_msgs::SetCustomLEDPatternRequest>
{
  static const char* value()
  {
    return MD5Sum< ::duckietown_msgs::SetCustomLEDPattern >::value();
  }
  static const char* value(const ::duckietown_msgs::SetCustomLEDPatternRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::duckietown_msgs::SetCustomLEDPatternRequest> should match
// service_traits::DataType< ::duckietown_msgs::SetCustomLEDPattern >
template<>
struct DataType< ::duckietown_msgs::SetCustomLEDPatternRequest>
{
  static const char* value()
  {
    return DataType< ::duckietown_msgs::SetCustomLEDPattern >::value();
  }
  static const char* value(const ::duckietown_msgs::SetCustomLEDPatternRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::duckietown_msgs::SetCustomLEDPatternResponse> should match
// service_traits::MD5Sum< ::duckietown_msgs::SetCustomLEDPattern >
template<>
struct MD5Sum< ::duckietown_msgs::SetCustomLEDPatternResponse>
{
  static const char* value()
  {
    return MD5Sum< ::duckietown_msgs::SetCustomLEDPattern >::value();
  }
  static const char* value(const ::duckietown_msgs::SetCustomLEDPatternResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::duckietown_msgs::SetCustomLEDPatternResponse> should match
// service_traits::DataType< ::duckietown_msgs::SetCustomLEDPattern >
template<>
struct DataType< ::duckietown_msgs::SetCustomLEDPatternResponse>
{
  static const char* value()
  {
    return DataType< ::duckietown_msgs::SetCustomLEDPattern >::value();
  }
  static const char* value(const ::duckietown_msgs::SetCustomLEDPatternResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_SETCUSTOMLEDPATTERN_H

// Generated by gencpp from file duckietown_msgs/GetVariableResponse.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_GETVARIABLERESPONSE_H
#define DUCKIETOWN_MSGS_MESSAGE_GETVARIABLERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>

namespace duckietown_msgs
{
template <class ContainerAllocator>
struct GetVariableResponse_
{
  typedef GetVariableResponse_<ContainerAllocator> Type;

  GetVariableResponse_()
    : value_json()  {
    }
  GetVariableResponse_(const ContainerAllocator& _alloc)
    : value_json(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::String_<ContainerAllocator>  _value_json_type;
  _value_json_type value_json;





  typedef boost::shared_ptr< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetVariableResponse_

typedef ::duckietown_msgs::GetVariableResponse_<std::allocator<void> > GetVariableResponse;

typedef boost::shared_ptr< ::duckietown_msgs::GetVariableResponse > GetVariableResponsePtr;
typedef boost::shared_ptr< ::duckietown_msgs::GetVariableResponse const> GetVariableResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator1> & lhs, const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator2> & rhs)
{
  return lhs.value_json == rhs.value_json;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator1> & lhs, const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8570e70d8c775be7006dff91bf8174b8";
  }

  static const char* value(const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8570e70d8c775be7ULL;
  static const uint64_t static_value2 = 0x006dff91bf8174b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/GetVariableResponse";
  }

  static const char* value(const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/String value_json\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
;
  }

  static const char* value(const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value_json);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetVariableResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::GetVariableResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::GetVariableResponse_<ContainerAllocator>& v)
  {
    s << indent << "value_json: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.value_json);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_GETVARIABLERESPONSE_H

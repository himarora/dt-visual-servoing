// Generated by gencpp from file duckietown_msgs/ObstacleImageDetection.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_OBSTACLEIMAGEDETECTION_H
#define DUCKIETOWN_MSGS_MESSAGE_OBSTACLEIMAGEDETECTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <duckietown_msgs/Rect.h>
#include <duckietown_msgs/ObstacleType.h>

namespace duckietown_msgs
{
template <class ContainerAllocator>
struct ObstacleImageDetection_
{
  typedef ObstacleImageDetection_<ContainerAllocator> Type;

  ObstacleImageDetection_()
    : bounding_box()
    , type()  {
    }
  ObstacleImageDetection_(const ContainerAllocator& _alloc)
    : bounding_box(_alloc)
    , type(_alloc)  {
  (void)_alloc;
    }



   typedef  ::duckietown_msgs::Rect_<ContainerAllocator>  _bounding_box_type;
  _bounding_box_type bounding_box;

   typedef  ::duckietown_msgs::ObstacleType_<ContainerAllocator>  _type_type;
  _type_type type;





  typedef boost::shared_ptr< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> const> ConstPtr;

}; // struct ObstacleImageDetection_

typedef ::duckietown_msgs::ObstacleImageDetection_<std::allocator<void> > ObstacleImageDetection;

typedef boost::shared_ptr< ::duckietown_msgs::ObstacleImageDetection > ObstacleImageDetectionPtr;
typedef boost::shared_ptr< ::duckietown_msgs::ObstacleImageDetection const> ObstacleImageDetectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator1> & lhs, const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator2> & rhs)
{
  return lhs.bounding_box == rhs.bounding_box &&
    lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator1> & lhs, const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e532bfbd15e6046dab5e4261999811a9";
  }

  static const char* value(const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe532bfbd15e6046dULL;
  static const uint64_t static_value2 = 0xab5e4261999811a9ULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/ObstacleImageDetection";
  }

  static const char* value(const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/Rect bounding_box\n"
"duckietown_msgs/ObstacleType type\n"
"================================================================================\n"
"MSG: duckietown_msgs/Rect\n"
"# all in pixel coordinate\n"
"# (x, y, w, h) defines a rectangle\n"
"int32 x\n"
"int32 y\n"
"int32 w\n"
"int32 h\n"
"\n"
"================================================================================\n"
"MSG: duckietown_msgs/ObstacleType\n"
"uint8 DUCKIE=0\n"
"uint8 CONE=1\n"
"uint8 type\n"
;
  }

  static const char* value(const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.bounding_box);
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObstacleImageDetection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator>& v)
  {
    s << indent << "bounding_box: ";
    s << std::endl;
    Printer< ::duckietown_msgs::Rect_<ContainerAllocator> >::stream(s, indent + "  ", v.bounding_box);
    s << indent << "type: ";
    s << std::endl;
    Printer< ::duckietown_msgs::ObstacleType_<ContainerAllocator> >::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_OBSTACLEIMAGEDETECTION_H

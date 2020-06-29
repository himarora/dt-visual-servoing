// Generated by gencpp from file duckietown_msgs/IntersectionPoseImg.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_INTERSECTIONPOSEIMG_H
#define DUCKIETOWN_MSGS_MESSAGE_INTERSECTIONPOSEIMG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/CompressedImage.h>

namespace duckietown_msgs
{
template <class ContainerAllocator>
struct IntersectionPoseImg_
{
  typedef IntersectionPoseImg_<ContainerAllocator> Type;

  IntersectionPoseImg_()
    : header()
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , type(0)
    , likelihood(0.0)
    , img()  {
    }
  IntersectionPoseImg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , type(0)
    , likelihood(0.0)
    , img(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _theta_type;
  _theta_type theta;

   typedef uint8_t _type_type;
  _type_type type;

   typedef float _likelihood_type;
  _likelihood_type likelihood;

   typedef  ::sensor_msgs::CompressedImage_<ContainerAllocator>  _img_type;
  _img_type img;





  typedef boost::shared_ptr< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> const> ConstPtr;

}; // struct IntersectionPoseImg_

typedef ::duckietown_msgs::IntersectionPoseImg_<std::allocator<void> > IntersectionPoseImg;

typedef boost::shared_ptr< ::duckietown_msgs::IntersectionPoseImg > IntersectionPoseImgPtr;
typedef boost::shared_ptr< ::duckietown_msgs::IntersectionPoseImg const> IntersectionPoseImgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator1> & lhs, const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.theta == rhs.theta &&
    lhs.type == rhs.type &&
    lhs.likelihood == rhs.likelihood &&
    lhs.img == rhs.img;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator1> & lhs, const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9be288b28225c76ee9ce1052704524b8";
  }

  static const char* value(const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9be288b28225c76eULL;
  static const uint64_t static_value2 = 0xe9ce1052704524b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/IntersectionPoseImg";
  }

  static const char* value(const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float32 x\n"
"float32 y\n"
"float32 theta\n"
"uint8 type\n"
"float32 likelihood\n"
"sensor_msgs/CompressedImage img\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/CompressedImage\n"
"# This message contains a compressed image\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"\n"
"string format        # Specifies the format of the data\n"
"                     #   Acceptable values:\n"
"                     #     jpeg, png\n"
"uint8[] data         # Compressed image buffer\n"
;
  }

  static const char* value(const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
      stream.next(m.type);
      stream.next(m.likelihood);
      stream.next(m.img);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IntersectionPoseImg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::IntersectionPoseImg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "likelihood: ";
    Printer<float>::stream(s, indent + "  ", v.likelihood);
    s << indent << "img: ";
    s << std::endl;
    Printer< ::sensor_msgs::CompressedImage_<ContainerAllocator> >::stream(s, indent + "  ", v.img);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_INTERSECTIONPOSEIMG_H

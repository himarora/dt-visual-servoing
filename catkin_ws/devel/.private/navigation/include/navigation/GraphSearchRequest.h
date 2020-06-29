// Generated by gencpp from file navigation/GraphSearchRequest.msg
// DO NOT EDIT!


#ifndef NAVIGATION_MESSAGE_GRAPHSEARCHREQUEST_H
#define NAVIGATION_MESSAGE_GRAPHSEARCHREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace navigation
{
template <class ContainerAllocator>
struct GraphSearchRequest_
{
  typedef GraphSearchRequest_<ContainerAllocator> Type;

  GraphSearchRequest_()
    : source_node()
    , target_node()  {
    }
  GraphSearchRequest_(const ContainerAllocator& _alloc)
    : source_node(_alloc)
    , target_node(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _source_node_type;
  _source_node_type source_node;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _target_node_type;
  _target_node_type target_node;





  typedef boost::shared_ptr< ::navigation::GraphSearchRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation::GraphSearchRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GraphSearchRequest_

typedef ::navigation::GraphSearchRequest_<std::allocator<void> > GraphSearchRequest;

typedef boost::shared_ptr< ::navigation::GraphSearchRequest > GraphSearchRequestPtr;
typedef boost::shared_ptr< ::navigation::GraphSearchRequest const> GraphSearchRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navigation::GraphSearchRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navigation::GraphSearchRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::navigation::GraphSearchRequest_<ContainerAllocator1> & lhs, const ::navigation::GraphSearchRequest_<ContainerAllocator2> & rhs)
{
  return lhs.source_node == rhs.source_node &&
    lhs.target_node == rhs.target_node;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::navigation::GraphSearchRequest_<ContainerAllocator1> & lhs, const ::navigation::GraphSearchRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace navigation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::navigation::GraphSearchRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation::GraphSearchRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation::GraphSearchRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation::GraphSearchRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation::GraphSearchRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation::GraphSearchRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navigation::GraphSearchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f05fda47731d8da1f80e3a499a42edf9";
  }

  static const char* value(const ::navigation::GraphSearchRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf05fda47731d8da1ULL;
  static const uint64_t static_value2 = 0xf80e3a499a42edf9ULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation::GraphSearchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation/GraphSearchRequest";
  }

  static const char* value(const ::navigation::GraphSearchRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navigation::GraphSearchRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string source_node\n"
"string target_node\n"
;
  }

  static const char* value(const ::navigation::GraphSearchRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navigation::GraphSearchRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.source_node);
      stream.next(m.target_node);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GraphSearchRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation::GraphSearchRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navigation::GraphSearchRequest_<ContainerAllocator>& v)
  {
    s << indent << "source_node: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.source_node);
    s << indent << "target_node: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.target_node);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVIGATION_MESSAGE_GRAPHSEARCHREQUEST_H

/* Auto-generated by genmsg_cpp for file /home/bruno/ros_workspace/src/projeto/msg/QuadStatus.msg */
#ifndef PROJETO_MESSAGE_QUADSTATUS_H
#define PROJETO_MESSAGE_QUADSTATUS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"

namespace projeto
{
template <class ContainerAllocator>
struct QuadStatus_ {
  typedef QuadStatus_<ContainerAllocator> Type;

  QuadStatus_()
  : header()
  , child_frame_id()
  , position()
  , vel()
  , theta()
  {
  }

  QuadStatus_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , child_frame_id(_alloc)
  , position(_alloc)
  , vel(_alloc)
  , theta(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _child_frame_id_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  child_frame_id;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _position_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  position;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _vel_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  vel;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _theta_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  theta;


  typedef boost::shared_ptr< ::projeto::QuadStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::projeto::QuadStatus_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct QuadStatus
typedef  ::projeto::QuadStatus_<std::allocator<void> > QuadStatus;

typedef boost::shared_ptr< ::projeto::QuadStatus> QuadStatusPtr;
typedef boost::shared_ptr< ::projeto::QuadStatus const> QuadStatusConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::projeto::QuadStatus_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::projeto::QuadStatus_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace projeto

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::projeto::QuadStatus_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::projeto::QuadStatus_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::projeto::QuadStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a44ffee83cb7c8bc90166eaeb034f993";
  }

  static const char* value(const  ::projeto::QuadStatus_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa44ffee83cb7c8bcULL;
  static const uint64_t static_value2 = 0x90166eaeb034f993ULL;
};

template<class ContainerAllocator>
struct DataType< ::projeto::QuadStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "projeto/QuadStatus";
  }

  static const char* value(const  ::projeto::QuadStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::projeto::QuadStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
string child_frame_id\n\
geometry_msgs/Vector3 position\n\
geometry_msgs/Vector3 vel\n\
geometry_msgs/Vector3 theta\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::projeto::QuadStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::projeto::QuadStatus_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::projeto::QuadStatus_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::projeto::QuadStatus_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.child_frame_id);
    stream.next(m.position);
    stream.next(m.vel);
    stream.next(m.theta);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct QuadStatus_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::projeto::QuadStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::projeto::QuadStatus_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "child_frame_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.child_frame_id);
    s << indent << "position: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "vel: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.vel);
    s << indent << "theta: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.theta);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PROJETO_MESSAGE_QUADSTATUS_H


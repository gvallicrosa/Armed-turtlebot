/* Auto-generated by genmsg_cpp for file /home/user/workspaces/ros/Armed-turtlebot/mc/msg/belief_msg.msg */
#ifndef MC_MESSAGE_BELIEF_MSG_H
#define MC_MESSAGE_BELIEF_MSG_H
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


namespace mc
{
template <class ContainerAllocator>
struct belief_msg_ {
  typedef belief_msg_<ContainerAllocator> Type;

  belief_msg_()
  : belief()
  , value(0)
  {
  }

  belief_msg_(const ContainerAllocator& _alloc)
  : belief(_alloc)
  , value(0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _belief_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  belief;

  typedef int8_t _value_type;
  int8_t value;


  typedef boost::shared_ptr< ::mc::belief_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mc::belief_msg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct belief_msg
typedef  ::mc::belief_msg_<std::allocator<void> > belief_msg;

typedef boost::shared_ptr< ::mc::belief_msg> belief_msgPtr;
typedef boost::shared_ptr< ::mc::belief_msg const> belief_msgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::mc::belief_msg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::mc::belief_msg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace mc

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::mc::belief_msg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::mc::belief_msg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::mc::belief_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a874dd055c20e6e3ee861af67dd756ee";
  }

  static const char* value(const  ::mc::belief_msg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa874dd055c20e6e3ULL;
  static const uint64_t static_value2 = 0xee861af67dd756eeULL;
};

template<class ContainerAllocator>
struct DataType< ::mc::belief_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mc/belief_msg";
  }

  static const char* value(const  ::mc::belief_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::mc::belief_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "#The mission control node listens for 'belief' messages.\n\
#Name of belief:\n\
#E.g. 'crashed', 'targetLocated' or 'atTarget'.\n\
string belief\n\
#Belief value/state:\n\
# 0: false\n\
# 1: true\n\
int8 value\n\
\n\
";
  }

  static const char* value(const  ::mc::belief_msg_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::mc::belief_msg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.belief);
    stream.next(m.value);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct belief_msg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mc::belief_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::mc::belief_msg_<ContainerAllocator> & v) 
  {
    s << indent << "belief: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.belief);
    s << indent << "value: ";
    Printer<int8_t>::stream(s, indent + "  ", v.value);
  }
};


} // namespace message_operations
} // namespace ros

#endif // MC_MESSAGE_BELIEF_MSG_H


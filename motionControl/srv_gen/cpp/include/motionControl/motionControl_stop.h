/* Auto-generated by genmsg_cpp for file /home/user/workspaces/ros/Armed-turtlebot/motionControl/srv/motionControl_stop.srv */
#ifndef MOTIONCONTROL_SERVICE_MOTIONCONTROL_STOP_H
#define MOTIONCONTROL_SERVICE_MOTIONCONTROL_STOP_H
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

#include "ros/service_traits.h"




namespace motionControl
{
template <class ContainerAllocator>
struct motionControl_stopRequest_ {
  typedef motionControl_stopRequest_<ContainerAllocator> Type;

  motionControl_stopRequest_()
  {
  }

  motionControl_stopRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::motionControl::motionControl_stopRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motionControl::motionControl_stopRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct motionControl_stopRequest
typedef  ::motionControl::motionControl_stopRequest_<std::allocator<void> > motionControl_stopRequest;

typedef boost::shared_ptr< ::motionControl::motionControl_stopRequest> motionControl_stopRequestPtr;
typedef boost::shared_ptr< ::motionControl::motionControl_stopRequest const> motionControl_stopRequestConstPtr;


template <class ContainerAllocator>
struct motionControl_stopResponse_ {
  typedef motionControl_stopResponse_<ContainerAllocator> Type;

  motionControl_stopResponse_()
  {
  }

  motionControl_stopResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::motionControl::motionControl_stopResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motionControl::motionControl_stopResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct motionControl_stopResponse
typedef  ::motionControl::motionControl_stopResponse_<std::allocator<void> > motionControl_stopResponse;

typedef boost::shared_ptr< ::motionControl::motionControl_stopResponse> motionControl_stopResponsePtr;
typedef boost::shared_ptr< ::motionControl::motionControl_stopResponse const> motionControl_stopResponseConstPtr;

struct motionControl_stop
{

typedef motionControl_stopRequest Request;
typedef motionControl_stopResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct motionControl_stop
} // namespace motionControl

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::motionControl::motionControl_stopRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::motionControl::motionControl_stopRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::motionControl::motionControl_stopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::motionControl::motionControl_stopRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::motionControl::motionControl_stopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "motionControl/motionControl_stopRequest";
  }

  static const char* value(const  ::motionControl::motionControl_stopRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::motionControl::motionControl_stopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::motionControl::motionControl_stopRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::motionControl::motionControl_stopRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::motionControl::motionControl_stopResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::motionControl::motionControl_stopResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::motionControl::motionControl_stopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::motionControl::motionControl_stopResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::motionControl::motionControl_stopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "motionControl/motionControl_stopResponse";
  }

  static const char* value(const  ::motionControl::motionControl_stopResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::motionControl::motionControl_stopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::motionControl::motionControl_stopResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::motionControl::motionControl_stopResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::motionControl::motionControl_stopRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct motionControl_stopRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::motionControl::motionControl_stopResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct motionControl_stopResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<motionControl::motionControl_stop> {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const motionControl::motionControl_stop&) { return value(); } 
};

template<>
struct DataType<motionControl::motionControl_stop> {
  static const char* value() 
  {
    return "motionControl/motionControl_stop";
  }

  static const char* value(const motionControl::motionControl_stop&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<motionControl::motionControl_stopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const motionControl::motionControl_stopRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<motionControl::motionControl_stopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "motionControl/motionControl_stop";
  }

  static const char* value(const motionControl::motionControl_stopRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<motionControl::motionControl_stopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const motionControl::motionControl_stopResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<motionControl::motionControl_stopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "motionControl/motionControl_stop";
  }

  static const char* value(const motionControl::motionControl_stopResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MOTIONCONTROL_SERVICE_MOTIONCONTROL_STOP_H


/* Auto-generated by genmsg_cpp for file /home/user/workspaces/ros/Armed-turtlebot/mc/srv/motionControl_timedMove.srv */
#ifndef MC_SERVICE_MOTIONCONTROL_TIMEDMOVE_H
#define MC_SERVICE_MOTIONCONTROL_TIMEDMOVE_H
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




namespace mc
{
template <class ContainerAllocator>
struct motionControl_timedMoveRequest_ {
  typedef motionControl_timedMoveRequest_<ContainerAllocator> Type;

  motionControl_timedMoveRequest_()
  : linear(0.0)
  , angular(0.0)
  , duration(0.0)
  {
  }

  motionControl_timedMoveRequest_(const ContainerAllocator& _alloc)
  : linear(0.0)
  , angular(0.0)
  , duration(0.0)
  {
  }

  typedef double _linear_type;
  double linear;

  typedef double _angular_type;
  double angular;

  typedef double _duration_type;
  double duration;


  typedef boost::shared_ptr< ::mc::motionControl_timedMoveRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mc::motionControl_timedMoveRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct motionControl_timedMoveRequest
typedef  ::mc::motionControl_timedMoveRequest_<std::allocator<void> > motionControl_timedMoveRequest;

typedef boost::shared_ptr< ::mc::motionControl_timedMoveRequest> motionControl_timedMoveRequestPtr;
typedef boost::shared_ptr< ::mc::motionControl_timedMoveRequest const> motionControl_timedMoveRequestConstPtr;


template <class ContainerAllocator>
struct motionControl_timedMoveResponse_ {
  typedef motionControl_timedMoveResponse_<ContainerAllocator> Type;

  motionControl_timedMoveResponse_()
  {
  }

  motionControl_timedMoveResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::mc::motionControl_timedMoveResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mc::motionControl_timedMoveResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct motionControl_timedMoveResponse
typedef  ::mc::motionControl_timedMoveResponse_<std::allocator<void> > motionControl_timedMoveResponse;

typedef boost::shared_ptr< ::mc::motionControl_timedMoveResponse> motionControl_timedMoveResponsePtr;
typedef boost::shared_ptr< ::mc::motionControl_timedMoveResponse const> motionControl_timedMoveResponseConstPtr;

struct motionControl_timedMove
{

typedef motionControl_timedMoveRequest Request;
typedef motionControl_timedMoveResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct motionControl_timedMove
} // namespace mc

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::mc::motionControl_timedMoveRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::mc::motionControl_timedMoveRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::mc::motionControl_timedMoveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a5b60effd8a8dc7915aa4668eca10e0f";
  }

  static const char* value(const  ::mc::motionControl_timedMoveRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa5b60effd8a8dc79ULL;
  static const uint64_t static_value2 = 0x15aa4668eca10e0fULL;
};

template<class ContainerAllocator>
struct DataType< ::mc::motionControl_timedMoveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mc/motionControl_timedMoveRequest";
  }

  static const char* value(const  ::mc::motionControl_timedMoveRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::mc::motionControl_timedMoveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 linear\n\
float64 angular\n\
float64 duration\n\
\n\
";
  }

  static const char* value(const  ::mc::motionControl_timedMoveRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::mc::motionControl_timedMoveRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::mc::motionControl_timedMoveResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::mc::motionControl_timedMoveResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::mc::motionControl_timedMoveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::mc::motionControl_timedMoveResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::mc::motionControl_timedMoveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mc/motionControl_timedMoveResponse";
  }

  static const char* value(const  ::mc::motionControl_timedMoveResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::mc::motionControl_timedMoveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::mc::motionControl_timedMoveResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::mc::motionControl_timedMoveResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::mc::motionControl_timedMoveRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.linear);
    stream.next(m.angular);
    stream.next(m.duration);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct motionControl_timedMoveRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::mc::motionControl_timedMoveResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct motionControl_timedMoveResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<mc::motionControl_timedMove> {
  static const char* value() 
  {
    return "a5b60effd8a8dc7915aa4668eca10e0f";
  }

  static const char* value(const mc::motionControl_timedMove&) { return value(); } 
};

template<>
struct DataType<mc::motionControl_timedMove> {
  static const char* value() 
  {
    return "mc/motionControl_timedMove";
  }

  static const char* value(const mc::motionControl_timedMove&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<mc::motionControl_timedMoveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a5b60effd8a8dc7915aa4668eca10e0f";
  }

  static const char* value(const mc::motionControl_timedMoveRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<mc::motionControl_timedMoveRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mc/motionControl_timedMove";
  }

  static const char* value(const mc::motionControl_timedMoveRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<mc::motionControl_timedMoveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a5b60effd8a8dc7915aa4668eca10e0f";
  }

  static const char* value(const mc::motionControl_timedMoveResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<mc::motionControl_timedMoveResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mc/motionControl_timedMove";
  }

  static const char* value(const mc::motionControl_timedMoveResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MC_SERVICE_MOTIONCONTROL_TIMEDMOVE_H

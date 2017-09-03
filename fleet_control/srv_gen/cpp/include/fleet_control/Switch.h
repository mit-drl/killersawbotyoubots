/* Auto-generated by genmsg_cpp for file /home/drl-mocap/ros_stacks/drl-youbot/fleet_control/srv/Switch.srv */
#ifndef FLEET_CONTROL_SERVICE_SWITCH_H
#define FLEET_CONTROL_SERVICE_SWITCH_H
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




namespace fleet_control
{
template <class ContainerAllocator>
struct SwitchRequest_ {
  typedef SwitchRequest_<ContainerAllocator> Type;

  SwitchRequest_()
  : skin(false)
  {
  }

  SwitchRequest_(const ContainerAllocator& _alloc)
  : skin(false)
  {
  }

  typedef uint8_t _skin_type;
  uint8_t skin;


  typedef boost::shared_ptr< ::fleet_control::SwitchRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fleet_control::SwitchRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SwitchRequest
typedef  ::fleet_control::SwitchRequest_<std::allocator<void> > SwitchRequest;

typedef boost::shared_ptr< ::fleet_control::SwitchRequest> SwitchRequestPtr;
typedef boost::shared_ptr< ::fleet_control::SwitchRequest const> SwitchRequestConstPtr;



template <class ContainerAllocator>
struct SwitchResponse_ {
  typedef SwitchResponse_<ContainerAllocator> Type;

  SwitchResponse_()
  : success(false)
  {
  }

  SwitchResponse_(const ContainerAllocator& _alloc)
  : success(false)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;


  typedef boost::shared_ptr< ::fleet_control::SwitchResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fleet_control::SwitchResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SwitchResponse
typedef  ::fleet_control::SwitchResponse_<std::allocator<void> > SwitchResponse;

typedef boost::shared_ptr< ::fleet_control::SwitchResponse> SwitchResponsePtr;
typedef boost::shared_ptr< ::fleet_control::SwitchResponse const> SwitchResponseConstPtr;


struct Switch
{

typedef SwitchRequest Request;
typedef SwitchResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Switch
} // namespace fleet_control

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fleet_control::SwitchRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fleet_control::SwitchRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fleet_control::SwitchRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c55896e2c19af4923aafe3f0c657bec0";
  }

  static const char* value(const  ::fleet_control::SwitchRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc55896e2c19af492ULL;
  static const uint64_t static_value2 = 0x3aafe3f0c657bec0ULL;
};

template<class ContainerAllocator>
struct DataType< ::fleet_control::SwitchRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/SwitchRequest";
  }

  static const char* value(const  ::fleet_control::SwitchRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fleet_control::SwitchRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool skin\n\
\n\
";
  }

  static const char* value(const  ::fleet_control::SwitchRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::fleet_control::SwitchRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fleet_control::SwitchResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fleet_control::SwitchResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fleet_control::SwitchResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const  ::fleet_control::SwitchResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::fleet_control::SwitchResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/SwitchResponse";
  }

  static const char* value(const  ::fleet_control::SwitchResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fleet_control::SwitchResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
\n\
\n\
";
  }

  static const char* value(const  ::fleet_control::SwitchResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::fleet_control::SwitchResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fleet_control::SwitchRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.skin);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SwitchRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fleet_control::SwitchResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SwitchResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<fleet_control::Switch> {
  static const char* value() 
  {
    return "36a4aec6eac5e7cc284bd46232f7ae74";
  }

  static const char* value(const fleet_control::Switch&) { return value(); } 
};

template<>
struct DataType<fleet_control::Switch> {
  static const char* value() 
  {
    return "fleet_control/Switch";
  }

  static const char* value(const fleet_control::Switch&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<fleet_control::SwitchRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "36a4aec6eac5e7cc284bd46232f7ae74";
  }

  static const char* value(const fleet_control::SwitchRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<fleet_control::SwitchRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/Switch";
  }

  static const char* value(const fleet_control::SwitchRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<fleet_control::SwitchResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "36a4aec6eac5e7cc284bd46232f7ae74";
  }

  static const char* value(const fleet_control::SwitchResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<fleet_control::SwitchResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/Switch";
  }

  static const char* value(const fleet_control::SwitchResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // FLEET_CONTROL_SERVICE_SWITCH_H

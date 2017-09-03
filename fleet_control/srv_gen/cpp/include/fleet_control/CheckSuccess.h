/* Auto-generated by genmsg_cpp for file /home/drl-mocap/ros_stacks/drl-youbot/fleet_control/srv/CheckSuccess.srv */
#ifndef FLEET_CONTROL_SERVICE_CHECKSUCCESS_H
#define FLEET_CONTROL_SERVICE_CHECKSUCCESS_H
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
struct CheckSuccessRequest_ {
  typedef CheckSuccessRequest_<ContainerAllocator> Type;

  CheckSuccessRequest_()
  {
  }

  CheckSuccessRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::fleet_control::CheckSuccessRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fleet_control::CheckSuccessRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct CheckSuccessRequest
typedef  ::fleet_control::CheckSuccessRequest_<std::allocator<void> > CheckSuccessRequest;

typedef boost::shared_ptr< ::fleet_control::CheckSuccessRequest> CheckSuccessRequestPtr;
typedef boost::shared_ptr< ::fleet_control::CheckSuccessRequest const> CheckSuccessRequestConstPtr;



template <class ContainerAllocator>
struct CheckSuccessResponse_ {
  typedef CheckSuccessResponse_<ContainerAllocator> Type;

  CheckSuccessResponse_()
  : success(false)
  {
  }

  CheckSuccessResponse_(const ContainerAllocator& _alloc)
  : success(false)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;


  typedef boost::shared_ptr< ::fleet_control::CheckSuccessResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fleet_control::CheckSuccessResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct CheckSuccessResponse
typedef  ::fleet_control::CheckSuccessResponse_<std::allocator<void> > CheckSuccessResponse;

typedef boost::shared_ptr< ::fleet_control::CheckSuccessResponse> CheckSuccessResponsePtr;
typedef boost::shared_ptr< ::fleet_control::CheckSuccessResponse const> CheckSuccessResponseConstPtr;


struct CheckSuccess
{

typedef CheckSuccessRequest Request;
typedef CheckSuccessResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CheckSuccess
} // namespace fleet_control

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fleet_control::CheckSuccessRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fleet_control::CheckSuccessRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fleet_control::CheckSuccessRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::fleet_control::CheckSuccessRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::fleet_control::CheckSuccessRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/CheckSuccessRequest";
  }

  static const char* value(const  ::fleet_control::CheckSuccessRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fleet_control::CheckSuccessRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::fleet_control::CheckSuccessRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::fleet_control::CheckSuccessRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fleet_control::CheckSuccessResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fleet_control::CheckSuccessResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fleet_control::CheckSuccessResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const  ::fleet_control::CheckSuccessResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::fleet_control::CheckSuccessResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/CheckSuccessResponse";
  }

  static const char* value(const  ::fleet_control::CheckSuccessResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fleet_control::CheckSuccessResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool success\n\
\n\
\n\
";
  }

  static const char* value(const  ::fleet_control::CheckSuccessResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::fleet_control::CheckSuccessResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fleet_control::CheckSuccessRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CheckSuccessRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fleet_control::CheckSuccessResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CheckSuccessResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<fleet_control::CheckSuccess> {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const fleet_control::CheckSuccess&) { return value(); } 
};

template<>
struct DataType<fleet_control::CheckSuccess> {
  static const char* value() 
  {
    return "fleet_control/CheckSuccess";
  }

  static const char* value(const fleet_control::CheckSuccess&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<fleet_control::CheckSuccessRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const fleet_control::CheckSuccessRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<fleet_control::CheckSuccessRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/CheckSuccess";
  }

  static const char* value(const fleet_control::CheckSuccessRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<fleet_control::CheckSuccessResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const fleet_control::CheckSuccessResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<fleet_control::CheckSuccessResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/CheckSuccess";
  }

  static const char* value(const fleet_control::CheckSuccessResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // FLEET_CONTROL_SERVICE_CHECKSUCCESS_H

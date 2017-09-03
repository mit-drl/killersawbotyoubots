/* Auto-generated by genmsg_cpp for file /home/drl-mocap/ros_stacks/drl-youbot/fleet_control/msg/FleetDebug.msg */
#ifndef FLEET_CONTROL_MESSAGE_FLEETDEBUG_H
#define FLEET_CONTROL_MESSAGE_FLEETDEBUG_H
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


namespace fleet_control
{
template <class ContainerAllocator>
struct FleetDebug_ {
  typedef FleetDebug_<ContainerAllocator> Type;

  FleetDebug_()
  : data()
  {
  }

  FleetDebug_(const ContainerAllocator& _alloc)
  : data(_alloc)
  {
  }

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _data_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  data;


  typedef boost::shared_ptr< ::fleet_control::FleetDebug_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fleet_control::FleetDebug_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct FleetDebug
typedef  ::fleet_control::FleetDebug_<std::allocator<void> > FleetDebug;

typedef boost::shared_ptr< ::fleet_control::FleetDebug> FleetDebugPtr;
typedef boost::shared_ptr< ::fleet_control::FleetDebug const> FleetDebugConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::fleet_control::FleetDebug_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::fleet_control::FleetDebug_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace fleet_control

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::fleet_control::FleetDebug_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::fleet_control::FleetDebug_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::fleet_control::FleetDebug_<ContainerAllocator> > {
  static const char* value() 
  {
    return "420cd38b6b071cd49f2970c3e2cee511";
  }

  static const char* value(const  ::fleet_control::FleetDebug_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x420cd38b6b071cd4ULL;
  static const uint64_t static_value2 = 0x9f2970c3e2cee511ULL;
};

template<class ContainerAllocator>
struct DataType< ::fleet_control::FleetDebug_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fleet_control/FleetDebug";
  }

  static const char* value(const  ::fleet_control::FleetDebug_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::fleet_control::FleetDebug_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32[] data\n\
\n\
";
  }

  static const char* value(const  ::fleet_control::FleetDebug_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::fleet_control::FleetDebug_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct FleetDebug_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fleet_control::FleetDebug_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::fleet_control::FleetDebug_<ContainerAllocator> & v) 
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.data[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // FLEET_CONTROL_MESSAGE_FLEETDEBUG_H


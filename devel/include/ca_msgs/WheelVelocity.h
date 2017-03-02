// Generated by gencpp from file ca_msgs/WheelVelocity.msg
// DO NOT EDIT!


#ifndef CA_MSGS_MESSAGE_WHEELVELOCITY_H
#define CA_MSGS_MESSAGE_WHEELVELOCITY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ca_msgs
{
template <class ContainerAllocator>
struct WheelVelocity_
{
  typedef WheelVelocity_<ContainerAllocator> Type;

  WheelVelocity_()
    : velocityLeft(0.0)
    , velocityRight(0.0)  {
    }
  WheelVelocity_(const ContainerAllocator& _alloc)
    : velocityLeft(0.0)
    , velocityRight(0.0)  {
  (void)_alloc;
    }



   typedef double _velocityLeft_type;
  _velocityLeft_type velocityLeft;

   typedef double _velocityRight_type;
  _velocityRight_type velocityRight;




  typedef boost::shared_ptr< ::ca_msgs::WheelVelocity_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ca_msgs::WheelVelocity_<ContainerAllocator> const> ConstPtr;

}; // struct WheelVelocity_

typedef ::ca_msgs::WheelVelocity_<std::allocator<void> > WheelVelocity;

typedef boost::shared_ptr< ::ca_msgs::WheelVelocity > WheelVelocityPtr;
typedef boost::shared_ptr< ::ca_msgs::WheelVelocity const> WheelVelocityConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ca_msgs::WheelVelocity_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ca_msgs::WheelVelocity_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ca_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ca_msgs': ['/home/joemelt101/catkin_ws/src/create_autonomy/ca_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ca_msgs::WheelVelocity_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ca_msgs::WheelVelocity_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ca_msgs::WheelVelocity_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ca_msgs::WheelVelocity_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ca_msgs::WheelVelocity_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ca_msgs::WheelVelocity_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ca_msgs::WheelVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6b7b70e8a4003594801b7cd9759d4202";
  }

  static const char* value(const ::ca_msgs::WheelVelocity_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6b7b70e8a4003594ULL;
  static const uint64_t static_value2 = 0x801b7cd9759d4202ULL;
};

template<class ContainerAllocator>
struct DataType< ::ca_msgs::WheelVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca_msgs/WheelVelocity";
  }

  static const char* value(const ::ca_msgs::WheelVelocity_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ca_msgs::WheelVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 velocityLeft\n\
float64 velocityRight\n\
";
  }

  static const char* value(const ::ca_msgs::WheelVelocity_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ca_msgs::WheelVelocity_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.velocityLeft);
      stream.next(m.velocityRight);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WheelVelocity_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ca_msgs::WheelVelocity_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ca_msgs::WheelVelocity_<ContainerAllocator>& v)
  {
    s << indent << "velocityLeft: ";
    Printer<double>::stream(s, indent + "  ", v.velocityLeft);
    s << indent << "velocityRight: ";
    Printer<double>::stream(s, indent + "  ", v.velocityRight);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CA_MSGS_MESSAGE_WHEELVELOCITY_H
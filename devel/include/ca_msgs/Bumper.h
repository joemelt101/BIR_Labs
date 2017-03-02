// Generated by gencpp from file ca_msgs/Bumper.msg
// DO NOT EDIT!


#ifndef CA_MSGS_MESSAGE_BUMPER_H
#define CA_MSGS_MESSAGE_BUMPER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ca_msgs
{
template <class ContainerAllocator>
struct Bumper_
{
  typedef Bumper_<ContainerAllocator> Type;

  Bumper_()
    : header()
    , is_left_pressed(false)
    , is_right_pressed(false)
    , is_light_left(false)
    , is_light_front_left(false)
    , is_light_center_left(false)
    , is_light_center_right(false)
    , is_light_front_right(false)
    , is_light_right(false)
    , light_signal_left(0)
    , light_signal_front_left(0)
    , light_signal_center_left(0)
    , light_signal_center_right(0)
    , light_signal_front_right(0)
    , light_signal_right(0)  {
    }
  Bumper_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , is_left_pressed(false)
    , is_right_pressed(false)
    , is_light_left(false)
    , is_light_front_left(false)
    , is_light_center_left(false)
    , is_light_center_right(false)
    , is_light_front_right(false)
    , is_light_right(false)
    , light_signal_left(0)
    , light_signal_front_left(0)
    , light_signal_center_left(0)
    , light_signal_center_right(0)
    , light_signal_front_right(0)
    , light_signal_right(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _is_left_pressed_type;
  _is_left_pressed_type is_left_pressed;

   typedef uint8_t _is_right_pressed_type;
  _is_right_pressed_type is_right_pressed;

   typedef uint8_t _is_light_left_type;
  _is_light_left_type is_light_left;

   typedef uint8_t _is_light_front_left_type;
  _is_light_front_left_type is_light_front_left;

   typedef uint8_t _is_light_center_left_type;
  _is_light_center_left_type is_light_center_left;

   typedef uint8_t _is_light_center_right_type;
  _is_light_center_right_type is_light_center_right;

   typedef uint8_t _is_light_front_right_type;
  _is_light_front_right_type is_light_front_right;

   typedef uint8_t _is_light_right_type;
  _is_light_right_type is_light_right;

   typedef uint16_t _light_signal_left_type;
  _light_signal_left_type light_signal_left;

   typedef uint16_t _light_signal_front_left_type;
  _light_signal_front_left_type light_signal_front_left;

   typedef uint16_t _light_signal_center_left_type;
  _light_signal_center_left_type light_signal_center_left;

   typedef uint16_t _light_signal_center_right_type;
  _light_signal_center_right_type light_signal_center_right;

   typedef uint16_t _light_signal_front_right_type;
  _light_signal_front_right_type light_signal_front_right;

   typedef uint16_t _light_signal_right_type;
  _light_signal_right_type light_signal_right;




  typedef boost::shared_ptr< ::ca_msgs::Bumper_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ca_msgs::Bumper_<ContainerAllocator> const> ConstPtr;

}; // struct Bumper_

typedef ::ca_msgs::Bumper_<std::allocator<void> > Bumper;

typedef boost::shared_ptr< ::ca_msgs::Bumper > BumperPtr;
typedef boost::shared_ptr< ::ca_msgs::Bumper const> BumperConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ca_msgs::Bumper_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ca_msgs::Bumper_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ca_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ca_msgs': ['/home/joemelt101/catkin_ws/src/create_autonomy/ca_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ca_msgs::Bumper_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ca_msgs::Bumper_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ca_msgs::Bumper_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ca_msgs::Bumper_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ca_msgs::Bumper_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ca_msgs::Bumper_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ca_msgs::Bumper_<ContainerAllocator> >
{
  static const char* value()
  {
    return "18fe5b1d31e6a8db180b924157ac665e";
  }

  static const char* value(const ::ca_msgs::Bumper_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x18fe5b1d31e6a8dbULL;
  static const uint64_t static_value2 = 0x180b924157ac665eULL;
};

template<class ContainerAllocator>
struct DataType< ::ca_msgs::Bumper_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca_msgs/Bumper";
  }

  static const char* value(const ::ca_msgs::Bumper_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ca_msgs::Bumper_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
# Contact sensors\n\
bool is_left_pressed\n\
bool is_right_pressed\n\
\n\
# Bumper light sensors (Create 2 only) in order from left to right\n\
# Value = true if an obstacle detected\n\
bool is_light_left\n\
bool is_light_front_left\n\
bool is_light_center_left\n\
bool is_light_center_right\n\
bool is_light_front_right\n\
bool is_light_right\n\
\n\
# Raw light sensor signals\n\
# Values in range [0, 4095]\n\
uint16 light_signal_left\n\
uint16 light_signal_front_left\n\
uint16 light_signal_center_left\n\
uint16 light_signal_center_right\n\
uint16 light_signal_front_right\n\
uint16 light_signal_right\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::ca_msgs::Bumper_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ca_msgs::Bumper_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.is_left_pressed);
      stream.next(m.is_right_pressed);
      stream.next(m.is_light_left);
      stream.next(m.is_light_front_left);
      stream.next(m.is_light_center_left);
      stream.next(m.is_light_center_right);
      stream.next(m.is_light_front_right);
      stream.next(m.is_light_right);
      stream.next(m.light_signal_left);
      stream.next(m.light_signal_front_left);
      stream.next(m.light_signal_center_left);
      stream.next(m.light_signal_center_right);
      stream.next(m.light_signal_front_right);
      stream.next(m.light_signal_right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Bumper_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ca_msgs::Bumper_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ca_msgs::Bumper_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "is_left_pressed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_left_pressed);
    s << indent << "is_right_pressed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_right_pressed);
    s << indent << "is_light_left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_light_left);
    s << indent << "is_light_front_left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_light_front_left);
    s << indent << "is_light_center_left: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_light_center_left);
    s << indent << "is_light_center_right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_light_center_right);
    s << indent << "is_light_front_right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_light_front_right);
    s << indent << "is_light_right: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_light_right);
    s << indent << "light_signal_left: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.light_signal_left);
    s << indent << "light_signal_front_left: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.light_signal_front_left);
    s << indent << "light_signal_center_left: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.light_signal_center_left);
    s << indent << "light_signal_center_right: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.light_signal_center_right);
    s << indent << "light_signal_front_right: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.light_signal_front_right);
    s << indent << "light_signal_right: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.light_signal_right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CA_MSGS_MESSAGE_BUMPER_H

// Generated by gencpp from file robot_msgs/RobotMsg.msg
// DO NOT EDIT!


#ifndef ROBOT_MSGS_MESSAGE_ROBOTMSG_H
#define ROBOT_MSGS_MESSAGE_ROBOTMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace robot_msgs
{
template <class ContainerAllocator>
struct RobotMsg_
{
  typedef RobotMsg_<ContainerAllocator> Type;

  RobotMsg_()
    : header()
    , detection(false)
    , rho(0.0)
    , theta(0.0)
    , psi(0.0)
    , x(0.0)
    , y(0.0)
    , angle(0.0)
    , collision(false)  {
    }
  RobotMsg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , detection(false)
    , rho(0.0)
    , theta(0.0)
    , psi(0.0)
    , x(0.0)
    , y(0.0)
    , angle(0.0)
    , collision(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _detection_type;
  _detection_type detection;

   typedef float _rho_type;
  _rho_type rho;

   typedef float _theta_type;
  _theta_type theta;

   typedef float _psi_type;
  _psi_type psi;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _angle_type;
  _angle_type angle;

   typedef uint8_t _collision_type;
  _collision_type collision;




  typedef boost::shared_ptr< ::robot_msgs::RobotMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_msgs::RobotMsg_<ContainerAllocator> const> ConstPtr;

}; // struct RobotMsg_

typedef ::robot_msgs::RobotMsg_<std::allocator<void> > RobotMsg;

typedef boost::shared_ptr< ::robot_msgs::RobotMsg > RobotMsgPtr;
typedef boost::shared_ptr< ::robot_msgs::RobotMsg const> RobotMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_msgs::RobotMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_msgs::RobotMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robot_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'robot_msgs': ['/home/david/robotx/catkin_ws/src/robot_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::RobotMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::RobotMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::RobotMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::RobotMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::RobotMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::RobotMsg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_msgs::RobotMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a67b7856741f6c2cdc71b30825fcf226";
  }

  static const char* value(const ::robot_msgs::RobotMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa67b7856741f6c2cULL;
  static const uint64_t static_value2 = 0xdc71b30825fcf226ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_msgs::RobotMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_msgs/RobotMsg";
  }

  static const char* value(const ::robot_msgs::RobotMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_msgs::RobotMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
bool detection\n\
float32 rho\n\
float32 theta\n\
float32 psi\n\
float32 x\n\
float32 y\n\
float32 angle\n\
bool collision\n\
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

  static const char* value(const ::robot_msgs::RobotMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_msgs::RobotMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.detection);
      stream.next(m.rho);
      stream.next(m.theta);
      stream.next(m.psi);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.angle);
      stream.next(m.collision);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_msgs::RobotMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_msgs::RobotMsg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "detection: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.detection);
    s << indent << "rho: ";
    Printer<float>::stream(s, indent + "  ", v.rho);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
    s << indent << "psi: ";
    Printer<float>::stream(s, indent + "  ", v.psi);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
    s << indent << "collision: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.collision);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MSGS_MESSAGE_ROBOTMSG_H
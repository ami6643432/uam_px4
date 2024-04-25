// Generated by gencpp from file dynamixel_workbench_msgs/WheelCommandRequest.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_WHEELCOMMANDREQUEST_H
#define DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_WHEELCOMMANDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dynamixel_workbench_msgs
{
template <class ContainerAllocator>
struct WheelCommandRequest_
{
  typedef WheelCommandRequest_<ContainerAllocator> Type;

  WheelCommandRequest_()
    : right_vel(0.0)
    , left_vel(0.0)  {
    }
  WheelCommandRequest_(const ContainerAllocator& _alloc)
    : right_vel(0.0)
    , left_vel(0.0)  {
  (void)_alloc;
    }



   typedef float _right_vel_type;
  _right_vel_type right_vel;

   typedef float _left_vel_type;
  _left_vel_type left_vel;





  typedef boost::shared_ptr< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> const> ConstPtr;

}; // struct WheelCommandRequest_

typedef ::dynamixel_workbench_msgs::WheelCommandRequest_<std::allocator<void> > WheelCommandRequest;

typedef boost::shared_ptr< ::dynamixel_workbench_msgs::WheelCommandRequest > WheelCommandRequestPtr;
typedef boost::shared_ptr< ::dynamixel_workbench_msgs::WheelCommandRequest const> WheelCommandRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator1> & lhs, const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator2> & rhs)
{
  return lhs.right_vel == rhs.right_vel &&
    lhs.left_vel == rhs.left_vel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator1> & lhs, const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dynamixel_workbench_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0cfb8c816ece0d38d0a1c8583bdd4252";
  }

  static const char* value(const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0cfb8c816ece0d38ULL;
  static const uint64_t static_value2 = 0xd0a1c8583bdd4252ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamixel_workbench_msgs/WheelCommandRequest";
  }

  static const char* value(const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message is used to send velocity command to dynamixel\n"
"\n"
"float32 right_vel\n"
"float32 left_vel\n"
;
  }

  static const char* value(const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.right_vel);
      stream.next(m.left_vel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WheelCommandRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dynamixel_workbench_msgs::WheelCommandRequest_<ContainerAllocator>& v)
  {
    s << indent << "right_vel: ";
    Printer<float>::stream(s, indent + "  ", v.right_vel);
    s << indent << "left_vel: ";
    Printer<float>::stream(s, indent + "  ", v.left_vel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_WHEELCOMMANDREQUEST_H

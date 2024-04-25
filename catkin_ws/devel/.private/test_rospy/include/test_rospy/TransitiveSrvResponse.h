// Generated by gencpp from file test_rospy/TransitiveSrvResponse.msg
// DO NOT EDIT!


#ifndef TEST_ROSPY_MESSAGE_TRANSITIVESRVRESPONSE_H
#define TEST_ROSPY_MESSAGE_TRANSITIVESRVRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace test_rospy
{
template <class ContainerAllocator>
struct TransitiveSrvResponse_
{
  typedef TransitiveSrvResponse_<ContainerAllocator> Type;

  TransitiveSrvResponse_()
    : a(0)  {
    }
  TransitiveSrvResponse_(const ContainerAllocator& _alloc)
    : a(0)  {
  (void)_alloc;
    }



   typedef int32_t _a_type;
  _a_type a;





  typedef boost::shared_ptr< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TransitiveSrvResponse_

typedef ::test_rospy::TransitiveSrvResponse_<std::allocator<void> > TransitiveSrvResponse;

typedef boost::shared_ptr< ::test_rospy::TransitiveSrvResponse > TransitiveSrvResponsePtr;
typedef boost::shared_ptr< ::test_rospy::TransitiveSrvResponse const> TransitiveSrvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator1> & lhs, const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator2> & rhs)
{
  return lhs.a == rhs.a;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator1> & lhs, const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace test_rospy

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5c9fb1a886e81e3162a5c87bf55c072b";
  }

  static const char* value(const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5c9fb1a886e81e31ULL;
  static const uint64_t static_value2 = 0x62a5c87bf55c072bULL;
};

template<class ContainerAllocator>
struct DataType< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_rospy/TransitiveSrvResponse";
  }

  static const char* value(const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 a\n"
"\n"
"\n"
;
  }

  static const char* value(const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TransitiveSrvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_rospy::TransitiveSrvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_rospy::TransitiveSrvResponse_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<int32_t>::stream(s, indent + "  ", v.a);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEST_ROSPY_MESSAGE_TRANSITIVESRVRESPONSE_H

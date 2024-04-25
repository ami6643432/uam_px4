// Generated by gencpp from file test_roscpp/FixedLength.msg
// DO NOT EDIT!


#ifndef TEST_ROSCPP_MESSAGE_FIXEDLENGTH_H
#define TEST_ROSCPP_MESSAGE_FIXEDLENGTH_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace test_roscpp
{
template <class ContainerAllocator>
struct FixedLength_
{
  typedef FixedLength_<ContainerAllocator> Type;

  FixedLength_()
    : a(0)
    , b(0.0)  {
    }
  FixedLength_(const ContainerAllocator& _alloc)
    : a(0)
    , b(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _a_type;
  _a_type a;

   typedef float _b_type;
  _b_type b;





  typedef boost::shared_ptr< ::test_roscpp::FixedLength_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::test_roscpp::FixedLength_<ContainerAllocator> const> ConstPtr;

}; // struct FixedLength_

typedef ::test_roscpp::FixedLength_<std::allocator<void> > FixedLength;

typedef boost::shared_ptr< ::test_roscpp::FixedLength > FixedLengthPtr;
typedef boost::shared_ptr< ::test_roscpp::FixedLength const> FixedLengthConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_roscpp::FixedLength_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::test_roscpp::FixedLength_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::test_roscpp::FixedLength_<ContainerAllocator1> & lhs, const ::test_roscpp::FixedLength_<ContainerAllocator2> & rhs)
{
  return lhs.a == rhs.a &&
    lhs.b == rhs.b;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::test_roscpp::FixedLength_<ContainerAllocator1> & lhs, const ::test_roscpp::FixedLength_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace test_roscpp

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::FixedLength_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_roscpp::FixedLength_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::FixedLength_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_roscpp::FixedLength_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::FixedLength_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_roscpp::FixedLength_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_roscpp::FixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "74143e1090cf694294f589605908b555";
  }

  static const char* value(const ::test_roscpp::FixedLength_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x74143e1090cf6942ULL;
  static const uint64_t static_value2 = 0x94f589605908b555ULL;
};

template<class ContainerAllocator>
struct DataType< ::test_roscpp::FixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_roscpp/FixedLength";
  }

  static const char* value(const ::test_roscpp::FixedLength_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_roscpp::FixedLength_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 a\n"
"float32 b\n"
;
  }

  static const char* value(const ::test_roscpp::FixedLength_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_roscpp::FixedLength_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FixedLength_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_roscpp::FixedLength_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_roscpp::FixedLength_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<float>::stream(s, indent + "  ", v.b);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEST_ROSCPP_MESSAGE_FIXEDLENGTH_H
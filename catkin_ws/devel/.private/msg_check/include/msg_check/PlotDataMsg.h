// Generated by gencpp from file msg_check/PlotDataMsg.msg
// DO NOT EDIT!


#ifndef MSG_CHECK_MESSAGE_PLOTDATAMSG_H
#define MSG_CHECK_MESSAGE_PLOTDATAMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Quaternion.h>

namespace msg_check
{
template <class ContainerAllocator>
struct PlotDataMsg_
{
  typedef PlotDataMsg_<ContainerAllocator> Type;

  PlotDataMsg_()
    : header()
    , curpos()
    , despos()
    , curvel()
    , desvel()
    , curacc()
    , desacc()
    , poserr()
    , velerr()
    , curor()
    , desor()
    , thrust(0.0)
    , M1_pos_err(0.0)
    , M2_pos_err(0.0)
    , M1_vel_err(0.0)
    , M2_vel_err(0.0)  {
    }
  PlotDataMsg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , curpos(_alloc)
    , despos(_alloc)
    , curvel(_alloc)
    , desvel(_alloc)
    , curacc(_alloc)
    , desacc(_alloc)
    , poserr(_alloc)
    , velerr(_alloc)
    , curor(_alloc)
    , desor(_alloc)
    , thrust(0.0)
    , M1_pos_err(0.0)
    , M2_pos_err(0.0)
    , M1_vel_err(0.0)
    , M2_vel_err(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _curpos_type;
  _curpos_type curpos;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _despos_type;
  _despos_type despos;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _curvel_type;
  _curvel_type curvel;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _desvel_type;
  _desvel_type desvel;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _curacc_type;
  _curacc_type curacc;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _desacc_type;
  _desacc_type desacc;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _poserr_type;
  _poserr_type poserr;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velerr_type;
  _velerr_type velerr;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _curor_type;
  _curor_type curor;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _desor_type;
  _desor_type desor;

   typedef double _thrust_type;
  _thrust_type thrust;

   typedef double _M1_pos_err_type;
  _M1_pos_err_type M1_pos_err;

   typedef double _M2_pos_err_type;
  _M2_pos_err_type M2_pos_err;

   typedef double _M1_vel_err_type;
  _M1_vel_err_type M1_vel_err;

   typedef double _M2_vel_err_type;
  _M2_vel_err_type M2_vel_err;





  typedef boost::shared_ptr< ::msg_check::PlotDataMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_check::PlotDataMsg_<ContainerAllocator> const> ConstPtr;

}; // struct PlotDataMsg_

typedef ::msg_check::PlotDataMsg_<std::allocator<void> > PlotDataMsg;

typedef boost::shared_ptr< ::msg_check::PlotDataMsg > PlotDataMsgPtr;
typedef boost::shared_ptr< ::msg_check::PlotDataMsg const> PlotDataMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_check::PlotDataMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_check::PlotDataMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_check::PlotDataMsg_<ContainerAllocator1> & lhs, const ::msg_check::PlotDataMsg_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.curpos == rhs.curpos &&
    lhs.despos == rhs.despos &&
    lhs.curvel == rhs.curvel &&
    lhs.desvel == rhs.desvel &&
    lhs.curacc == rhs.curacc &&
    lhs.desacc == rhs.desacc &&
    lhs.poserr == rhs.poserr &&
    lhs.velerr == rhs.velerr &&
    lhs.curor == rhs.curor &&
    lhs.desor == rhs.desor &&
    lhs.thrust == rhs.thrust &&
    lhs.M1_pos_err == rhs.M1_pos_err &&
    lhs.M2_pos_err == rhs.M2_pos_err &&
    lhs.M1_vel_err == rhs.M1_vel_err &&
    lhs.M2_vel_err == rhs.M2_vel_err;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_check::PlotDataMsg_<ContainerAllocator1> & lhs, const ::msg_check::PlotDataMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_check

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_check::PlotDataMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_check::PlotDataMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_check::PlotDataMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_check::PlotDataMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_check::PlotDataMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_check::PlotDataMsg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_check::PlotDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0d96fc0f4d709ad5a1e1b4d1fe446936";
  }

  static const char* value(const ::msg_check::PlotDataMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0d96fc0f4d709ad5ULL;
  static const uint64_t static_value2 = 0xa1e1b4d1fe446936ULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_check::PlotDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_check/PlotDataMsg";
  }

  static const char* value(const ::msg_check::PlotDataMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_check::PlotDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"geometry_msgs/Vector3 curpos\n"
"geometry_msgs/Vector3 despos\n"
"\n"
"geometry_msgs/Vector3 curvel\n"
"geometry_msgs/Vector3 desvel\n"
"\n"
"geometry_msgs/Vector3 curacc\n"
"geometry_msgs/Vector3 desacc\n"
"\n"
"geometry_msgs/Vector3 poserr\n"
"geometry_msgs/Vector3 velerr\n"
"\n"
"geometry_msgs/Quaternion curor\n"
"geometry_msgs/Quaternion desor\n"
"\n"
"float64 thrust\n"
"float64 M1_pos_err\n"
"float64 M2_pos_err\n"
"float64 M1_vel_err\n"
"float64 M2_vel_err\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::msg_check::PlotDataMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_check::PlotDataMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.curpos);
      stream.next(m.despos);
      stream.next(m.curvel);
      stream.next(m.desvel);
      stream.next(m.curacc);
      stream.next(m.desacc);
      stream.next(m.poserr);
      stream.next(m.velerr);
      stream.next(m.curor);
      stream.next(m.desor);
      stream.next(m.thrust);
      stream.next(m.M1_pos_err);
      stream.next(m.M2_pos_err);
      stream.next(m.M1_vel_err);
      stream.next(m.M2_vel_err);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlotDataMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_check::PlotDataMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_check::PlotDataMsg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "curpos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.curpos);
    s << indent << "despos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.despos);
    s << indent << "curvel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.curvel);
    s << indent << "desvel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.desvel);
    s << indent << "curacc: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.curacc);
    s << indent << "desacc: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.desacc);
    s << indent << "poserr: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.poserr);
    s << indent << "velerr: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velerr);
    s << indent << "curor: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.curor);
    s << indent << "desor: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.desor);
    s << indent << "thrust: ";
    Printer<double>::stream(s, indent + "  ", v.thrust);
    s << indent << "M1_pos_err: ";
    Printer<double>::stream(s, indent + "  ", v.M1_pos_err);
    s << indent << "M2_pos_err: ";
    Printer<double>::stream(s, indent + "  ", v.M2_pos_err);
    s << indent << "M1_vel_err: ";
    Printer<double>::stream(s, indent + "  ", v.M1_vel_err);
    s << indent << "M2_vel_err: ";
    Printer<double>::stream(s, indent + "  ", v.M2_vel_err);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_CHECK_MESSAGE_PLOTDATAMSG_H

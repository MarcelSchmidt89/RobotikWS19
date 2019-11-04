// Generated by gencpp from file simple_parking_maneuver/ParkingManeuverResponse.msg
// DO NOT EDIT!


#ifndef SIMPLE_PARKING_MANEUVER_MESSAGE_PARKINGMANEUVERRESPONSE_H
#define SIMPLE_PARKING_MANEUVER_MESSAGE_PARKINGMANEUVERRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace simple_parking_maneuver
{
template <class ContainerAllocator>
struct ParkingManeuverResponse_
{
  typedef ParkingManeuverResponse_<ContainerAllocator> Type;

  ParkingManeuverResponse_()
    : info()  {
    }
  ParkingManeuverResponse_(const ContainerAllocator& _alloc)
    : info(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _info_type;
  _info_type info;





  typedef boost::shared_ptr< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ParkingManeuverResponse_

typedef ::simple_parking_maneuver::ParkingManeuverResponse_<std::allocator<void> > ParkingManeuverResponse;

typedef boost::shared_ptr< ::simple_parking_maneuver::ParkingManeuverResponse > ParkingManeuverResponsePtr;
typedef boost::shared_ptr< ::simple_parking_maneuver::ParkingManeuverResponse const> ParkingManeuverResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace simple_parking_maneuver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c10fc26d5cca9a4b9114f5fc5dea9570";
  }

  static const char* value(const ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc10fc26d5cca9a4bULL;
  static const uint64_t static_value2 = 0x9114f5fc5dea9570ULL;
};

template<class ContainerAllocator>
struct DataType< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "simple_parking_maneuver/ParkingManeuverResponse";
  }

  static const char* value(const ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string info\n"
;
  }

  static const char* value(const ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.info);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ParkingManeuverResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::simple_parking_maneuver::ParkingManeuverResponse_<ContainerAllocator>& v)
  {
    s << indent << "info: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.info);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SIMPLE_PARKING_MANEUVER_MESSAGE_PARKINGMANEUVERRESPONSE_H

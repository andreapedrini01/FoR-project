// Generated by gencpp from file ros_impedance_controller/generic_float.msg
// DO NOT EDIT!


#ifndef ROS_IMPEDANCE_CONTROLLER_MESSAGE_GENERIC_FLOAT_H
#define ROS_IMPEDANCE_CONTROLLER_MESSAGE_GENERIC_FLOAT_H

#include <ros/service_traits.h>


#include <motion/generic_floatRequest.h>
#include <motion/generic_floatResponse.h>


namespace ros_impedance_controller
{

struct generic_float
{

typedef generic_floatRequest Request;
typedef generic_floatResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct generic_float
} // namespace ros_impedance_controller


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ros_impedance_controller::generic_float > {
  static const char* value()
  {
    return "28f45893a01d82a5ea9c839d84112d68";
  }

  static const char* value(const ::ros_impedance_controller::generic_float&) { return value(); }
};

template<>
struct DataType< ::ros_impedance_controller::generic_float > {
  static const char* value()
  {
    return "ros_impedance_controller/generic_float";
  }

  static const char* value(const ::ros_impedance_controller::generic_float&) { return value(); }
};


// service_traits::MD5Sum< ::ros_impedance_controller::generic_floatRequest> should match
// service_traits::MD5Sum< ::ros_impedance_controller::generic_float >
template<>
struct MD5Sum< ::ros_impedance_controller::generic_floatRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ros_impedance_controller::generic_float >::value();
  }
  static const char* value(const ::ros_impedance_controller::generic_floatRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_impedance_controller::generic_floatRequest> should match
// service_traits::DataType< ::ros_impedance_controller::generic_float >
template<>
struct DataType< ::ros_impedance_controller::generic_floatRequest>
{
  static const char* value()
  {
    return DataType< ::ros_impedance_controller::generic_float >::value();
  }
  static const char* value(const ::ros_impedance_controller::generic_floatRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ros_impedance_controller::generic_floatResponse> should match
// service_traits::MD5Sum< ::ros_impedance_controller::generic_float >
template<>
struct MD5Sum< ::ros_impedance_controller::generic_floatResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ros_impedance_controller::generic_float >::value();
  }
  static const char* value(const ::ros_impedance_controller::generic_floatResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_impedance_controller::generic_floatResponse> should match
// service_traits::DataType< ::ros_impedance_controller::generic_float >
template<>
struct DataType< ::ros_impedance_controller::generic_floatResponse>
{
  static const char* value()
  {
    return DataType< ::ros_impedance_controller::generic_float >::value();
  }
  static const char* value(const ::ros_impedance_controller::generic_floatResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROS_IMPEDANCE_CONTROLLER_MESSAGE_GENERIC_FLOAT_H

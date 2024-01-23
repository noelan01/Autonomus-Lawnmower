// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MapInfo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/map_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MapInfo_poi_paths
{
public:
  explicit Init_MapInfo_poi_paths(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MapInfo poi_paths(::hqv_public_interface::msg::MapInfo::_poi_paths_type arg)
  {
    msg_.poi_paths = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_transport_paths
{
public:
  explicit Init_MapInfo_transport_paths(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  Init_MapInfo_poi_paths transport_paths(::hqv_public_interface::msg::MapInfo::_transport_paths_type arg)
  {
    msg_.transport_paths = std::move(arg);
    return Init_MapInfo_poi_paths(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_hidden_areas
{
public:
  explicit Init_MapInfo_hidden_areas(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  Init_MapInfo_transport_paths hidden_areas(::hqv_public_interface::msg::MapInfo::_hidden_areas_type arg)
  {
    msg_.hidden_areas = std::move(arg);
    return Init_MapInfo_transport_paths(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_forbidden_areas
{
public:
  explicit Init_MapInfo_forbidden_areas(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  Init_MapInfo_hidden_areas forbidden_areas(::hqv_public_interface::msg::MapInfo::_forbidden_areas_type arg)
  {
    msg_.forbidden_areas = std::move(arg);
    return Init_MapInfo_hidden_areas(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_transport_areas
{
public:
  explicit Init_MapInfo_transport_areas(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  Init_MapInfo_forbidden_areas transport_areas(::hqv_public_interface::msg::MapInfo::_transport_areas_type arg)
  {
    msg_.transport_areas = std::move(arg);
    return Init_MapInfo_forbidden_areas(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_working_areas
{
public:
  explicit Init_MapInfo_working_areas(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  Init_MapInfo_transport_areas working_areas(::hqv_public_interface::msg::MapInfo::_working_areas_type arg)
  {
    msg_.working_areas = std::move(arg);
    return Init_MapInfo_transport_areas(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_charging_station
{
public:
  explicit Init_MapInfo_charging_station(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  Init_MapInfo_working_areas charging_station(::hqv_public_interface::msg::MapInfo::_charging_station_type arg)
  {
    msg_.charging_station = std::move(arg);
    return Init_MapInfo_working_areas(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_pc_pos
{
public:
  explicit Init_MapInfo_pc_pos(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  Init_MapInfo_charging_station pc_pos(::hqv_public_interface::msg::MapInfo::_pc_pos_type arg)
  {
    msg_.pc_pos = std::move(arg);
    return Init_MapInfo_charging_station(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_cs_pos
{
public:
  explicit Init_MapInfo_cs_pos(::hqv_public_interface::msg::MapInfo & msg)
  : msg_(msg)
  {}
  Init_MapInfo_pc_pos cs_pos(::hqv_public_interface::msg::MapInfo::_cs_pos_type arg)
  {
    msg_.cs_pos = std::move(arg);
    return Init_MapInfo_pc_pos(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

class Init_MapInfo_map_name
{
public:
  Init_MapInfo_map_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapInfo_cs_pos map_name(::hqv_public_interface::msg::MapInfo::_map_name_type arg)
  {
    msg_.map_name = std::move(arg);
    return Init_MapInfo_cs_pos(msg_);
  }

private:
  ::hqv_public_interface::msg::MapInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MapInfo>()
{
  return hqv_public_interface::msg::builder::Init_MapInfo_map_name();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__BUILDER_HPP_

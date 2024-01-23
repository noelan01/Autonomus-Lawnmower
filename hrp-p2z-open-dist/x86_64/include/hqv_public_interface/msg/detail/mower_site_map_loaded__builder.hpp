// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerSiteMapLoaded.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_site_map_loaded__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerSiteMapLoaded_sitename
{
public:
  explicit Init_MowerSiteMapLoaded_sitename(::hqv_public_interface::msg::MowerSiteMapLoaded & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerSiteMapLoaded sitename(::hqv_public_interface::msg::MowerSiteMapLoaded::_sitename_type arg)
  {
    msg_.sitename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSiteMapLoaded msg_;
};

class Init_MowerSiteMapLoaded_header
{
public:
  Init_MowerSiteMapLoaded_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerSiteMapLoaded_sitename header(::hqv_public_interface::msg::MowerSiteMapLoaded::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerSiteMapLoaded_sitename(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSiteMapLoaded msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerSiteMapLoaded>()
{
  return hqv_public_interface::msg::builder::Init_MowerSiteMapLoaded_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__BUILDER_HPP_

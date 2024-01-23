// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerSiteMapChecksumUpdate.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_CHECKSUM_UPDATE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_CHECKSUM_UPDATE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_site_map_checksum_update__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerSiteMapChecksumUpdate_description
{
public:
  explicit Init_MowerSiteMapChecksumUpdate_description(::hqv_public_interface::msg::MowerSiteMapChecksumUpdate & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerSiteMapChecksumUpdate description(::hqv_public_interface::msg::MowerSiteMapChecksumUpdate::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSiteMapChecksumUpdate msg_;
};

class Init_MowerSiteMapChecksumUpdate_checksum
{
public:
  explicit Init_MowerSiteMapChecksumUpdate_checksum(::hqv_public_interface::msg::MowerSiteMapChecksumUpdate & msg)
  : msg_(msg)
  {}
  Init_MowerSiteMapChecksumUpdate_description checksum(::hqv_public_interface::msg::MowerSiteMapChecksumUpdate::_checksum_type arg)
  {
    msg_.checksum = std::move(arg);
    return Init_MowerSiteMapChecksumUpdate_description(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSiteMapChecksumUpdate msg_;
};

class Init_MowerSiteMapChecksumUpdate_header
{
public:
  Init_MowerSiteMapChecksumUpdate_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerSiteMapChecksumUpdate_checksum header(::hqv_public_interface::msg::MowerSiteMapChecksumUpdate::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerSiteMapChecksumUpdate_checksum(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSiteMapChecksumUpdate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerSiteMapChecksumUpdate>()
{
  return hqv_public_interface::msg::builder::Init_MowerSiteMapChecksumUpdate_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_CHECKSUM_UPDATE__BUILDER_HPP_

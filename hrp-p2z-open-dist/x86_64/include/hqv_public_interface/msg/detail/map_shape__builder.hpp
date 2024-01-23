// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MapShape.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/map_shape__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MapShape_hidden
{
public:
  explicit Init_MapShape_hidden(::hqv_public_interface::msg::MapShape & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MapShape hidden(::hqv_public_interface::msg::MapShape::_hidden_type arg)
  {
    msg_.hidden = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MapShape msg_;
};

class Init_MapShape_fill_color
{
public:
  explicit Init_MapShape_fill_color(::hqv_public_interface::msg::MapShape & msg)
  : msg_(msg)
  {}
  Init_MapShape_hidden fill_color(::hqv_public_interface::msg::MapShape::_fill_color_type arg)
  {
    msg_.fill_color = std::move(arg);
    return Init_MapShape_hidden(msg_);
  }

private:
  ::hqv_public_interface::msg::MapShape msg_;
};

class Init_MapShape_stroke_color
{
public:
  explicit Init_MapShape_stroke_color(::hqv_public_interface::msg::MapShape & msg)
  : msg_(msg)
  {}
  Init_MapShape_fill_color stroke_color(::hqv_public_interface::msg::MapShape::_stroke_color_type arg)
  {
    msg_.stroke_color = std::move(arg);
    return Init_MapShape_fill_color(msg_);
  }

private:
  ::hqv_public_interface::msg::MapShape msg_;
};

class Init_MapShape_type
{
public:
  explicit Init_MapShape_type(::hqv_public_interface::msg::MapShape & msg)
  : msg_(msg)
  {}
  Init_MapShape_stroke_color type(::hqv_public_interface::msg::MapShape::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_MapShape_stroke_color(msg_);
  }

private:
  ::hqv_public_interface::msg::MapShape msg_;
};

class Init_MapShape_id
{
public:
  explicit Init_MapShape_id(::hqv_public_interface::msg::MapShape & msg)
  : msg_(msg)
  {}
  Init_MapShape_type id(::hqv_public_interface::msg::MapShape::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MapShape_type(msg_);
  }

private:
  ::hqv_public_interface::msg::MapShape msg_;
};

class Init_MapShape_polygon
{
public:
  Init_MapShape_polygon()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapShape_id polygon(::hqv_public_interface::msg::MapShape::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return Init_MapShape_id(msg_);
  }

private:
  ::hqv_public_interface::msg::MapShape msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MapShape>()
{
  return hqv_public_interface::msg::builder::Init_MapShape_polygon();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__BUILDER_HPP_

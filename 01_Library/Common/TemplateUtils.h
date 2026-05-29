/// @file TemplateUtils.h
/// @brief General-purpose templates for metaprogramming.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef TEMPLATE_UTILS_H
#define TEMPLATE_UTILS_H

template<typename... Types> struct TFirstTypeInPack;

template<typename First>
struct TFirstTypeInPack<First>
{
  using Type = First;
};

template<typename First, typename... Rest>
struct TFirstTypeInPack<First, Rest...>
{
  using Type = First;
};

#endif // TEMPLATE_UTILS_H

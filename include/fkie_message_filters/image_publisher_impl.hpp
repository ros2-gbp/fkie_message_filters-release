/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2025 Fraunhofer FKIE
 * Author: Timo Röhling
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_IMAGE_PUBLISHER_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_IMAGE_PUBLISHER_IMPL_HPP_
#pragma once

// IWYU pragma: private; include "image_publisher.hpp"

#include "image_publisher.hpp"
#include "version.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

template<template<typename> class Translate>
ImagePublisher<Translate>::ImagePublisher(rclcpp::Node::SharedPtr& node, const std::string& base_topic,
                                          const rclcpp::QoS& qos, const rclcpp::PublisherOptions& options)
{
    advertise(node, base_topic, qos, options);
}

template<template<typename> class Translate>
ImagePublisher<Translate>::~ImagePublisher()
{
    shutdown_monitor();
}

template<template<typename> class Translate>
bool ImagePublisher<Translate>::is_active() const noexcept
{
    return pub_.getNumSubscribers() > 0;
}

template<template<typename> class Translate>
std::string ImagePublisher<Translate>::topic() const noexcept
{
    return pub_.getTopic();
}

template<template<typename> class Translate>
void ImagePublisher<Translate>::advertise(rclcpp::Node::SharedPtr& node, const std::string& base_topic,
                                          const rclcpp::QoS& qos, const rclcpp::PublisherOptions& options)
{
#if FKIE_MF_IMAGE_TRANSPORT_VERSION >= FKIE_MF_VERSION_TUPLE(3, 2, 0)
    pub_ = image_transport::create_publisher(node.get(), base_topic, qos.get_rmw_qos_profile(), options);
#else
    pub_ = image_transport::create_publisher(node.get(), base_topic, qos.get_rmw_qos_profile());
#endif
    start_monitor(node);
    update_subscriber_state();
}

template<template<typename> class Translate>
void ImagePublisher<Translate>::receive(
    helpers::argument_t<typename Translate<sensor_msgs::msg::Image>::FilterType> img)
{
    Translate<sensor_msgs::msg::Image>::publish(pub_, img);
}

FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif

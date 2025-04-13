/****************************************************************************
 *
 * fkie_message_filters
 * Copyright © 2018-2025 Fraunhofer FKIE
 * Author: Timo Röhling
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
#ifndef INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_POLICY_BASE_IMPL_HPP_
#define INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_POLICY_BASE_IMPL_HPP_
#pragma once

#include "policy_base.hpp"

namespace fkie_message_filters
{

FKIE_MF_BEGIN_ABI_NAMESPACE

namespace combiner_policies
{

template<class... IOs>
void PolicyBase<IOs...>::set_emitter_callback(const EmitterCB& cb) noexcept
{
    emit_ = cb;
}

template<class... IOs>
void PolicyBase<IOs...>::emit(OutgoingTuple& out)
{
    if (emit_)
        emit_(out);
}

}  // namespace combiner_policies
FKIE_MF_END_ABI_NAMESPACE
}  // namespace fkie_message_filters

#endif /* INCLUDE_FKIE_MESSAGE_FILTERS_COMBINER_POLICIES_POLICY_BASE_IMPL_HPP_ */

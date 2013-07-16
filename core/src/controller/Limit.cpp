//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <rtt/types/Types.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/types/TemplateConstructor.hpp>
#include <base/SetupFunction.h>

#include "Limit.h"
#include <boost/make_shared.hpp>

namespace uxvcos {
namespace Controller {

namespace {
  typedef Limit<double> LimitType;

  int registerLimitType() {
    if (!RTT::types::Types()->getTypeInfo<LimitType>()) {
      RTT::types::Types()->addType(new RTT::types::TemplateTypeInfo<LimitType,true>("Limit"));
      RTT::types::Types()->type("Limit")->setCompositionFactory(boost::make_shared<LimitType::CompositionFactory>());
      RTT::types::Types()->type("Limit")->addConstructor(RTT::types::newConstructor(&LimitType::construct2, true));
      RTT::types::Types()->type("Limit")->addConstructor(RTT::types::newConstructor(&LimitType::construct1, true));
    }
    return 0;
  }

  SetupFunction init(&registerLimitType, "registerLimitType()");
}

} // namespace Controller
} // namespace uxvcos

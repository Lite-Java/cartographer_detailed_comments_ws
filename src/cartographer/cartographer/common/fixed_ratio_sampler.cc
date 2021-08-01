/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/common/fixed_ratio_sampler.h"

#include "glog/logging.h"

namespace cartographer {
namespace common {

FixedRatioSampler::FixedRatioSampler(const double ratio) : ratio_(ratio) {
  CHECK_GE(ratio, 0.);
  LOG_IF(WARNING, ratio == 0.) << "FixedRatioSampler is dropping all data.";
  CHECK_LE(ratio, 1.);
}

FixedRatioSampler::~FixedRatioSampler() {}

// 在比例小于ratio_时返回true, ratio_设置为1时都返回true, 也就是说使用所有的数据
//假设ratio_=0.5
/*num_samples_     num_pulses_ 
    0                  1             true
    1                  2             false
    1                  3             true
    2                  4             false
    2                  5             true
    3                  6             false
    3                  7             true
    可以这样理解num_samples_×（1/ ratio_）<num_pulses_时才为真，这时num_samples才+1,左侧就增加了1/ ratio_，右边num_pulses_也必须增加1/ ratio_才为真，这样就实现了每隔1/ ratio_抽样一次，妙啊！！
*/
bool FixedRatioSampler::Pulse() {
  ++num_pulses_;
  if (static_cast<double>(num_samples_) / num_pulses_ < ratio_) {
    ++num_samples_;
    return true;
  }
  // 返回false时代表数据可以不用,可以跳过计算
  return false;
}

std::string FixedRatioSampler::DebugString() {
  return std::to_string(num_samples_) + " (" +
         std::to_string(100. * num_samples_ / num_pulses_) + "%)";
}

}  // namespace common
}  // namespace cartographer

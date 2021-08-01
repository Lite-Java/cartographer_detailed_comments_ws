/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

constexpr float RangeDataCollator::kDefaultIntensityValue;

// todo: RangeDataCollator::AddRangeData 多个雷达数据的时间同步
sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    sensor::TimedPointCloudData timed_point_cloud_data) {
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  timed_point_cloud_data.intensities.resize(
      timed_point_cloud_data.ranges.size(), kDefaultIntensityValue);
  // TODO(gaschler): These two cases can probably be one.
      /**
       如果当前融合时间段内融合结果中已包含同一ID的传感器数据，则应采用最新的点云数据进行替换，但是结束的时间戳仍然采用原来的时间刻度，
       开始进行数据截取合并CropAndMerge()并返回，其CropAndMerge()下部分会详细讲解。
       简单理解如果有一个传感器频率较高，已经来过一帧并进行了缓存，另外一个未来，这个传感器又来一帧，
       则先进行截取合并送出结果（实际上就是上帧缓存的点云直接发出），然后将新来的一帧替换换来的buffer。
       * */
      // 此传感器类型数据已有
  if (id_to_pending_data_.count(sensor_id) != 0) {
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    current_end_ = id_to_pending_data_.at(sensor_id).time;// 采用旧的时间戳
    auto result = CropAndMerge();
    id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));//用新的数据替换原有的数据
    return result;
  }
  id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
      //订阅的传感器的topic还没有齐 若现在收到的数据类型未全，即期望收到种类未全，直接退出，无需融合
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }
  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);//pair.second.time返回的时间是本帧激光最晚的点
  }
      //current_end_是下次融合的开始时间，是本次融合的最后时间刻度，但其实是此次融合所有传感器中最早的时间戳
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    sensor::TimedPointCloudData& data = it->second;//带原点位置和时间戳
    const sensor::TimedPointCloud& ranges = it->second.ranges;// 仅点云（每一个点也都带有测量时间戳）
    const std::vector<float>& intensities = it->second.intensities;
/**一个传感点云中的时间戳为点云N个光束的起始时刻，
 * 但N个光束采样实际上也存在一定measurement间隔，一般在ros的激光驱动中都已添加，
 * 而cartographer中的TimedPointCloud点云类型，是将点云中每个点也都存储了与起始点测量时间间隔的时间戳。
 * 通过增加起始时刻的时间戳data.time + common::FromSeconds((*overlap_begin).time) ，
 * 则可精确获取每个端点的时间戳。上段代码的目的是此传感器的每个点云需要满足在current_start_和current_end_之间。
 * 由于可能存在多个传感器，在融合时，可能会导致前后时间重叠，因此进行时间段限制。
**/
    auto overlap_begin = ranges.begin();// 记录在所有点云中开始时间戳的位置，即上时刻集合的时间戳
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) <
               current_start_) {
      ++overlap_begin;
    }
    auto overlap_end = overlap_begin;// 记录所有点云结束时间戳的位置，即此时刻集合的时间戳
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <=
               current_end_) {
      ++overlap_end;
    }
    // 如果某个传感器点云前面时间戳早于当前集合定义的时间戳，则丢弃
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      std::size_t origin_index = result.origins.size();//获取下个插入的index，即当前集合的个数
      result.origins.push_back(data.origin);// 插入原点坐标
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));//// 获取此传感器时间与集合时间戳的误差，
      auto intensities_overlap_it =
          intensities.begin() + (overlap_begin - ranges.begin());
      result.ranges.reserve(result.ranges.size() +
                            std::distance(overlap_begin, overlap_end));
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it, ++intensities_overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{
            *overlap_it, *intensities_overlap_it, origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        point.point_time.time += time_correction;// 针对每个点时间戳进行修正 上段代码目的如果未被全部抛弃，则需要将点云每个点的偏移时间进行修正，调整到同一的current_end_为起始时刻（即本次融合统一起始时刻）。
        result.ranges.push_back(point);
      }
    }

    // Drop buffered points until overlap_end.
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);// 每个点都融合进去，则直接清除此ID的数据点
    } else if (overlap_end == ranges.begin()) {
      ++it;// 完全没融进去，则跳过用于下次融合
    } else {
      const auto intensities_overlap_end =
          intensities.begin() + (overlap_end - ranges.begin());
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end()),
          std::vector<float>(intensities_overlap_end, intensities.end())};
      ++it;
    }
  }

  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });// 对集合中所有点云，进行按照时间顺序排序
  return result;
}

}  // namespace mapping
}  // namespace cartographer

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

std_msgs/Header header
<<<<<<< HEAD
#//这里的[]表示什么意思 submap_list.submap.push_back(submap_entry)，为什用push_back,submap是数组还是vector注意这里的数组使用的是无长度限制的，
#也就是方扩号内没有东西。在使用的时候，不能够直接用数组赋值那样去做，它实际上是一个向量，往里面填充数据应使用c++中vector的push_back、resize之类的函数。
=======
>>>>>>> upstream/master
cartographer_ros_msgs/SubmapEntry[] submap

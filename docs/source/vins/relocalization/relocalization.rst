.. highlight:: c++

.. default-domain:: cpp

==============
Relocalization
==============

Loop Detection
--------------

Vins-Mono 利用词袋 DBoW2 做 Keyframe Database 的构建和查询。在建立闭环检测的数据
库时，关键帧的 Features 包括两部分：VIO 部分的 200 个强角点和 500 个 Fast 角点，
然后描述 子使用 BRIEF (因为旋转可观，匹配过程中对旋转有一定的适应性，所以不用使用 ORB)。

Feature Retrieval
-----------------

Tightly-Coupled Relocalization
------------------------------
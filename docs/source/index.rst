======
X-SLAM 
======

X-SLAM is an open source C++ demo for learn vision slam and lidar slam.
Through open source engineering, we can learn the following knowledge content.

  * **C++** learning tutorial
  * **CMake**  learning tutorial
  * **Eigen** learning tutorial
  * **OpenCV** learning tutorial
  * **Quaternions** learning tutorial
  * **Ceres Solver** learning tutorial
  * **Kalman Filter** learning tutorial
  * **VINS Mono(Project)** learning tutorial(VIO)
  * **ORB SLAM(Project)** learning tutorial(VIO)
  * **MSCKF-VIO(Project)** learning tutorial(VO)
  * **LSD-SLAM(Project)** learning tutorial(VIO)
  * **Cartographer(Project)** learning tutorial(lidar SLAM)
  * **BALM(Project)** learning tutorial(lidar SLAM)

QQ开源社区
===========

.. image:: ./images/aibot_beginer_qq.png
   :alt: QQ
   :align: center

.. toctree::
   :maxdepth: 2
   :hidden:

   introduction
   installation
   math/math
   kalman_filter/kalman_filter
   tutorial/tutorial
   vslam/vision_slam
   vins/vins
   orb_slam/orb_slam
   msckf/msckf
   lsd_slam/lsd_slam
   cartographer/cartographer
   balm/balm
   learning _resources/learning _resources
   faqs
   license
  

SLAM Research  
=============

**1.开源代码**

1.1 Geometric SLAM
-----------------------

1. PTAM 

   + **论文**：Klein G, Murray D. [**Parallel tracking and mapping for small AR workspaces**](http://www.robots.ox.ac.uk/ActiveVision/Publications/klein_murray_ismar2007/klein_murray_ismar2007.pdf)[C]//Mixed and Augmented Reality, 2007. ISMAR 2007. 6th IEEE and ACM International Symposium on. IEEE, **2007**: 225-234.
   + **代码**：https://github.com/Oxford-PTAM/PTAM-GPL
   + 工程地址：http://www.robots.ox.ac.uk/~gk/PTAM/
   + 作者其他研究：http://www.robots.ox.ac.uk/~gk/publications.html

2. S-PTAM（双目 PTAM）

   + **论文**：Taihú Pire,Thomas Fischer, Gastón Castro, Pablo De Cristóforis, Javier Civera and Julio Jacobo Berlles. [**S-PTAM: Stereo Parallel Tracking and Mapping**](http://webdiis.unizar.es/~jcivera/papers/pire_etal_ras17.pdf). Robotics and Autonomous Systems, **2017**.
   + **代码**：https://github.com/lrse/sptam
   + 作者其他论文：Castro G, Nitsche M A, Pire T, et al. [**Efficient on-board Stereo SLAM through constrained-covisibility strategies**](https://www.researchgate.net/profile/Gaston_Castro/publication/332147108_Efficient_on-board_Stereo_SLAM_through_constrained-covisibility_strategies/links/5cacb327a6fdccfa0e7c3e4b/Efficient-on-board-Stereo-SLAM-through-constrained-covisibility-strategies.pdf)[J]. Robotics and Autonomous Systems, **2019**.

3. MonoSLAM

   + **论文**：Davison A J, Reid I D, Molton N D, et al. [**MonoSLAM: Real-time single camera SLAM**](https://ieeexplore.ieee.org/abstract/document/4160954/)[J]. IEEE transactions on pattern analysis and machine intelligence, **2007**, 29(6): 1052-1067.
   + **代码**：https://github.com/hanmekim/SceneLib2

4. ORB-SLAM2

   + **论文**：Mur-Artal R, Tardós J D. [**Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras**](https://github.com/raulmur/ORB_SLAM2)[J]. IEEE Transactions on Robotics, **2017**, 33(5): 1255-1262.
   + **代码**：https://github.com/raulmur/ORB_SLAM2
   + 作者其他论文：
      + **单目半稠密建图**：Mur-Artal R, Tardós J D. [Probabilistic Semi-Dense Mapping from Highly Accurate Feature-Based Monocular SLAM](https://www.researchgate.net/profile/Raul_Mur-Artal/publication/282807894_Probabilistic_Semi-Dense_Mapping_from_Highly_Accurate_Feature-Based_Monocular_SLAM/links/561cd04308ae6d17308ce267.pdf)[C]//Robotics: Science and Systems. **2015**, 2015.
      + **VIORB**：Mur-Artal R, Tardós J D. [Visual-inertial monocular SLAM with map reuse](https://arxiv.org/pdf/1610.05949.pdf)[J]. IEEE Robotics and Automation Letters, **2017**, 2(2): 796-803.
      + **多地图**：Elvira R, Tardós J D, Montiel J M M. [ORBSLAM-Atlas: a robust and accurate multi-map system](https://arxiv.org/pdf/1908.11585)[J]. arXiv preprint arXiv:1908.11585, **2019**.

5. DSO

   + **论文**：Engel J, Koltun V, Cremers D. [**Direct sparse odometry**](https://ieeexplore.ieee.org/iel7/34/4359286/07898369.pdf)[J]. IEEE transactions on pattern analysis and machine intelligence, **2017**, 40(3): 611-625.
   + **代码**：https://github.com/JakobEngel/dso
   + **双目 DSO**：Wang R, Schworer M, Cremers D. [**Stereo DSO: Large-scale direct sparse visual odometry with stereo cameras**](http://openaccess.thecvf.com/content_ICCV_2017/papers/Wang_Stereo_DSO_Large-Scale_ICCV_2017_paper.pdf)[C]//Proceedings of the IEEE International Conference on Computer Vision. **2017**: 3903-3911.
   + **VI-DSO**：Von Stumberg L, Usenko V, Cremers D. [**Direct sparse visual-inertial odometry using dynamic marginalization**](https://arxiv.org/pdf/1804.05625)[C]//2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2018**: 2510-2517.

6. LDSO

   + 高翔在 DSO 上添加闭环的工作
   + **论文**：Gao X, Wang R, Demmel N, et al. [**LDSO: Direct sparse odometry with loop closure**](https://arxiv.org/pdf/1808.01111)[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2018**: 2198-2204.
   + **代码**：https://github.com/tum-vision/LDSO

7. LSD-SLAM

   + **论文**：Engel J, Schöps T, Cremers D. [**LSD-SLAM: Large-scale direct monocular SLAM**](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.646.7193&rep=rep1&type=pdf)[C]//European conference on computer vision. Springer, Cham, **2014**: 834-849.
   + **代码**：https://github.com/tum-vision/lsd_slam

8. DVO-SLAM

   + **论文**：Kerl C, Sturm J, Cremers D. [**Dense visual SLAM for RGB-D cameras**]()[C]//2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, **2013**: 2100-2106.
   + **代码 1**：https://github.com/tum-vision/dvo_slam
   + **代码 2**：https://github.com/tum-vision/dvo
   + 其他论文：
      + Kerl C, Sturm J, Cremers D. [**Robust odometry estimation for RGB-D cameras**](https://vision.in.tum.de/_media/spezial/bib/kerl13icra.pdf)[C]//2013 IEEE international conference on robotics and automation. IEEE, **2013**: 3748-3754.
      + Steinbrücker F, Sturm J, Cremers D. [**Real-time visual odometry from dense RGB-D images**](https://jsturm.de/publications/data/steinbruecker_sturm_cremers_iccv11.pdf)[C]//2011 IEEE international conference on computer vision workshops (ICCV Workshops). IEEE, **2011**: 719-722.

9. SVO

   + [苏黎世大学机器人与感知课题组](http://rpg.ifi.uzh.ch/publications.html)
   + **论文**：Forster C, Pizzoli M, Scaramuzza D. [**SVO: Fast semi-direct monocular visual odometry**](https://www.zora.uzh.ch/id/eprint/125453/1/ICRA14_Forster.pdf)[C]//2014 IEEE international conference on robotics and automation (ICRA). IEEE, **2014**: 15-22.
   + **代码**：https://github.com/uzh-rpg/rpg_svo
   + Forster C, Zhang Z, Gassner M, et al. [**SVO: Semidirect visual odometry for monocular and multicamera systems**](https://www.zora.uzh.ch/id/eprint/127902/1/TRO16_Forster-SVO.pdf)[J]. IEEE Transactions on Robotics, **2016**, 33(2): 249-265.

10. DSM

   + **论文**：Zubizarreta J, Aguinaga I, Montiel J M M. [**Direct sparse mapping**](https://arxiv.org/pdf/1904.06577)[J]. arXiv preprint arXiv:1904.06577, **2019**.
   + **代码**：https://github.com/jzubizarreta/dsm ；[Video](https://www.youtube.com/watch?v=sj1GIF-7BYo&feature=youtu.be)

11. openvslam

   + 论文：Sumikura S, Shibuya M, Sakurada K. [**OpenVSLAM: A Versatile Visual SLAM Framework**](https://dl.acm.org/ft_gateway.cfm?id=3350539&type=pdf)[C]//Proceedings of the 27th ACM International Conference on Multimedia. **2019**: 2292-2295.
   + 代码：https://github.com/xdspacelab/openvslam ；[文档](https://openvslam.readthedocs.io/en/master/)

12. se2lam（地面车辆位姿估计的视觉里程计）

   + **论文**：Zheng F, Liu Y H. [**Visual-Odometric Localization and Mapping for Ground Vehicles Using SE (2)-XYZ Constraints**](https://ieeexplore.ieee.org/abstract/document/8793928)[C]//2019 International Conference on Robotics and Automation (**ICRA**). IEEE, **2019**: 3556-3562.
   + **代码**：https://github.com/izhengfan/se2lam
   + 作者的另外一项工作
      + 论文：Zheng F, Tang H, Liu Y H. [**Odometry-vision-based ground vehicle motion estimation with se (2)-constrained se (3) poses**](https://ieeexplore.ieee.org/abstract/document/8357438/)[J]. IEEE transactions on cybernetics, **2018**, 49(7): 2652-2663.
      + 代码：https://github.com/izhengfan/se2clam

13. GraphSfM（基于图的并行大规模 SFM）

   + 论文：Chen Y, Shen S, Chen Y, et al. [**Graph-Based Parallel Large Scale Structure from Motion**](https://arxiv.org/pdf/1912.10659.pdf)[J]. arXiv preprint arXiv:1912.10659, **2019**.
   + 代码：https://github.com/AIBluefisher/GraphSfM

14. LCSD_SLAM（松耦合的半直接法单目 SLAM）

   + **论文**：Lee S H, Civera J. [**Loosely-Coupled semi-direct monocular SLAM**](https://arxiv.org/pdf/1807.10073)[J]. IEEE Robotics and Automation Letters, **2018**, 4(2): 399-406.
   + **代码**：https://github.com/sunghoon031/LCSD_SLAM ；[谷歌学术](https://scholar.google.com/citations?user=FeMFP7EAAAAJ&hl=zh-CN&oi=sra) ；[演示视频](https://www.youtube.com/watch?v=j7WnU7ZpZ8c&feature=youtu.be)
   + 作者另外一篇关于**单目尺度**的文章 [代码开源](https://github.com/sunghoon031/stability_scale) ：Lee S H, de Croon G. [**Stability-based scale estimation for monocular SLAM**](https://www.researchgate.net/profile/Seong_Hun_Lee3/publication/322260802_Stability-based_Scale_Estimation_for_Monocular_SLAM/links/5b3def9b0f7e9b0df5f42d67/Stability-based-Scale-Estimation-for-Monocular-SLAM.pdf)[J]. IEEE Robotics and Automation Letters, **2018**, 3(2): 780-787.

15. RESLAM（基于边的 SLAM）

   + **论文**：Schenk F, Fraundorfer F. [**RESLAM: A real-time robust edge-based SLAM system**](https://ieeexplore.ieee.org/abstract/document/8794462/)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 154-160.
   + **代码**：https://github.com/fabianschenk/RESLAM ； [项目主页](https://graz.pure.elsevier.com/en/publications/reslam-a-real-time-robust-edge-based-slam-system)

16. scale_optimization（将单目 DSO 拓展到双目）

   + **论文**：Mo J, Sattar J. [**Extending Monocular Visual Odometry to Stereo Camera System by Scale Optimization**](https://arxiv.org/pdf/1905.12723.pdf)[C]. International Conference on Intelligent Robots and Systems (**IROS**), **2019**.
   + **代码**：https://github.com/jiawei-mo/scale_optimization

17. BAD-SLAM（直接法 RGB-D SLAM）

   + **论文**：Schops T, Sattler T, Pollefeys M. [**BAD SLAM: Bundle Adjusted Direct RGB-D SLAM**](http://openaccess.thecvf.com/content_CVPR_2019/papers/Schops_BAD_SLAM_Bundle_Adjusted_Direct_RGB-D_SLAM_CVPR_2019_paper.pdf)[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. **2019**: 134-144.
   + **代码**：https://github.com/ETH3D/badslam

18. GSLAM（集成 ORB-SLAM2，DSO，SVO 的通用框架）

   + **论文**：Zhao Y, Xu S, Bu S, et al. [**GSLAM: A general SLAM framework and benchmark**](http://openaccess.thecvf.com/content_ICCV_2019/papers/Zhao_GSLAM_A_General_SLAM_Framework_and_Benchmark_ICCV_2019_paper.pdf)[C]//Proceedings of the IEEE International Conference on Computer Vision. **2019**: 1110-1120.
   + **代码**：https://github.com/zdzhaoyong/GSLAM

19. ARM-VO（运行于 ARM 处理器上的单目 VO）

   + **论文**：Nejad Z Z, Ahmadabadian A H. [**ARM-VO: an efficient monocular visual odometry for ground vehicles on ARM CPUs**](https://link.springer.com/article/10.1007/s00138-019-01037-5)[J]. Machine Vision and Applications, **2019**: 1-10.
   + **代码**：https://github.com/zanazakaryaie/ARM-VO

20. cvo-rgbd（直接法 RGB-D VO）

   + **论文**：Ghaffari M, Clark W, Bloch A, et al. [**Continuous Direct Sparse Visual Odometry from RGB-D Images**](https://arxiv.org/pdf/1904.02266.pdf)[J]. arXiv preprint arXiv:1904.02266, **2019**.
   + **代码**：https://github.com/MaaniGhaffari/cvo-rgbd

21. Map2DFusion（单目 SLAM 无人机图像拼接）

   + **论文**：Bu S, Zhao Y, Wan G, et al. [**Map2DFusion: Real-time incremental UAV image mosaicing based on monocular slam**](http://www.adv-ci.com/publications/2016_IROS.pdf)[C]//2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2016**: 4564-4571.
   + **代码**：https://github.com/zdzhaoyong/Map2DFusion

22. CCM-SLAM（多机器人协同单目 SLAM）

   + **论文**：Schmuck P, Chli M. [**CCM‐SLAM: Robust and efficient centralized collaborative monocular simultaneous localization and mapping for robotic teams**](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21854)[J]. Journal of Field Robotics, **2019**, 36(4): 763-781.
   + **代码**：https://github.com/VIS4ROB-lab/ccm_slam &emsp; [Video](https://www.youtube.com/watch?v=P3b7UiTlmbQ&feature=youtu.be)

23. ORB-SLAM3

   + 论文：Carlos Campos, Richard Elvira, et al.[**ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**](https://arxiv.org/abs/2007.11898)[J]. arXiv preprint arXiv:2007.11898, **2020**.
   + 代码：https://github.com/UZ-SLAMLab/ORB_SLAM3 | [Video](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q)

24. OV²SLAM（完全实时在线多功能 SLAM）

   + 论文：Ferrera M, Eudes A, Moras J, et al. [**OV $^{2} $ SLAM: A Fully Online and Versatile Visual SLAM for Real-Time Applications**](https://ieeexplore.ieee.org/abstract/document/9351614/)[J]. IEEE Robotics and Automation Letters, **2021**, 6(2): 1399-1406.
   + 代码：https://github.com/ov2slam/ov2slam

25. ESVO（基于事件的双目视觉里程计）

   + 论文：Zhou Y, Gallego G, Shen S. [**Event-based stereo visual odometry**](https://ieeexplore.ieee.org/abstract/document/9386209/)[J]. IEEE Transactions on Robotics, **2021**.
   + 代码：https://github.com/HKUST-Aerial-Robotics/ESVO

26. VOLDOR-SLAM（实时稠密非直接法 SLAM）

   + 论文：Min Z, Dunn E. [**VOLDOR-SLAM: For the Times When Feature-Based or Direct Methods Are Not Good Enough**](https://arxiv.org/abs/2104.06800)[J]. arXiv preprint arXiv:2104.06800, 2021.
   + 代码：https://github.com/htkseason/VOLDOR

1.2 Semantic / Deep SLAM
--------------------------

1. MsakFusion

   + **论文**：Runz M, Buffier M, Agapito L. [**Maskfusion: Real-time recognition, tracking and reconstruction of multiple moving objects**](https://arxiv.org/pdf/1804.09194)[C]//2018 IEEE International Symposium on Mixed and Augmented Reality (ISMAR). IEEE, **2018**: 10-20.
   + **代码**：https://github.com/martinruenz/maskfusion

2. SemanticFusion

   + **论文**：McCormac J, Handa A, Davison A, et al. [**Semanticfusion: Dense 3d semantic mapping with convolutional neural networks**](https://arxiv.org/pdf/1609.05130.pdf)[C]//2017 IEEE International Conference on Robotics and automation (ICRA). IEEE, **2017**: 4628-4635.
   + **代码**：https://github.com/seaun163/semanticfusion

3. semantic_3d_mapping

   + **论文**：Yang S, Huang Y, Scherer S. [**Semantic 3D occupancy mapping through efficient high order CRFs**](https://arxiv.org/pdf/1707.07388.pdf)[C]//2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2017**: 590-597.
   + **代码**：https://github.com/shichaoy/semantic_3d_mapping

4. Kimera（实时度量与语义定位建图开源库）

   + 论文：Rosinol A, Abate M, Chang Y, et al. [**Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping**](https://arxiv.org/pdf/1910.02490.pdf)[J]. arXiv preprint arXiv:1910.02490, **2019**.
   + 代码：https://github.com/MIT-SPARK/Kimera ；[演示视频](https://www.youtube.com/watch?v=3lVD0i-5p10)

5. NeuroSLAM（脑启发式 SLAM）

   + **论文**：Yu F, Shang J, Hu Y, et al. [**NeuroSLAM: a brain-inspired SLAM system for 3D environments**](https://www.researchgate.net/profile/Fangwen_Yu/publication/336164484_NeuroSLAM_a_brain-inspired_SLAM_system_for_3D_environments/links/5d95f38d458515c1d3908a20/NeuroSLAM-a-brain-inspired-SLAM-system-for-3D-environments.pdf)[J]. Biological Cybernetics, **2019**: 1-31.
   + **代码**：https://github.com/cognav/NeuroSLAM
   + 第四作者就是 Rat SLAM 的作者，文章也比较了十余种脑启发式的 SLAM

6. gradSLAM（自动分区的稠密 SLAM）

   + **论文**：Jatavallabhula K M, Iyer G, Paull L. [**gradSLAM: Dense SLAM meets Automatic Differentiation**](https://arxiv.org/pdf/1910.10672.pdf)[J]. arXiv preprint arXiv:1910.10672, **2019**.
   + **代码**（预计 20 年 4 月放出）：https://github.com/montrealrobotics/gradSLAM ；[项目主页](http://montrealrobotics.ca/gradSLAM/)，[演示视频](https://www.youtube.com/watch?v=2ygtSJTmo08&feature=youtu.be)

7. ORB-SLAM2 + 目标检测/分割的方案语义建图

   + https://github.com/floatlazer/semantic_slam
   + https://github.com/qixuxiang/orb-slam2_with_semantic_labelling
   + https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic

8. SIVO（语义辅助特征选择）

   + **论文**：Ganti P, Waslander S. [**Network Uncertainty Informed Semantic Feature Selection for Visual SLAM**](https://ieeexplore.ieee.org/abstract/document/8781616)[C]//2019 16th Conference on Computer and Robot Vision (CRV). IEEE, **2019**: 121-128.
   + **代码**：https://github.com/navganti/SIVO

9. FILD（临近图增量式闭环检测）

   + **论文**：Shan An, Guangfu Che, Fangru Zhou, Xianglong Liu, Xin Ma, Yu Chen.[ **Fast and Incremental Loop Closure Detection using Proximity Graphs**](https://arxiv.org/pdf/1911.10752.pdf). pp. 378-385, The 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS **2019**)
   + **代码**：https://github.com/AnshanTJU/FILD

10. object-detection-sptam（目标检测与双目 SLAM）

   + **论文**：Pire T, Corti J, Grinblat G. [**Online Object Detection and Localization on Stereo Visual SLAM System**](https://www.researchgate.net/profile/Taihu_Pire/publication/335432416_Online_Object_Detection_and_Localization_on_Stereo_Visual_SLAM_System/links/5d663a14a6fdccf343f93830/Online-Object-Detection-and-Localization-on-Stereo-Visual-SLAM-System.pdf)[J]. Journal of Intelligent & Robotic Systems, **2019**: 1-10.
   + **代码**：https://github.com/CIFASIS/object-detection-sptam

11. Map Slammer（单目深度估计 + SLAM）

   + **论文**：Torres-Camara J M, Escalona F, Gomez-Donoso F, et al. [**Map Slammer: Densifying Scattered KSLAM 3D Maps with Estimated Depth**](https://link.springer.com/chapter/10.1007/978-3-030-36150-1_46)[C]//Iberian Robotics conference. Springer, Cham, **2019**: 563-574.
   + **代码**：https://github.com/jmtc7/mapSlammer

12. NOLBO（变分模型的概率 SLAM）

   + **论文**：Yu H, Lee B. [**Not Only Look But Observe: Variational Observation Model of Scene-Level 3D Multi-Object Understanding for Probabilistic SLAM**](https://arxiv.org/pdf/1907.09760.pdf)[J]. arXiv preprint arXiv:1907.09760, **2019**.
   + **代码**：https://github.com/bogus2000/NOLBO

13. GCNv2_SLAM （基于图卷积神经网络 SLAM）

   + **论文**：Tang J, Ericson L, Folkesson J, et al. [**GCNv2: Efficient correspondence prediction for real-time SLAM**](https://ieeexplore.ieee.org/abstract/document/8758836/)[J]. IEEE Robotics and Automation Letters, **2019**, 4(4): 3505-3512.
   + **代码**：https://github.com/jiexiong2016/GCNv2_SLAM &emsp; [Video](https://www.youtube.com/watch?v=pz-gdnR9tAM)

14. semantic_suma（激光语义建图）

   + **论文**：Chen X, Milioto A, Palazzolo E, et al. [**SuMa++: Efficient LiDAR-based semantic SLAM**](https://ieeexplore.ieee.org/abstract/document/8967704/)[C]//2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2019**: 4530-4537.
   + **代码**：https://github.com/PRBonn/semantic_suma/ ；[Video](https://www.youtube.com/watch?v=uo3ZuLuFAzk&feature=youtu.be)

15. Neural-SLAM（主动神经 SLAM）

   + **论文**：Chaplot D S, Gandhi D, Gupta S, et al. [**Learning to explore using active neural slam**](https://arxiv.org/abs/2004.05155)[C]. ICLR **2020**.
   + **代码**：https://github.com/devendrachaplot/Neural-SLAM

16. TartanVO：一种通用的基于学习的 VO

   + **论文**：Wang W, Hu Y, Scherer S. [**TartanVO: A Generalizable Learning-based VO**](https://arxiv.org/abs/2011.00359)[J]. arXiv preprint arXiv:2011.00359, **2020**.
   + **代码**：https://github.com/castacks/tartanvo
   + 数据集：IROS2020 [**TartanAir: A Dataset to Push the Limits of Visual SLAM**](https://arxiv.org/abs/2003.14338)，[数据集地址](http://theairlab.org/tartanair-dataset/)

17. DF-VO

   + **论文**：Zhan H, Weerasekera C S, Bian J W, et al. [**DF-VO: What Should Be Learnt for Visual Odometry?**](https://arxiv.org/abs/2103.00933)[J]. arXiv preprint arXiv:2103.00933, **2021**. 
   + Zhan H, Weerasekera C S, Bian J W, et al. [**Visual odometry revisited: What should be learnt?**](https://ieeexplore.ieee.org/abstract/document/9197374/)[C]//2020 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2020**: 4203-4210.
   + **代码**：https://github.com/Huangying-Zhan/DF-VO

1.3 Multi-Landmarks / Object SLAM
------------------------------------

1. PL-SVO（点线 SVO）

   + **论文**：Gomez-Ojeda R, Briales J, Gonzalez-Jimenez J. [**PL-SVO: Semi-direct Monocular Visual Odometry by combining points and line segments**](http://mapir.isa.uma.es/rgomez/publications/iros16plsvo.pdf)[C]//Intelligent Robots and Systems (IROS), 2016 IEEE/RSJ International Conference on. IEEE, **2016**: 4211-4216.
   + **代码**：https://github.com/rubengooj/pl-svo

2. stvo-pl（双目点线 VO）

   + **论文**：Gomez-Ojeda R, Gonzalez-Jimenez J. [**Robust stereo visual odometry through a probabilistic combination of points and line segments**](https://riuma.uma.es/xmlui/bitstream/handle/10630/11515/icra16rgo.pdf?sequence=1&isAllowed=y)[C]//2016 IEEE International Conference on Robotics and Automation (**ICRA**). IEEE, **2016**: 2521-2526.
   + **代码**：https://github.com/rubengooj/stvo-pl

3. PL-SLAM（点线 SLAM）

   + **论文**：Gomez-Ojeda R, Zuñiga-Noël D, Moreno F A, et al. [**PL-SLAM: a Stereo SLAM System through the Combination of Points and Line Segments**](https://arxiv.org/pdf/1705.09479.pdf)[J]. arXiv preprint arXiv:1705.09479, **2017**.
   + **代码**：https://github.com/rubengooj/pl-slam
   + Gomez-Ojeda R, Moreno F A, Zuñiga-Noël D, et al. [**PL-SLAM: a stereo SLAM system through the combination of points and line segments**](https://arxiv.org/pdf/1705.09479)[J]. IEEE Transactions on Robotics, **2019**, 35(3): 734-746.

4. PL-VIO

   + **论文**：He Y, Zhao J, Guo Y, et al. [**PL-VIO: Tightly-coupled monocular visual–inertial odometry using point and line features**](https://www.mdpi.com/1424-8220/18/4/1159)[J]. Sensors, **2018**, 18(4): 1159.
   + **代码**：https://github.com/HeYijia/PL-VIO
   + **VINS + 线段**：https://github.com/Jichao-Peng/VINS-Mono-Optimization

5. lld-slam（用于 SLAM 的可学习型线段描述符）

   + **论文**：Vakhitov A, Lempitsky V. [**Learnable line segment descriptor for visual SLAM**](https://ieeexplore.ieee.org/iel7/6287639/6514899/08651490.pdf)[J]. IEEE Access, **2019**, 7: 39923-39934.
   + **代码**：https://github.com/alexandervakhitov/lld-slam ；[**Video**](https://www.youtube.com/watch?v=ntFFiwXIhoA)

   点线结合的工作还有很多，国内的比如        

   + 上交邹丹平老师的 Zou D, Wu Y, Pei L, et al. [**StructVIO: visual-inertial odometry with structural regularity of man-made environments**](https://arxiv.org/pdf/1810.06796)[J]. IEEE Transactions on Robotics, **2019**, 35(4): 999-1013.         
   + 浙大的 Zuo X, Xie X, Liu Y, et al. [**Robust visual SLAM with point and line features**](https://arxiv.org/pdf/1711.08654)[C]//2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017: 1775-1782.</small>

6. PlaneSLAM

   + **论文**：Wietrzykowski J. [**On the representation of planes for efficient graph-based slam with high-level features**](https://yadda.icm.edu.pl/baztech/element/bwmeta1.element.baztech-7ac7a8f3-9caa-4a34-8a27-8f6c5f43408b)[J]. Journal of Automation Mobile Robotics and Intelligent Systems, **2016**, 10.
   + **代码**：https://github.com/LRMPUT/PlaneSLAM
   + 作者另外一项开源代码，没有找到对应的论文：https://github.com/LRMPUT/PUTSLAM

7. Eigen-Factors（特征因子平面对齐）

   + **论文**：Ferrer G. [**Eigen-Factors: Plane Estimation for Multi-Frame and Time-Continuous Point Cloud Alignment**](http://sites.skoltech.ru/app/data/uploads/sites/50/2019/07/ferrer2019planes.pdf)[C]//2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2019**: 1278-1284.
   + **代码**：https://gitlab.com/gferrer/eigen-factors-iros2019 ；[演示视频](https://www.youtube.com/watch?v=_1u_c43DFUE&feature=youtu.be)

8. PlaneLoc

   + **论文**：Wietrzykowski J, Skrzypczyński P. [**PlaneLoc: Probabilistic global localization in 3-D using local planar features**]()[J]. Robotics and Autonomous Systems, **2019**, 113: 160-173.
   + **代码**：https://github.com/LRMPUT/PlaneLoc

9. Pop-up SLAM

   + **论文**：Yang S, Song Y, Kaess M, et al. [**Pop-up slam: Semantic monocular plane slam for low-texture environments**](https://arxiv.org/pdf/1703.07334.pdf)[C]//2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2016**: 1222-1229.
   + **代码**：https://github.com/shichaoy/pop_up_slam

10. Object SLAM

   + **论文**：Mu B, Liu S Y, Paull L, et al. [**Slam with objects using a nonparametric pose graph**](https://arxiv.org/pdf/1704.05959.pdf)[C]//2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2016**: 4602-4609.
   + **代码**：https://github.com/BeipengMu/objectSLAM ；[Video](https://www.youtube.com/watch?v=YANUWdVLJD4&feature=youtu.be)

11. voxblox-plusplus（物体级体素建图）

   + **论文**：Grinvald M, Furrer F, Novkovic T, et al. [**Volumetric instance-aware semantic mapping and 3D object discovery**](https://arxiv.org/pdf/1903.00268.pdf)[J]. IEEE Robotics and Automation Letters, **2019**, 4(3): 3037-3044.
   + **代码**：https://github.com/ethz-asl/voxblox-plusplus

12. Cube SLAM

   + **论文**：Yang S, Scherer S. [**Cubeslam: Monocular 3-d object slam**](https://arxiv.org/pdf/1806.00557)[J]. IEEE Transactions on Robotics, **2019**, 35(4): 925-938.
   + **代码**：https://github.com/shichaoy/cube_slam
   + 对，这就是带我入坑的一项工作，2018 年 11 月份看到这篇论文（当时是预印版）之后开始学习物体级 SLAM，个人对 Cube SLAM 的一些注释和总结：[链接](https://wym.netlify.app/categories/cube-slam/)。
   + 也有很多有意思的但没开源的物体级 SLAM
      + Ok K, Liu K, Frey K, et al. [**Robust Object-based SLAM for High-speed Autonomous Navigation**](http://groups.csail.mit.edu/rrg/papers/OkLiu19icra.pdf)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 669-675.
      + Li J, Meger D, Dudek G. [**Semantic Mapping for View-Invariant Relocalization**](https://www.cim.mcgill.ca/~mrl/pubs/jimmy/li2019icra.pdf)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 7108-7115.
      + Nicholson L, Milford M, Sünderhauf N. [**Quadricslam: Dual quadrics from object detections as landmarks in object-oriented slam**](https://arxiv.org/pdf/1804.04011)[J]. IEEE Robotics and Automation Letters, **2018**, 4(1): 1-8.

13. VPS-SLAM（平面语义 SLAM）

   + **论文**：Bavle H, De La Puente P, How J, et al. [**VPS-SLAM: Visual Planar Semantic SLAM for Aerial Robotic Systems**](https://ieeexplore.ieee.org/iel7/6287639/8948470/09045978.pdf)[J]. IEEE Access, **2020**.
   + **代码**：https://bitbucket.org/hridaybavle/semantic_slam/src/master/

14. Structure-SLAM （低纹理环境下点线 SLAM）

   + **论文**：Li Y, Brasch N, Wang Y, et al. [**Structure-SLAM: Low-Drift Monocular SLAM in Indoor Environments**](https://ieeexplore.ieee.org/abstract/document/9165014)[J]. IEEE Robotics and Automation Letters, **2020**, 5(4): 6583-6590.
   + **代码**：https://github.com/yanyan-li/Structure-SLAM-PointLine

15. PL-VINS

   + **论文**：Fu Q, Wang J, Yu H, et al. [**PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and Line**](https://arxiv.org/abs/2009.07462)[J]. arXiv preprint arXiv:2009.07462, **2020**.
   + **代码**：https://github.com/cnqiangfu/PL-VINS

1.4 Sensor Fusion
---------------------

1. msckf_vio

   + **论文**：Sun K, Mohta K, Pfrommer B, et al. [**Robust stereo visual inertial odometry for fast autonomous flight**](https://arxiv.org/pdf/1712.00036)[J]. IEEE Robotics and Automation Letters, **2018**, 3(2): 965-972.
   + **代码**：https://github.com/KumarRobotics/msckf_vio ；[Video](https://www.youtube.com/watch?v=jxfJFgzmNSw&t)

2. rovio

   + **论文**：Bloesch M, Omari S, Hutter M, et al. [**Robust visual inertial odometry using a direct EKF-based approach**](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/155340/1/eth-48374-01.pdf)[C]//2015 IEEE/RSJ international conference on intelligent robots and systems (IROS). IEEE, **2015**: 298-304.
   + **代码**：https://github.com/ethz-asl/rovio ；[Video](https://www.youtube.com/watch?v=ZMAISVy-6ao&feature=youtu.be)

3. R-VIO

   + **论文**：Huai Z, Huang G. [**Robocentric visual-inertial odometry**](https://arxiv.org/pdf/1805.04031)[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2018**: 6319-6326.
   + **代码**：https://github.com/rpng/R-VIO ；[Video](https://www.youtube.com/watch?v=l9IC2ddBEYQ)
   + **VI_ORB_SLAM2**：https://github.com/YoujieXia/VI_ORB_SLAM2

4. okvis

   + **论文**：Leutenegger S, Lynen S, Bosse M, et al. [**Keyframe-based visual–inertial odometry using nonlinear optimization**](https://spiral.imperial.ac.uk/bitstream/10044/1/23413/2/ijrr2014_revision_1.pdf)[J]. The International Journal of Robotics Research, **2015**, 34(3): 314-334.
   + **代码**：https://github.com/ethz-asl/okvis

5. VIORB

   + **论文**：Mur-Artal R, Tardós J D. [Visual-inertial monocular SLAM with map reuse](https://arxiv.org/pdf/1610.05949.pdf)[J]. IEEE Robotics and Automation Letters, **2017**, 2(2): 796-803.
   + **代码**：https://github.com/jingpang/LearnVIORB （VIORB 本身是没有开源的，这是王京大佬复现的一个版本）

6. VINS-mono

   + **论文**：Qin T, Li P, Shen S. [**Vins-mono: A robust and versatile monocular visual-inertial state estimator**](https://arxiv.org/pdf/1708.03852)[J]. IEEE Transactions on Robotics, **2018**, 34(4): 1004-1020.
   + **代码**：https://github.com/HKUST-Aerial-Robotics/VINS-Mono
   + 双目版 **VINS-Fusion**：https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
   + 移动段 **VINS-mobile**：https://github.com/HKUST-Aerial-Robotics/VINS-Mobile

7. VINS-RGBD

   + **论文**：Shan Z, Li R, Schwertfeger S. [**RGBD-Inertial Trajectory Estimation and Mapping for Ground Robots**](https://www.mdpi.com/1424-8220/19/10/2251)[J]. Sensors, **2019**, 19(10): 2251.
   + **代码**：https://github.com/STAR-Center/VINS-RGBD ；[**Video**](https://robotics.shanghaitech.edu.cn/datasets/VINS-RGBD)

8. Open-VINS

   + **论文**：Geneva P, Eckenhoff K, Lee W, et al. [**Openvins: A research platform for visual-inertial estimation**](https://pdfs.semanticscholar.org/cb63/60f21255834297e32826bff6366a769b49e9.pdf)[C]//IROS 2019 Workshop on Visual-Inertial Navigation: Challenges and Applications, Macau, China. **IROS 2019**.
   + **代码**：https://github.com/rpng/open_vins

9. versavis（多功能的视惯传感器系统）

   + 论文：Tschopp F, Riner M, Fehr M, et al. [**VersaVIS—An Open Versatile Multi-Camera Visual-Inertial Sensor Suite**](https://www.mdpi.com/1424-8220/20/5/1439)[J]. Sensors, **2020**, 20(5): 1439.
   + 代码：https://github.com/ethz-asl/versavis

10. CPI（视惯融合的封闭式预积分）

   + **论文**：Eckenhoff K, Geneva P, Huang G. [**Closed-form preintegration methods for graph-based visual–inertial navigation**](http://sage.cnpereading.com/paragraph/article/10.1177/0278364919835021)[J]. The International Journal of Robotics Research, 2018.
   + **代码**：https://github.com/rpng/cpi ；[**Video**](https://www.youtube.com/watch?v=yIgQX2SH_pI)

11. TUM Basalt

   + **论文**：Usenko V, Demmel N, Schubert D, et al. [Visual-inertial mapping with non-linear factor recovery](https://link.zhihu.com/?target=https%3A//arxiv.org/pdf/1904.06504.pdf)[J]. IEEE Robotics and Automation Letters, **2019**.
   + **代码**：[https://github.com/VladyslavUsenko/basalt-mirror](https://link.zhihu.com/?target=https%3A//github.com/VladyslavUsenko/basalt-mirror) ；[Video](https://link.zhihu.com/?target=https%3A//www.youtube.com/watch%3Fv%3Dr3CJ2JP75Tc)；[Project Page](https://link.zhihu.com/?target=https%3A//vision.in.tum.de/research/vslam/basalt)

12. Limo（激光单目视觉里程计）

   + **论文**：Graeter J, Wilczynski A, Lauer M. [**Limo: Lidar-monocular visual odometry**](https://arxiv.org/pdf/1807.07524)[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2018**: 7872-7879.
   + **代码**：https://github.com/johannes-graeter/limo ； [**Video**](https://www.youtube.com/watch?v=wRemjJBjp64&feature=youtu.be)

13. LARVIO（多状态约束卡尔曼滤波的单目 VIO）

   + **论文**：Qiu X, Zhang H, Fu W, et al. [Monocular Visual-Inertial Odometry with an Unbiased Linear System Model and Robust Feature Tracking Front-End](https://www.mdpi.com/1424-8220/19/8/1941)[J]. Sensors, **2019**, 19(8): 1941.
   + **代码**：https://github.com/PetWorm/LARVIO
   + 北航邱笑晨博士的一项工作

14. vig-init（垂直边缘加速视惯初始化）

   + **论文**：Li J, Bao H, Zhang G. [**Rapid and Robust Monocular Visual-Inertial Initialization with Gravity Estimation via Vertical Edges**](http://www.cad.zju.edu.cn/home/gfzhang/projects/iros2019-vi-initialization.pdf)[C]//2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2019**: 6230-6236.
   + **代码**：https://github.com/zju3dv/vig-init
   + 浙大章国峰老师组的一项工作

15. vilib（VIO 前端库）

   + **论文**：Nagy B, Foehn P, Scaramuzza D. [**Faster than FAST: GPU-Accelerated Frontend for High-Speed VIO**](https://arxiv.org/pdf/2003.13493)[J]. arXiv preprint arXiv:2003.13493, **2020**.
   + **代码**：https://github.com/uzh-rpg/vilib

16. Kimera-VIO

   + **论文**：A. Rosinol, M. Abate, Y. Chang, L. Carlone, [**Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping**](https://arxiv.org/abs/1910.02490). IEEE Intl. Conf. on Robotics and Automation (ICRA), 2020.
   + **代码**：https://github.com/MIT-SPARK/Kimera-VIO

17. maplab（视惯建图框架）

   + **论文**：Schneider T, Dymczyk M, Fehr M, et al. [**maplab: An open framework for research in visual-inertial mapping and localization**](https://arxiv.org/pdf/1711.10250)[J]. IEEE Robotics and Automation Letters, **2018**, 3(3): 1418-1425. 
   + **代码**：https://github.com/ethz-asl/maplab
   + 多会话建图，地图合并，视觉惯性批处理优化和闭环

18. lili-om：固态雷达惯性里程计与建图

   + **论文**：Li K, Li M, Hanebeck U D. [**Towards high-performance solid-state-lidar-inertial odometry and mapping**](https://arxiv.org/abs/2010.13150)[J]. arXiv preprint arXiv:2010.13150, **2020**.
   + **代码**：https://github.com/KIT-ISAS/lili-om

19. CamVox：Lidar 辅助视觉 SLAM

   + **论文**：ZHU, Yuewen, et al. [**CamVox: A Low-cost and Accurate Lidar-assisted Visual SLAM System**](https://arxiv.org/abs/2011.11357). arXiv preprint arXiv:2011.11357, **2020**.
   + **代码**：https://github.com/ISEE-Technology/CamVox

20. SSL_SLAM：固态 LiDAR 轻量级 3D 定位与建图

   + **论文**：Wang H, Wang C, Xie L. [**Lightweight 3-D Localization and Mapping for Solid-State LiDAR**](https://ieeexplore.ieee.org/abstract/document/9357899)[J]. IEEE Robotics and Automation Letters, **2021**, 6(2): 1801-1807.
   + **代码**：https://github.com/wh200720041/SSL_SLAM

21. r2live：LiDAR-Inertial-Visual 紧耦合

   + **论文**：Lin J, Zheng C, Xu W, et al. [**R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual tightly-coupled state Estimator and mapping**](https://arxiv.org/abs/2102.12400)[J]. arXiv preprint arXiv:2102.12400, **2021**.
   + **代码**：https://github.com/hku-mars/r2live

22. GVINS：GNSS-视觉-惯导紧耦合

   + **论文**：Cao S, Lu X, Shen S. [**GVINS: Tightly Coupled GNSS-Visual-Inertial for Smooth and Consistent State Estimation**](https://ui.adsabs.harvard.edu/abs/2021arXiv210307899C/abstract)[J]. arXiv e-prints, **2021**: arXiv: 2103.07899.
   + **代码**：https://github.com/HKUST-Aerial-Robotics/GVINS

23. LVI-SAM：Lidar-Visual-Inertial 建图与定位

   + **论文**：Shan T, Englot B, Ratti C, et al. [**LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping**](https://arxiv.org/abs/2104.10831)[J]. arXiv preprint arXiv:2104.10831, **2021**. (ICRA2021)
   + **代码**：https://github.com/TixiaoShan/LVI-SAM

1.5 Dynamic SLAM
---------------------

1. DynamicSemanticMapping（动态语义建图）

   + **论文**：Kochanov D, Ošep A, Stückler J, et al. [**Scene flow propagation for semantic mapping and object discovery in dynamic street scenes**](http://web-info8.informatik.rwth-aachen.de/media/papers/paper_compressed.pdf)[C]//Intelligent Robots and Systems (IROS), 2016 IEEE/RSJ International Conference on. IEEE, **2016**: 1785-1792.
   + **代码**：https://github.com/ganlumomo/DynamicSemanticMapping ；[wiki](https://github.com/ganlumomo/DynamicSemanticMapping/wiki)

2. DS-SLAM（动态语义 SLAM）

   + **论文**：Yu C, Liu Z, Liu X J, et al. [**DS-SLAM: A semantic visual SLAM towards dynamic environments**](https://arxiv.org/pdf/1809.08379)[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2018**: 1168-1174.
   + **代码**：https://github.com/ivipsourcecode/DS-SLAM

3. Co-Fusion（实时分割与跟踪多物体）

   + **论文**：Rünz M, Agapito L. [**Co-fusion: Real-time segmentation, tracking and fusion of multiple objects**](https://ieeexplore.ieee.org/abstract/document/7989518)[C]//2017 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2017: 4471-4478.
   + **代码**：https://github.com/martinruenz/co-fusion ； [Video](http://visual.cs.ucl.ac.uk/pubs/cofusion/index.html)

4. DynamicFusion

   + **论文**：Newcombe R A, Fox D, Seitz S M. [**Dynamicfusion: Reconstruction and tracking of non-rigid scenes in real-time**](https://www.cv-foundation.org/openaccess/content_cvpr_2015/papers/Newcombe_DynamicFusion_Reconstruction_and_2015_CVPR_paper.pdf)[C]//Proceedings of the IEEE conference on computer vision and pattern recognition. **2015**: 343-352.
   + **代码**：https://github.com/mihaibujanca/dynamicfusion

5. ReFusion（动态场景利用残差三维重建）

   + **论文**：Palazzolo E, Behley J, Lottes P, et al. [**ReFusion: 3D Reconstruction in Dynamic Environments for RGB-D Cameras Exploiting Residuals**](https://arxiv.org/pdf/1905.02082.pdf)[J]. arXiv preprint arXiv:1905.02082, **2019**.
   + **代码**：https://github.com/PRBonn/refusion ；[**Video**](https://www.youtube.com/watch?v=1P9ZfIS5-p4&feature=youtu.be)

6. DynSLAM（室外大规模稠密重建）

   + **论文**：Bârsan I A, Liu P, Pollefeys M, et al. [**Robust dense mapping for large-scale dynamic environments**](https://arxiv.org/pdf/1905.02781.pdf?utm_term)[C]//2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2018**: 7510-7517.
   + **代码**：https://github.com/AndreiBarsan/DynSLAM
   + **作者博士学位论文**：Barsan I A. [**Simultaneous localization and mapping in dynamic scenes**](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/202829/1/Barsan_Ioan.pdf)[D]. ETH Zurich, Department of Computer Science, **2017**.

7. VDO-SLAM（动态物体感知的 SLAM）

   + **论文**：Zhang J, Henein M, Mahony R, et al. [**VDO-SLAM: A Visual Dynamic Object-aware SLAM System**](https://arxiv.org/abs/2005.11052)[J]. arXiv preprint arXiv:2005.11052, **2020**.（IJRR Under Review）
      + 相关论文
         + IROS 2020 [Robust Ego and Object 6-DoF Motion Estimation and Tracking](https://arxiv.org/abs/2007.13993)
         + ICRA 2020 [Dynamic SLAM: The Need For Speed](https://arxiv.org/abs/2002.08584)
   **代码**：https://github.com/halajun/VDO_SLAM | [video](https://drive.google.com/file/d/1PbL4KiJ3sUhxyJSQPZmRP6mgi9dIC0iu/view)

1.6 Mapping
--------------

1. InfiniTAM（跨平台 CPU 实时重建）

   + 论文：Prisacariu V A, Kähler O, Golodetz S, et al. [**Infinitam v3: A framework for large-scale 3d reconstruction with loop closure**](https://arxiv.org/pdf/1708.00783)[J]. arXiv preprint arXiv:1708.00783, **2017**.
   + 代码：https://github.com/victorprad/InfiniTAM ；[project page](http://www.robots.ox.ac.uk/~victor/infinitam/)

2. BundleFusion

   + **论文**：Dai A, Nießner M, Zollhöfer M, et al. [**Bundlefusion: Real-time globally consistent 3d reconstruction using on-the-fly surface reintegration**](https://arxiv.org/pdf/1604.01093.pdf)[J]. ACM Transactions on Graphics (TOG), **2017**, 36(4): 76a.
   + **代码**：https://github.com/niessner/BundleFusion ；[工程地址](http://graphics.stanford.edu/projects/bundlefusion/)

3. KinectFusion

   + **论文**：Newcombe R A, Izadi S, Hilliges O, et al. [**KinectFusion: Real-time dense surface mapping and tracking**](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/kinectfusion-uist-comp.pdf)[C]//2011 10th IEEE International Symposium on Mixed and Augmented Reality. IEEE, **2011**: 127-136.
   + **代码**：https://github.com/chrdiller/KinectFusionApp

4. ElasticFusion

   + **论文**：Whelan T, Salas-Moreno R F, Glocker B, et al. [**ElasticFusion: Real-time dense SLAM and light source estimation**](https://spiral.imperial.ac.uk/bitstream/10044/1/39502/4/Whelan16ijrr.pdf)[J]. The International Journal of Robotics Research, **2016**, 35(14): 1697-1716.
   + **代码**：https://github.com/mp3guy/ElasticFusion

5. Kintinuous

   + ElasticFusion 同一个团队的工作，帝国理工 Stefan Leutenegger [谷歌学术](https://scholar.google.com/citations?user=SmGQ48gAAAAJ&hl=zh-CN&oi=sra)
   + **论文**：Whelan T, Kaess M, Johannsson H, et al. [**Real-time large-scale dense RGB-D SLAM with volumetric fusion**](https://dspace.mit.edu/bitstream/handle/1721.1/97583/Leonard_Real-time.pdf%3Bjsessionid%3D8C351776D7D5E5C614AF641625837212?sequence%3D1)[J]. The International Journal of Robotics Research, **2015**, 34(4-5): 598-626.
   + **代码**：https://github.com/mp3guy/Kintinuous

6. ElasticReconstruction

   + **论文**：Choi S, Zhou Q Y, Koltun V. [**Robust reconstruction of indoor scenes**](https://www.cv-foundation.org/openaccess/content_cvpr_2015/papers/Choi_Robust_Reconstruction_of_2015_CVPR_paper.pdf)[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. **2015**: 5556-5565.
   + **代码**：https://github.com/qianyizh/ElasticReconstruction ；[作者主页](http://qianyi.info/publication.html)

7. FlashFusion

   + **论文**：Han L, Fang L. [**FlashFusion: Real-time Globally Consistent Dense 3D Reconstruction using CPU Computing**](http://www.roboticsproceedings.org/rss14/p06.pdf)[C]. RSS, 2018.
   + **代码**（一直没放出来）：https://github.com/lhanaf/FlashFusion ； [Project Page](http://www.luvision.net/FlashFusion/?tdsourcetag=s_pctim_aiomsg)

8. RTAB-Map（激光视觉稠密重建）

   + **论文**：Labbé M, Michaud F. [**RTAB‐Map as an open‐source lidar and visual simultaneous localization and mapping library for large‐scale and long‐term online operation**](https://pdfs.semanticscholar.org/3957/7f85f3b1a16f496a2160d1a71894d12c1acc.pdf)[J]. Journal of Field Robotics, **2019**, 36(2): 416-446.
   + **代码**：https://github.com/introlab/rtabmap ；[Video](https://www.youtube.com/user/matlabbe) ；[project page](http://introlab.github.io/rtabmap/)

9. RobustPCLReconstruction（户外稠密重建）

   + **论文**：Lan Z, Yew Z J, Lee G H. [**Robust Point Cloud Based Reconstruction of Large-Scale Outdoor Scenes**](http://openaccess.thecvf.com/content_CVPR_2019/papers/Lan_Robust_Point_Cloud_Based_Reconstruction_of_Large-Scale_Outdoor_Scenes_CVPR_2019_paper.pdf)[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. **2019**: 9690-9698.
   + **代码**：https://github.com/ziquan111/RobustPCLReconstruction ；[Video](https://www.youtube.com/watch?v=ZZQT_REkItU)

10. plane-opt-rgbd（室内平面重建）

   + **论文**：Wang C, Guo X. [**Efficient Plane-Based Optimization of Geometry and Texture for Indoor RGB-D Reconstruction**](http://openaccess.thecvf.com/content_CVPRW_2019/papers/SUMO/Wang_Efficient_Plane-Based_Optimization_of_Geometry_and_Texture_for_Indoor_RGB-D_CVPRW_2019_paper.pdf)[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition Workshops. **2019**: 49-53.
   + **代码**：https://github.com/chaowang15/plane-opt-rgbd

11. DenseSurfelMapping（稠密表面重建）

   + **论文**：Wang K, Gao F, Shen S. [**Real-time scalable dense surfel mapping**](https://arxiv.org/pdf/1909.04250.pdf)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 6919-6925.
   + **代码**：https://github.com/HKUST-Aerial-Robotics/DenseSurfelMapping

12. surfelmeshing（网格重建）

   + **论文**：Schöps T, Sattler T, Pollefeys M. [**Surfelmeshing: Online surfel-based mesh reconstruction**](https://arxiv.org/pdf/1810.00729.pdf)[J]. IEEE Transactions on Pattern Analysis and Machine Intelligence, **2019**.
   + **代码**：https://github.com/puzzlepaint/surfelmeshing

13. DPPTAM（单目稠密重建）

   + **论文**：Concha Belenguer A, Civera Sancho J. [**DPPTAM: Dense piecewise planar tracking and mapping from a monocular sequence**](https://zaguan.unizar.es/record/36752/files/texto_completo.pdf)[C]//Proc. IEEE/RSJ Int. Conf. Intell. Rob. Syst. **2015** (ART-2015-92153).
   + **代码**：https://github.com/alejocb/dpptam
   + **相关研究**：基于超像素的单目 SLAM：[**Using Superpixels in Monocular SLAM**](http://webdiis.unizar.es/~jcivera/papers/concha_civera_icra14.pdf) ICRA 2014 ；[谷歌学术](https://scholar.google.com/citations?user=GIaG3CsAAAAJ&hl=zh-CN&oi=sra)

14. VI-MEAN（单目视惯稠密重建）

   + **论文**：Yang Z, Gao F, Shen S. [**Real-time monocular dense mapping on aerial robots using visual-inertial fusion**](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7989529)[C]//2017 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2017: 4552-4559.
   + **代码**：https://github.com/dvorak0/VI-MEAN ；[Video](https://www.youtube.com/watch?v=M4BMks6bQbc)

15. REMODE（单目概率稠密重建）

   + **论文**：Pizzoli M, Forster C, Scaramuzza D. [**REMODE: Probabilistic, monocular dense reconstruction in real time**](https://files.ifi.uzh.ch/rpg/website/rpg.ifi.uzh.ch/html/docs/ICRA14_Pizzoli.pdf)[C]//2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014: 2609-2616.
   + **原始开源代码**：https://github.com/uzh-rpg/rpg_open_remode
   + **与 ORB-SLAM2 结合版本**：https://github.com/ayushgaud/ORB_SLAM2  https://github.com/ayushgaud/ORB_SLAM2

16. DeepFactors（实时的概率单目稠密 SLAM）

   + 帝国理工学院戴森机器人实验室
   + **论文**：Czarnowski J, Laidlow T, Clark R, et al. [**DeepFactors: Real-Time Probabilistic Dense Monocular SLAM**](https://arxiv.org/pdf/2001.05049.pdf)[J]. arXiv preprint arXiv:2001.05049, **2020**.
   + **代码**：https://github.com/jczarnowski/DeepFactors （还未放出）
   + 其他论文：Bloesch M, Czarnowski J, Clark R, et al. [**CodeSLAM—learning a compact, optimisable representation for dense visual SLAM**](http://openaccess.thecvf.com/content_cvpr_2018/papers/Bloesch_CodeSLAM_--_Learning_CVPR_2018_paper.pdf)[C]//Proceedings of the IEEE conference on computer vision and pattern recognition. **2018**: 2560-2568.

17. probabilistic_mapping（单目概率稠密重建）

   + 港科沈邵劼老师团队
   + **论文**：Ling Y, Wang K, Shen S. [**Probabilistic dense reconstruction from a moving camera**](https://arxiv.org/pdf/1903.10673.pdf)[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2018**: 6364-6371.
   + **代码**：https://github.com/ygling2008/probabilistic_mapping
   + 另外一篇稠密重建文章的代码一直没放出来 [Github](https://github.com/ygling2008/dense_mapping) ：Ling Y, Shen S. [**Real‐time dense mapping for online processing and navigation**](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21868)[J]. Journal of Field Robotics, **2019**, 36(5): 1004-1036.

18. ORB-SLAM2 单目半稠密建图

   + **论文**：Mur-Artal R, Tardós J D. [Probabilistic Semi-Dense Mapping from Highly Accurate Feature-Based Monocular SLAM](https://www.researchgate.net/profile/Raul_Mur-Artal/publication/282807894_Probabilistic_Semi-Dense_Mapping_from_Highly_Accurate_Feature-Based_Monocular_SLAM/links/561cd04308ae6d17308ce267.pdf)[C]//Robotics: Science and Systems. **2015**, 2015.
   + **代码**（本身没有开源，贺博复现的一个版本）：https://github.com/HeYijia/ORB_SLAM2
   + 加上线段之后的半稠密建图
      + **论文**：He S, Qin X, Zhang Z, et al. [**Incremental 3d line segment extraction from semi-dense slam**](https://arxiv.org/pdf/1708.03275)[C]//2018 24th International Conference on Pattern Recognition (ICPR). IEEE, **2018**: 1658-1663.
      + **代码**：https://github.com/shidahe/semidense-lines
      + 作者在此基础上用于指导远程抓取操作的一项工作：https://github.com/atlas-jj/ORB-SLAM-free-space-carving

19. Voxgraph（SDF 体素建图）

   + **论文**：Reijgwart V, Millane A, Oleynikova H, et al. [**Voxgraph: Globally Consistent, Volumetric Mapping Using Signed Distance Function Submaps**](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/385682/1/Voxgraph-ETHpreprintversion.pdf)[J]. IEEE Robotics and Automation Letters, **2019**, 5(1): 227-234.
   + **代码**：https://github.com/ethz-asl/voxgraph

20. SegMap（三维分割建图）

   + **论文**：Dubé R, Cramariuc A, Dugas D, et al. [**SegMap: 3d segment mapping using data-driven descriptors**](https://arxiv.org/pdf/1804.09557)[J]. arXiv preprint arXiv:1804.09557, **2018**.
   + **代码**：https://github.com/ethz-asl/segmap

21. OpenREALM：无人机实时建图框架

   + **论文**：Kern A, Bobbe M, Khedar Y, et al. [**OpenREALM: Real-time Mapping for Unmanned Aerial Vehicles**](https://arxiv.org/abs/2009.10492)[J]. arXiv preprint arXiv:2009.10492, **2020**.
   + **代码**：https://github.com/laxnpander/OpenREALM

22. c-blox：可拓展的 TSDF 稠密建图

   + **论文**：Millane A, Taylor Z, Oleynikova H, et al. [**C-blox: A scalable and consistent tsdf-based dense mapping approach**](https://ieeexplore.ieee.org/abstract/document/8593427/)[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2018**: 995-1002.
   + **代码**：https://github.com/ethz-asl/cblox

**2.优秀作者与实验室**

**1 美国卡耐基梅陇大学机器人研究所**

.. NOTE::

   * 研究方向：机器人感知、结构，服务型、运输、制造业、现场机器
   * 研究所主页：https://www.ri.cmu.edu/
   * 下属 Field Robotic Center 主页：https://frc.ri.cmu.edu/
   * 发表论文：https://www.ri.cmu.edu/pubs/
   * Michael Kaess：个人主页 ，谷歌学术
   * Sebastian Scherer：个人主页 ，谷歌学术
   * Kaess M, Ranganathan A, Dellaert F. iSAM: Incremental smoothing and mapping[J]. IEEE Transactions on Robotics, 2008, 24(6): 1365-1378.
   * Hsiao M, Westman E, Zhang G, et al. Keyframe-based dense planar SLAM[C]//2017 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2017: 5110-5117.
   * Kaess M. Simultaneous localization and mapping with infinite planes[C]//2015 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2015: 4605-4611.

**2 美国加州大学圣地亚哥分校语境机器人研究所**

.. NOTE::

   * 研究方向：多模态环境理解，语义导航，自主信息获取
   * 实验室主页：https://existentialrobotics.org/index.html
   * 发表论文汇总：https://existentialrobotics.org/pages/publications.html
   * Nikolay Atanasov：个人主页 谷歌学术
   * 机器人状态估计与感知课程 ppt：https://natanaso.github.io/ece276a2019/schedule.html
   * 语义 SLAM 经典论文：Bowman S L, Atanasov N, Daniilidis K, et al. Probabilistic data association for semantic slam[C]//2017 IEEE international conference on robotics and automation (ICRA). IEEE, 2017: 1722-1729.
   * 实例网格模型定位与建图：Feng Q, Meng Y, Shan M, et al. Localization and Mapping using Instance-specific Mesh Models[C]//2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2019: 4985-4991.
   * 基于事件相机的 VIO：Zihao Zhu A, Atanasov N, Daniilidis K. Event-based visual inertial odometry[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. 2017: 5391-5399.


**3 美国特拉华大学机器人感知与导航组**

.. NOTE::

   * 研究方向：SLAM、VINS、语义定位与建图等
   * 实验室主页：https://sites.udel.edu/robot/
   * 发表论文汇总：https://sites.udel.edu/robot/publications/
   * Github 地址：https://github.com/rpng?page=2
   * Geneva P, Eckenhoff K, Lee W, et al. Openvins: A research platform for visual-inertial estimation[C]//IROS 2019 Workshop on Visual-Inertial Navigation: Challenges and Applications, Macau, China. IROS 2019.（代码：https://github.com/rpng/open_vins ）
   * Huai Z, Huang G. Robocentric visual-inertial odometry[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018: 6319-6326.（代码：https://github.com/rpng/R-VIO ）
   * Zuo X, Geneva P, Yang Y, et al. Visual-Inertial Localization With Prior LiDAR Map Constraints[J]. IEEE Robotics and Automation Letters, 2019, 4(4): 3394-3401.
   * Zuo X, Ye W, Yang Y, et al. Multimodal localization: Stereo over LiDAR map[J]. Journal of Field Robotics, 2020 （ 左星星博士谷歌学术）
   * `黄国权教授主页 <http://udel.edu/~ghuang/index.html>`_

**4 美国麻省理工学院航空航天实验室**

.. NOTE::

   + **研究方向**：位姿估计与导航，路径规划，控制与决策，机器学习与强化学习
   + **实验室主页**：http://acl.mit.edu/
   + **发表论文**：http://acl.mit.edu/publications （实验室的**学位论文**也可以在这里找到）
   + 👦 **Jonathan P. How** 教授：[个人主页](http://www.mit.edu/people/jhow/) &emsp;[谷歌学术](https://scholar.google.com/citations?user=gX7rSCcAAAAJ&hl=en)
   + 👦 **Kasra Khosoussi**（SLAM 图优化）：[谷歌学术](https://scholar.google.com/citations?user=SRCCuo0AAAAJ&hl=zh-CN&oi=sra)
   + 📜 **物体级 SLAM**：Mu B, Liu S Y, Paull L, et al. [**Slam with objects using a nonparametric pose graph**](https://arxiv.org/pdf/1704.05959.pdf?source=post_page---------------------------)[C]//2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2016**: 4602-4609.（**代码**：https://github.com/BeipengMu/objectSLAM）
   + 📜 **物体级 SLAM 导航**：Ok K, Liu K, Frey K, et al. [**Robust Object-based SLAM for High-speed Autonomous Navigation**](http://groups.csail.mit.edu/rrg/papers/OkLiu19icra.pdf)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 669-675.
   + 📜 **SLAM 的图优化**：Khosoussi, K., Giamou, M., Sukhatme, G., Huang, S., Dissanayake, G., and How, J. P., [**Reliable Graphs for SLAM**](https://journals.sagepub.com/doi/full/10.1177/0278364918823086) [C]//International Journal of Robotics Research (IJRR), 2019.

**5 美国麻省理工学院 SPARK 实验室**

.. NOTE::

   + **研究方向**：移动机器人环境感知
   + **实验室主页**：http://web.mit.edu/sparklab/
   + 👦 **Luca Carlone** 教授：[个人主页](https://lucacarlone.mit.edu/) &emsp;[谷歌学术](https://scholar.google.com/citations?user=U4kKRdMAAAAJ&hl=zh-CN&oi=sra)
   + 📜 **SLAM 经典综述**：Cadena C, Carlone L, Carrillo H, et al. [**Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age**](https://arxiv.org/pdf/1606.05830)[J]. IEEE Transactions on robotics, **2016**, 32(6): 1309-1332.
   + 📜 **VIO 流形预积分**：Forster C, Carlone L, Dellaert F, et al. [**On-Manifold Preintegration for Real-Time Visual--Inertial Odometry**](https://arxiv.org/pdf/1512.02363)[J]. IEEE Transactions on Robotics, **2016**, 33(1): 1-21.
   + 📜 **开源语义 SLAM**：Rosinol A, Abate M, Chang Y, et al. [**Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping**](https://arxiv.org/pdf/1910.02490)[J]. arXiv preprint arXiv:1910.02490, **2019**.（代码：https://github.com/MIT-SPARK/Kimera ）

**6 美国麻省理工学院海洋机器人组**

.. NOTE::

   + **研究方向**：水下或陆地移动机器人导航与建图
   + **实验室主页**：https://marinerobotics.mit.edu/ （隶属于 MIT [计算机科学与人工智能实验室](https://www.csail.mit.edu/)）
   + 👦 **John Leonard 教授**：[谷歌学术](https://scholar.google.com/citations?user=WPe7vWwAAAAJ&hl=zh-CN&authuser=1&oi=ao)
   + **发表论文汇总**：https://marinerobotics.mit.edu/biblio
   + 📜 **面向物体的 SLAM**：Finman R, Paull L, Leonard J J. [**Toward object-based place recognition in dense rgb-d maps**](http://marinerobotics.mit.edu/sites/default/files/icra2015.pdf)[C]//ICRA Workshop Visual Place Recognition in Changing Environments, Seattle, WA. **2015**.
   + 📜 **拓展 KinectFusion**：Whelan T, Kaess M, Fallon M, et al. [**Kintinuous: Spatially extended kinectfusion**]()[J]. **2012**.
   + 📜 **语义 SLAM 概率数据关联**：Doherty K, Fourie D, Leonard J. [**Multimodal semantic slam with probabilistic data association**](https://marinerobotics.mit.edu/sites/default/files/doherty_icra2019_revised.pdf)[C]//2019 international conference on robotics and automation (ICRA). IEEE, **2019**: 2419-2425.


**7 美国明尼苏达大学多元自主机器人系统实验室**

.. NOTE::

   + **研究方向**：视觉、激光、惯性导航系统，移动设备大规模三维建模与定位
   + **实验室主页**：http://mars.cs.umn.edu/index.php
   + **发表论文汇总**：http://mars.cs.umn.edu/publications.php
   + 👦 **Stergios I. Roumeliotis**：[个人主页](https://www-users.cs.umn.edu/~stergios/) ，[谷歌学术](https://scholar.google.com/citations?user=c5HeXxsAAAAJ&hl=zh-CN&oi=ao)
   + 📜 **移动设备 VIO**：Wu K, Ahmed A, Georgiou G A, et al. [**A Square Root Inverse Filter for Efficient Vision-aided Inertial Navigation on Mobile Devices**](http://roboticsproceedings.org/rss11/p08.pdf)[C]//Robotics: Science and Systems. **2015**, 2.（**项目主页**：http://mars.cs.umn.edu/research/sriswf.php ）
   + 📜 **移动设备大规模三维半稠密建图**：Guo C X, Sartipi K, DuToit R C, et al. [**Resource-aware large-scale cooperative three-dimensional mapping using multiple mobile devices**](https://pdfs.semanticscholar.org/e0fd/6d963307a0d5d6dfb6f05ad21845dd4f40c8.pdf)[J]. IEEE Transactions on Robotics, **2018**, 34(5): 1349-1369. （**项目主页**：http://mars.cs.umn.edu/research/semi_dense_mapping.php ）
   + 📜 **VIO 相关研究**：http://mars.cs.umn.edu/research/vins_overview.php

**8 美国宾夕法尼亚大学 Vijay Kumar 实验室**

.. NOTE::

   + **研究方向**：自主微型无人机
   + **实验室主页**：https://www.kumarrobotics.org/
   + **发表论文**：https://www.kumarrobotics.org/publications/
   + **研究成果视频**：https://www.youtube.com/user/KumarLabPenn/videos
   + 📜 **无人机半稠密 VIO**：Liu W, Loianno G, Mohta K, et al. [**Semi-Dense Visual-Inertial Odometry and Mapping for Quadrotors with SWAP Constraints**](https://www.cis.upenn.edu/~kostas/mypub.dir/wenxin18icra.pdf)[C]//2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2018**: 1-6.
   + 📜 **语义数据关联**：Liu X, Chen S W, Liu C, et al. [**Monocular Camera Based Fruit Counting and Mapping with Semantic Data Association**](https://arxiv.org/pdf/1811.01417)[J]. IEEE Robotics and Automation Letters, **2019**, 4(3): 2296-2303.

**9 Srikumar Ramalingam（美国犹他大学计算机学院）**

.. NOTE::

   + **研究方向**：三维重构、语义分割、视觉 SLAM、图像定位、深度神经网络
   + 👦 **Srikumar Ramalingam**：[个人主页](https://www.cs.utah.edu/~srikumar/) &emsp; [谷歌学术](https://scholar.google.com/citations?user=6m1ptOgAAAAJ&hl=en/)
   + 📜 **点面 SLAM**：Taguchi Y, Jian Y D, Ramalingam S, et al. [**Point-plane SLAM for hand-held 3D sensors**](https://merl.com/publications/docs/TR2013-031.pdf)[C]//2013 IEEE international conference on robotics and automation. IEEE, **2013**: 5182-5189.
   + 📜 **点线定位**：Ramalingam S, Bouaziz S, Sturm P. [**Pose estimation using both points and lines for geo-localization**](https://hal.inria.fr/inria-00590279/document)[C]//2011 IEEE International Conference on Robotics and Automation. IEEE, **2011**: 4716-4723.（[视频](https://www.youtube.com/watch?v=wc7hK0zEkCw&feature=emb_logo)）
   + 📜 **2D 3D 定位**：Ataer-Cansizoglu E, Taguchi Y, Ramalingam S. [**Pinpoint SLAM: A hybrid of 2D and 3D simultaneous localization and mapping for RGB-D sensors**](http://yuichitaguchi.com/pub/16ICRA_PinpointSLAM.pdf)[C]//2016 IEEE international conference on robotics and automation (ICRA). IEEE, **2016**: 1300-1307.（[视频](https://www.youtube.com/watch?v=iZ1psxcMvrQ&feature=emb_logo)）

**10 Frank Dellaert（美国佐治亚理工学院机器人与智能机器研究中心）**

.. NOTE::

   + **研究方向**：SLAM，图像时空重构
   + 👦 [个人主页](https://www.cc.gatech.edu/~dellaert/FrankDellaert/Frank_Dellaert/Frank_Dellaert.html)，[谷歌学术](https://scholar.google.com/citations?user=ZxXBaswAAAAJ&hl=en)
   + 📜 **因子图**：Dellaert F. [**Factor graphs and GTSAM: A hands-on introduction**](https://smartech.gatech.edu/handle/1853/45226)[R]. Georgia Institute of Technology, **2012**. （GTSAM 代码：http://borg.cc.gatech.edu/ ）
   + 📜 **多机器人分布式 SLAM**：Cunningham A, Wurm K M, Burgard W, et al. [**Fully distributed scalable smoothing and mapping with robust multi-robot data association**](https://smartech.gatech.edu/bitstream/handle/1853/44686/Cunningham12icra.pdf?sequence=1&isAllowed=y)[C]//2012 IEEE International Conference on Robotics and Automation. IEEE, **2012**: 1093-1100.
   + 📜 Choudhary S, Trevor A J B, Christensen H I, et al. [**SLAM with object discovery, modeling and mapping**](https://smartech.gatech.edu/bitstream/handle/1853/53723/Choudhary14iros.pdf)[C]//2014 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, **2014**: 1018-1025.

**11 Patricio Vela （美国佐治亚理工学院智能视觉与自动化实验室**

.. NOTE::

   + **研究方向**：机器人控制、定位与导航
   + **实验室主页**：http://ivalab.gatech.edu/
   + 👦 Patricio Vela [个人主页](https://pvela.gatech.edu/)
   + 👦 赵轶璞 [个人主页](https://sites.google.com/site/zhaoyipu/home?authuser=0) &emsp; [谷歌学术](https://scholar.google.com/citations?user=HiM_WcYAAAAJ&hl=zh-CN&authuser=1&oi=ao)
   + 📜 Zhao Y, Smith J S, Karumanchi S H, et al. [**Closed-Loop Benchmarking of Stereo Visual-Inertial SLAM Systems: Understanding the Impact of Drift and Latency on Tracking Accuracy**](https://arxiv.org/pdf/2003.01317)[J]. arXiv preprint arXiv:2003.01317, **2020**.
   + 📜 Zhao Y, Vela P A. [**Good feature selection for least squares pose optimization in VO/VSLAM**](https://arxiv.org/pdf/1905.07807)[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2018**: 1183-1189.（**代码**：https://github.com/ivalab/FullResults_GoodFeature ）
   + 📜 Zhao Y, Vela P A. [**Good line cutting: Towards accurate pose tracking of line-assisted VO/VSLAM**](http://openaccess.thecvf.com/content_ECCV_2018/papers/Yipu_Zhao_Good_Line_Cutting_ECCV_2018_paper.pdf)[C]//Proceedings of the European Conference on Computer Vision (ECCV). **2018**: 516-531. （**代码**：https://github.com/ivalab/GF_PL_SLAM ）

**12 加拿大蒙特利尔大学 机器人与嵌入式 AI 实验室**

.. NOTE::

   + **研究方向**：SLAM，不确定性建模
   + **实验室主页**：http://montrealrobotics.ca/
   + 👦 **Liam Paull** 教授：[个人主页](https://liampaull.ca/index.html)&emsp;[谷歌学术](https://scholar.google.com/citations?user=H9xADK0AAAAJ&hl=zh-CN&oi=ao)
   + 📜 Mu B, Liu S Y, Paull L, et al. [**Slam with objects using a nonparametric pose graph**](https://arxiv.org/pdf/1704.05959.pdf?source=post_page---------------------------)[C]//2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2016**: 4602-4609.（**代码**：https://github.com/BeipengMu/objectSLAM）
   + 📜 Murthy Jatavallabhula K, Iyer G, Paull L. [**gradSLAM: Dense SLAM meets Automatic Differentiation**](http://adsabs.harvard.edu/abs/2019arXiv191010672M)[J]. arXiv preprint arXiv:1910.10672, **2019**.（**代码**：https://github.com/montrealrobotics/gradSLAM ）

**13. 加拿大舍布鲁克大学智能、交互、综合、跨学科机器人实验室**

.. NOTE::

   + **研究方向**：移动机器人软硬件设计
   + **实验室主页**：https://introlab.3it.usherbrooke.ca/
   + 📜 **激光视觉稠密重建**：Labbé M, Michaud F. [**RTAB‐Map as an open‐source lidar and visual simultaneous localization and mapping library for large‐scale and long‐term online operation**](https://pdfs.semanticscholar.org/3957/7f85f3b1a16f496a2160d1a71894d12c1acc.pdf)[J]. Journal of Field Robotics, **2019**, 36(2): 416-446.
   + 代码：https://github.com/introlab/rtabmap
   + 项目主页：http://introlab.github.io/rtabmap/

**14. 瑞士苏黎世大学机器人与感知课题组**

.. NOTE::

   + **研究方向**：移动机器人、无人机环境感知与导航，**VISLAM**，**事件相机**
   + **实验室主页**：http://rpg.ifi.uzh.ch/index.html
   + **发表论文汇总**：http://rpg.ifi.uzh.ch/publications.html
   + **Github 代码公开地址**：https://github.com/uzh-rpg
   + 📜 Forster C, Pizzoli M, Scaramuzza D. [**SVO: Fast semi-direct monocular visual odometry**](https://www.zora.uzh.ch/id/eprint/125453/1/ICRA14_Forster.pdf)[C]//2014 IEEE international conference on robotics and automation (ICRA). IEEE, **2014**: 15-22.
   + 📜 VO/VIO 轨迹评估工具 **rpg_trajectory_evaluation**：https://github.com/uzh-rpg/rpg_trajectory_evaluation
   + 📜 事件相机项目主页：http://rpg.ifi.uzh.ch/research_dvs.html
   + 👦 **人物**：[Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html) &emsp;[张子潮](https://www.ifi.uzh.ch/en/rpg/people/zichao.html)

**15. 瑞士苏黎世联邦理工计算机视觉与几何实验室**

.. NOTE::

   + **研究方向**：定位、三维重建、语义分割、机器人视觉
   + **实验室主页**：http://www.cvg.ethz.ch/index.php
   + **发表论文**：http://www.cvg.ethz.ch/publications/
   + 📜 **视觉语义里程计**：Lianos K N, Schonberger J L, Pollefeys M, et al. [**Vso: Visual semantic odometry**](http://openaccess.thecvf.com/content_ECCV_2018/papers/Konstantinos-Nektarios_Lianos_VSO_Visual_Semantic_ECCV_2018_paper.pdf)[C]//Proceedings of the European conference on computer vision (ECCV). **2018**: 234-250.
   + 📜 **视觉语义定位**：CVPR 2018 [**Semantic visual localization**](http://openaccess.thecvf.com/content_cvpr_2018/papers/Schonberger_Semantic_Visual_Localization_CVPR_2018_paper.pdf)
   + **作者博士学位论文**：2018 [**Robust Methods for Accurate and Efficient 3D Modeling from Unstructured Imagery**](https://www.research-collection.ethz.ch/handle/20.500.11850/295763)
   + 📜 **大规模户外建图**：Bârsan I A, Liu P, Pollefeys M, et al. [**Robust dense mapping for large-scale dynamic environments**](https://arxiv.org/pdf/1905.02781.pdf?utm_term)[C]//2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2018**: 7510-7517.
   + **代码**：https://github.com/AndreiBarsan/DynSLAM 
   + **作者博士学位论文**：Barsan I A. [**Simultaneous localization and mapping in dynamic scenes**](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/202829/1/Barsan_Ioan.pdf)[D]. ETH Zurich, Department of Computer Science, **2017**.
   + 👦 **Marc Pollefeys**：[个人主页](http://people.inf.ethz.ch/pomarc/index.html)，[谷歌学术](https://scholar.google.com/citations?user=YYH0BjEAAAAJ&hl=zh-CN&oi=ao)
   + 👦 **Johannes L. Schönberger**：[个人主页](https://demuc.de/)，[谷歌学术](https://scholar.google.com/citations?user=MlcMCd0AAAAJ)

**16. 英国帝国理工学院戴森机器人实验室**

.. NOTE::

   + **研究方向**：机器人视觉场景与物体理解、机器人操纵
   + **实验室主页**：https://www.imperial.ac.uk/dyson-robotics-lab/
   + **发表论文**：https://www.imperial.ac.uk/dyson-robotics-lab/publications/
   + **代表性工作**：**MonoSLAM、CodeSLAM、ElasticFusion、KinectFusion**
   + 📜 **ElasticFusion**：Whelan T, Leutenegger S, Salas-Moreno R, et al. [**ElasticFusion: Dense SLAM without a pose graph**](https://spiral.imperial.ac.uk/bitstream/10044/1/23438/2/whelan2015rss.pdf)[C]. Robotics: Science and Systems, **2015**.（**代码**：https://github.com/mp3guy/ElasticFusion ）
   + 📜 **Semanticfusion**：McCormac J, Handa A, Davison A, et al. [**Semanticfusion: Dense 3d semantic mapping with convolutional neural networks**](https://arxiv.org/pdf/1609.05130)[C]//2017 IEEE International Conference on Robotics and automation (ICRA). IEEE, **2017**: 4628-4635.（**代码**：https://github.com/seaun163/semanticfusion ）
   + 📜 **Code-SLAM**：Bloesch M, Czarnowski J, Clark R, et al. [**CodeSLAM—learning a compact, optimisable representation for dense visual SLAM**](http://openaccess.thecvf.com/content_cvpr_2018/papers/Bloesch_CodeSLAM_--_Learning_CVPR_2018_paper.pdf)[C]//Proceedings of the IEEE conference on computer vision and pattern recognition. **2018**: 2560-2568.
   + 👦 **Andrew Davison**：[谷歌学术](https://scholar.google.com/citations?user=A0ae1agAAAAJ&hl=zh-CN&oi=ao)

**17. 英国牛津大学信息工程学**

.. NOTE::

   + **研究方向**：SLAM、目标跟踪、运动结构、场景增强、移动机器人运动规划、导航与建图等等等
   + **实验室主页**：http://www.robots.ox.ac.uk/
   + 主动视觉实验室：http://www.robots.ox.ac.uk/ActiveVision/
   + 牛津机器人学院：https://ori.ox.ac.uk/
   + **发表论文汇总**：
   + 主动视觉实验室：http://www.robots.ox.ac.uk/ActiveVision/Publications/index.html
   + 机器人学院：https://ori.ox.ac.uk/publications/papers/
   + **代表性工作**：
   + 📜 Klein G, Murray D. [**PTAM: Parallel tracking and mapping for small AR workspaces**](https://dl.acm.org/ft_gateway.cfm?id=1514363&type=pdf)[C]//2007 6th IEEE and ACM international symposium on mixed and augmented reality. IEEE, **2007**: 225-234.
   + 📜 RobotCar 数据集：https://robotcar-dataset.robots.ox.ac.uk/
   + 👦 **人物**（谷歌学术）：[David Murray](https://scholar.google.com.hk/citations?hl=zh-CN&user=O5QreiwAAAAJ) &emsp; [Maurice Fallon](https://ori.ox.ac.uk/ori-people/maurice-fallon/)
   + 部分博士学位论文可以在这里搜到：https://ora.ox.ac.uk/

**18. 德国慕尼黑工业大学计算机视觉组**

.. NOTE::

   + **研究方向**：三维重建、机器人视觉、深度学习、**视觉 SLAM** 等
   + **实验室主页**：https://vision.in.tum.de/research/vslam
   + **发表论文汇总**：https://vision.in.tum.de/publications
   + **代表作**：DSO、LDSO、LSD_SLAM、DVO_SLAM
   + 📜 **DSO**：Engel J, Koltun V, Cremers D. [**Direct sparse odometry**](https://ieeexplore.ieee.org/iel7/34/4359286/07898369.pdf)[J]. IEEE transactions on pattern analysis and machine intelligence, **2017**, 40(3): 611-625.（**代码**：https://github.com/JakobEngel/dso ）
   + 📜 **LSD-SLAM**： Engel J, Schöps T, Cremers D. [LSD-SLAM: Large-scale direct monocular SLAM](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.646.7193&rep=rep1&type=pdf)[C]//European conference on computer vision. Springer, Cham, **2014**: 834-849.（**代码**：https://github.com/tum-vision/lsd_slam ）2. 
   + **Github 地址**：https://github.com/tum-vision
   + 👦 **Daniel Cremers** 教授：[个人主页](https://vision.in.tum.de/members/cremers) [谷歌学术](https://scholar.google.com/citations?user=cXQciMEAAAAJ)
   + 👦 **Jakob Engel**（LSD-SLAM，DSO 作者）：[个人主页](https://jakobengel.github.io/) &emsp;[谷歌学术](https://scholar.google.de/citations?user=ndOMZXMAAAAJ)

**19. 德国马克斯普朗克智能系统研究所嵌入式视觉组**

.. NOTE::

   + **研究方向**：智能体自主环境理解、导航与物体操纵
   + **实验室主页**：https://ev.is.tuebingen.mpg.de/
   + 👦 负责人 **Jörg Stückler**（前 TUM 教授）：[个人主页](https://ev.is.tuebingen.mpg.de/person/jstueckler) &emsp; [谷歌学术](https://scholar.google.de/citations?user=xrOzfucAAAAJ&hl=de)
   + 📜 **发表论文汇总**：https://ev.is.tuebingen.mpg.de/publications
   + Kasyanov A, Engelmann F, Stückler J, et al. [**Keyframe-based visual-inertial online SLAM with relocalization**](https://arxiv.org/pdf/1702.02175)[C]//2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2017**: 6662-6669.
   + 📜 Strecke M, Stuckler J. [**EM-Fusion: Dynamic Object-Level SLAM with Probabilistic Data Association**](http://openaccess.thecvf.com/content_ICCV_2019/papers/Strecke_EM-Fusion_Dynamic_Object-Level_SLAM_With_Probabilistic_Data_Association_ICCV_2019_paper.pdf)[C]//Proceedings of the IEEE International Conference on Computer Vision. **2019**: 5865-5874.
   + 📜 Usenko, V., Demmel, N., Schubert, D., Stückler, J., Cremers, D. [**Visual-Inertial Mapping with Non-Linear Factor Recovery**](https://arxiv.org/pdf/1904.06504) IEEE Robotics and Automation Letters (RA-L), 5, **2020**

**20. 德国弗莱堡大学智能自主系统实验室**

.. NOTE::

   + **研究方向**：多机器人导航与协作，环境建模与状态估计
   + **实验室主页**：http://ais.informatik.uni-freiburg.de/index_en.php
   + **发表论文汇总**：http://ais.informatik.uni-freiburg.de/publications/index_en.php （学位论文也可以在这里找到）
   + 👦 **Wolfram Burgard**：[谷歌学术](https://scholar.google.com/citations?user=zj6FavAAAAAJ&hl=zh-CN&oi=ao)
   + **开放数据集**：http://aisdatasets.informatik.uni-freiburg.de/
   + 📜 **RGB-D SLAM**：Endres F, Hess J, Sturm J, et al. [**3-D mapping with an RGB-D camera**](http://perpustakaan.unitomo.ac.id/repository/3-D%20Mapping%20With%20an%20RGB-D%20Camera06594910.pdf)[J]. IEEE transactions on robotics, **2013**, 30(1): 177-187.（**代码**：https://github.com/felixendres/rgbdslam_v2 ）
   + 📜 **跨季节的 SLAM**：Naseer T, Ruhnke M, Stachniss C, et al. [**Robust visual SLAM across seasons**](http://ais.informatik.uni-freiburg.de/publications/papers/naseer15iros.pdf)[C]//2015 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2015**: 2529-2535.
   + 📜 **博士学位论文**：[Robust Graph-Based Localization and Mapping](http://ais.informatik.uni-freiburg.de/publications/papers/agarwal15phd.pdf) 2015
   + 📜 **博士学位论文**：[Discovering and Leveraging Deep Multimodal Structure for Reliable Robot Perception and Localization](http://ais.informatik.uni-freiburg.de/publications/papers/valada19phd.pdf) 2019
   + 📜 **博士学位论文**：[Robot Localization and Mapping in Dynamic Environments](https://freidok.uni-freiburg.de/fedora/objects/freidok:149938/datastreams/FILE1/content) 2019

**21. 西班牙萨拉戈萨大学机器人、感知与实时组 SLAM 实验室**

.. NOTE::

   + **研究方向**：视觉 SLAM、物体 SLAM、非刚性 SLAM、机器人、增强现实
   + **实验室主页**：http://robots.unizar.es/slamlab/
   + **发表论文**：http://robots.unizar.es/slamlab/?extra=3 （论文好像没更新，可以访问下面实验室大佬的谷歌学术查看最新论文）
   + 👦 **J. M. M. Montiel**：[谷歌学术](https://scholar.google.com/citations?user=D99JRxwAAAAJ&hl=zh-CN&oi=sra)
   + 📜 Mur-Artal R, Tardós J D. [**Orb-slam2: An open-source slam system for monocular, stereo, and rgb-d cameras**](https://github.com/raulmur/ORB_SLAM2)[J]. IEEE Transactions on Robotics, **2017**, 33(5): 1255-1262.
   + Gálvez-López D, Salas M, Tardós J D, et al. [**Real-time monocular object slam**](https://arxiv.org/pdf/1504.02398.pdf)[J]. Robotics and Autonomous Systems, **2016**, 75: 435-449.
   + 📜 Strasdat H, Montiel J M M, Davison A J. [**Real-time monocular SLAM: Why filter?**](http://www.hauke.strasdat.net/files/strasdat2010icra.pdf)[C]//2010 IEEE International Conference on Robotics and Automation. IEEE, **2010**: 2657-2664.
   + 📜 Zubizarreta J, Aguinaga I, Montiel J M M. [**Direct sparse mapping**](https://arxiv.org/pdf/1904.06577)[J]. arXiv preprint arXiv:1904.06577, **2019**.
   + Elvira R, Tardós J D, Montiel J M M. [**ORBSLAM-Atlas: a robust and accurate multi-map system**](https://arxiv.org/pdf/1908.11585)[J]. arXiv preprint arXiv:1908.11585, **2019**.

**22. 西班牙马拉加大学机器感知与智能机器人课题组**

.. NOTE::

   + **研究方向**：自主机器人、人工嗅觉、计算机视觉
   + **实验室主页**：http://mapir.uma.es/mapirwebsite/index.php/topics-2.html
   + **发表论文汇总**：http://mapir.isa.uma.es/mapirwebsite/index.php/publications-menu-home.html
   + 📜 Gomez-Ojeda R, Moreno F A, Zuñiga-Noël D, et al. [**PL-SLAM: a stereo SLAM system through the combination of points and line segments**](https://arxiv.org/pdf/1705.09479)[J]. IEEE Transactions on Robotics, **2019**, 35(3): 734-746.（代码：https://github.com/rubengooj/pl-slam ）
   + 👦 [**Francisco-Angel Moreno**](http://mapir.isa.uma.es/mapirwebsite/index.php/people/199-francisco-moreno-due%C3%B1as)
   + 👦 [**Ruben Gomez-Ojeda**](https://scholar.google.com/citations?user=7jne0V4AAAAJ&hl=zh-CN&oi=sra) 点线 SLAM
   + 📜 Gomez-Ojeda R, Briales J, Gonzalez-Jimenez J. [**PL-SVO: Semi-direct Monocular Visual Odometry by combining points and line segments**](http://mapir.isa.uma.es/rgomez/publications/iros16plsvo.pdf)[C]//Intelligent Robots and Systems (IROS), 2016 IEEE/RSJ International Conference on. IEEE, **2016**: 4211-4216.（**代码**：https://github.com/rubengooj/pl-svo ）
   + 📜 Gomez-Ojeda R, Gonzalez-Jimenez J. [**Robust stereo visual odometry through a probabilistic combination of points and line segments**](https://riuma.uma.es/xmlui/bitstream/handle/10630/11515/icra16rgo.pdf?sequence=1&isAllowed=y)[C]//2016 IEEE International Conference on Robotics and Automation (**ICRA**). IEEE, **2016**: 2521-2526.（**代码**：https://github.com/rubengooj/stvo-pl ）
   + 📜 Gomez-Ojeda R, Zuñiga-Noël D, Moreno F A, et al. [**PL-SLAM: a Stereo SLAM System through the Combination of Points and Line Segments**](https://arxiv.org/pdf/1705.09479.pdf)[J]. arXiv preprint arXiv:1705.09479, **2017**.（**代码**：https://github.com/rubengooj/pl-slam ）

**23. Alejo Concha（Oculus VR，西班牙萨拉戈萨大学）**

.. NOTE::

   + **研究方向**：SLAM，单目稠密重建，传感器融合
   + 👦 **个人主页**：https://sites.google.com/view/alejoconcha/ &emsp; [**谷歌学术**](https://scholar.google.com/citations?user=GIaG3CsAAAAJ&hl=zh-CN&oi=sra)
   + Github：https://github.com/alejocb
   + 📜 **IROS 2015** 单目平面重建：[**DPPTAM: Dense piecewise planar tracking and mapping from a monocular sequence**](https://zaguan.unizar.es/record/36752/files/texto_completo.pdf) （代码：https://github.com/alejocb/dpptam ）
   + 📜 **IROS 2017** 开源 RGB-D SLAM：[**RGBDTAM: A Cost-Effective and Accurate RGB-D Tracking and Mapping System**](http://webdiis.unizar.es/~jcivera/papers/concha_etal_icra16.pdf)（代码：https://github.com/alejocb/rgbdtam ）
   + 📜 **ICRA 2016**：[**Visual-inertial direct SLAM**](http://webdiis.unizar.es/~jcivera/papers/concha_etal_icra16.pdf)
   + 📜 **ICRA 2014**：[**Using Superpixels in Monocular SLAM**](https://www.researchgate.net/profile/Alejo_Concha/publication/281559193_Using_superpixels_in_monocular_SLAM/links/55edffcb08aedecb68fc6ac2/Using-superpixels-in-monocular-SLAM.pdf)
   + **RSS 2014**：[**Manhattan and Piecewise-Planar Constraints for Dense Monocular Mapping**](http://roboticsproceedings.org/rss10/p16.pdf)

**24. 奥地利格拉茨技术大学计算机图形学与视觉研究所**

.. NOTE::

   + **研究方向**：AR/VR，机器人视觉，机器学习，目标识别与三维重建
   + **实验室主页**：https://www.tugraz.at/institutes/icg/home/
   + 👦 **Friedrich Fraundorfer** 教授：[团队主页](https://www.tugraz.at/institutes/icg/research/team-fraundorfer/) &emsp;[谷歌学术](https://scholar.google.com/citations?user=M0boL5kAAAAJ&hl=zh-CN&oi=sra)
   + 📜 [**Visual Odometry: Part I The First 30 Years and Fundamentals**](http://www.eng.auburn.edu/~troppel/courses/7970%202015A%20AdvMobRob%20sp15/literature/vis%20odom%20tutor%20part1%20.pdf)
   + 📜 [**Visual Odometry: Part II: Matching, Robustness, Optimization, and Applications**](https://www.zora.uzh.ch/id/eprint/71030/1/Fraundorfer_Scaramuzza_Visual_odometry.pdf)
   + 📜 Schenk F, Fraundorfer F. [**RESLAM: A real-time robust edge-based SLAM system**](https://ieeexplore.ieee.org/abstract/document/8794462/)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 154-160.（**代码**：https://github.com/fabianschenk/RESLAM ）
   + 👦 **Dieter Schmalstieg** 教授：[团队主页](https://www.tugraz.at/institutes/icg/research/team-schmalstieg/) &emsp;[谷歌学术](https://scholar.google.com/citations?user=xXu8K6IAAAAJ&hl=zh-CN&oi=ao)
   + 📜 教科书：[Augmented Reality: Principles and Practice](augmentedrealitybook.org)
   + 📜 Arth C, Pirchheim C, Ventura J, et al. [**Instant outdoor localization and slam initialization from 2.5 d maps**](https://ieeexplore.ieee.org/abstract/document/7164332/)[J]. IEEE transactions on visualization and computer graphics, **2015**, 21(11): 1309-1318.
   + 📜 Hachiuma R, Pirchheim C, Schmalstieg D, et al. [**DetectFusion: Detecting and Segmenting Both Known and Unknown Dynamic Objects in Real-time SLAM**](https://arxiv.org/pdf/1907.09127)[J]. arXiv preprint arXiv:1907.09127, **2019**.

**25. 波兰波兹南工业大学移动机器人实验室**

.. NOTE::

   + **研究方向**：SLAM，机器人运动规划，控制
   + **实验室主页**：http://lrm.put.poznan.pl/
   + **Github 主页**：https://github.com/LRMPUT
   + 📜 Wietrzykowski J. [**On the representation of planes for efficient graph-based slam with high-level features**](https://yadda.icm.edu.pl/baztech/element/bwmeta1.element.baztech-7ac7a8f3-9caa-4a34-8a27-8f6c5f43408b)[J]. Journal of Automation Mobile Robotics and Intelligent Systems, **2016**, 10.（**代码**：https://github.com/LRMPUT/PlaneSLAM ）
   + 📜 Wietrzykowski J, Skrzypczyński P. [**PlaneLoc: Probabilistic global localization in 3-D using local planar features**](https://www.sciencedirect.com/science/article/pii/S0921889018303701)[J]. Robotics and Autonomous Systems, **2019**.（**代码**：https://github.com/LRMPUT/PlaneLoc ）
   + 📜 **PUTSLAM**：http://lrm.put.poznan.pl/putslam/

**26. Alexander Vakhitov（三星莫斯科 AI 中心）**

.. NOTE::

   + **研究方向**：SLAM，几何视觉
   + 👦 **个人主页**：https://alexandervakhitov.github.io/ ，[谷歌学术](https://scholar.google.ru/citations?user=g_2iut0AAAAJ&hl=ru%22)
   + 📜 **点线 SLAM**：ICRA 2017 [**PL-SLAM: Real-time monocular visual SLAM with points and lines**](https://upcommons.upc.edu/bitstream/handle/2117/110259/1836-PL-SLAM--Real-Time-Monocular-Visual-SLAM-with-Points-and-Lines.pdf)
   + 📜 **点线定位**：Pumarola A, Vakhitov A, Agudo A, et al. [**Relative localization for aerial manipulation with PL-SLAM**](https://upcommons.upc.edu/bitstream/handle/2117/182388/2205-Relative-localization-for-aerial-manipulation-with-PL-SLAM.pdf)[M]//Aerial Robotic Manipulation. Springer, Cham, **2019**: 239-248.
   + 📜 **学习型线段**：IEEE Access 2019 [**Learnable line segment descriptor for visual SLAM**](https://ieeexplore.ieee.org/iel7/6287639/6514899/08651490.pdf)（**代码**：https://github.com/alexandervakhitov/lld-slam ）

**27. 澳大利亚昆士兰科技大学机器人技术中心**

.. NOTE::

   + **研究方向**：脑启发式机器人，采矿机器人，机器人视觉
   + **实验室主页**：https://www.qut.edu.au/research/centre-for-robotics
   + **开源代码**：https://research.qut.edu.au/qcr/open-source-code/
   + 👦 **Niko Sünderhauf**：[个人主页](https://nikosuenderhauf.github.io/) ，[谷歌学术](https://scholar.google.com/citations?user=WnKjfFEAAAAJ&hl=zh-CN&oi=ao)
   + 📜 RA-L 2018 **二次曲面 SLAM**：[**QuadricSLAM: Dual quadrics from object detections as landmarks in object-oriented SLAM**](https://ieeexplore.ieee.org/abstract/document/8440105/)
   + 📜 Nicholson L, Milford M, Sunderhauf N. [**QuadricSLAM: Dual quadrics as SLAM landmarks**](http://openaccess.thecvf.com/content_cvpr_2018_workshops/papers/w9/Nicholson_QuadricSLAM_Dual_Quadrics_CVPR_2018_paper.pdf)[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition Workshops. **2018**: 313-314.
   + 📜 **Semantic SLAM 项目主页**：http://www.semanticslam.ai/
   + 📜 **IROS 2017**：[**Meaningful maps with object-oriented semantic mapping**](https://arxiv.org/pdf/1609.07849)
   + 👦 **Michael Milford**：谷歌学术 https://scholar.google.com/citations?user=TDSmCKgAAAAJ&hl=zh-CN&oi=ao
   + 📜 **ICRA 2012**：[**SeqSLAM: Visual route-based navigation for sunny summer days and stormy winter nights**](http://www.cim.mcgill.ca/~dudek/417/Resources/seqslam-milford.pdf) （代码：https://michaelmilford.com/seqslam/）
   + 📜 Ball D, Heath S, Wiles J, et al. [**OpenRatSLAM: an open source brain-based SLAM system**](https://static.springer.com/sgw/documents/1388513/application/pdf/10-3.pdf)[J]. Autonomous Robots, **2013**, 34(3): 149-176.（代码：https://openslam-org.github.io/openratslam.html ）
   + 📜 Yu F, Shang J, Hu Y, et al. [**NeuroSLAM: a brain-inspired SLAM system for 3D environments**](https://link.springer.com/article/10.1007/s00422-019-00806-9)[J]. Biological Cybernetics, **2019**, 113(5-6): 515-545. （**代码**：https://github.com/cognav/NeuroSLAM ）

**28. 澳大利亚机器人视觉中心**

.. NOTE::

   + **研究方向**：机器人感知、理解与学习 （集合了昆士兰科技大学，澳大利亚国立大学，阿德莱德大学，昆士兰大学等学校机器人领域的研究者）
   + **实验室主页**：https://www.roboticvision.org/
   + **人物**：https://www.roboticvision.org/rv_person_category/researchers/
   + **发表论文汇总**：https://www.roboticvision.org/publications/scientific-publications/
   + 👦 **Yasir Latif**：[个人主页](http://ylatif.github.io/)，[谷歌学术](https://scholar.google.com/citations?user=pGsO6EkAAAAJ&hl=zh-CN)
   + 📜 Latif Y, Cadena C, Neira J. [**Robust loop closing over time for pose graph SLAM**](http://webdiis.unizar.es/~ylatif/papers/IJRR.pdf)[J]. The International Journal of Robotics Research, **2013**, 32(14): 1611-1626.
   + 📜 Latif Y, Cadena C, Neira J. [**Robust loop closing over time**](https://pdfs.semanticscholar.org/62fb/619f7fc036c4dfb4c55a7c53907a112fe001.pdf)[C]//Proc. Robotics: Science Systems. **2013**: 233-240.（代码：https://github.com/ylatif/rrr ）
   + 👦 **Ian D Reid**：谷歌学术：https://scholar.google.com/citations?user=ATkNLcQAAAAJ&hl=zh-CN&oi=sra
   + 📜 **ICRA 2019**：[**Real-time monocular object-model aware sparse SLAM**](https://arxiv.org/pdf/1809.09149)
   + 📜 Reid I. [**Towards semantic visual SLAM**](https://ieeexplore.ieee.org/abstract/document/7064267/)[C]//2014 13th International Conference on Control Automation Robotics & Vision (ICARCV). IEEE, **2014**: 1-1.

**29. 日本国立先进工业科学技术研究所**

.. NOTE::

   + **人工智能研究中心**：https://www.airc.aist.go.jp/en/intro/
   + 👦 **Ken Sakurada**：[个人主页](https://kensakurada.github.io/)，[谷歌学术](https://scholar.google.com/citations?user=Q4JO-ncAAAAJ&hl=zh-CN&oi=sra)
   + 📜 Sumikura S, Shibuya M, Sakurada K. [**OpenVSLAM: A Versatile Visual SLAM Framework**](https://dl.acm.org/doi/pdf/10.1145/3343031.3350539)[C]//Proceedings of the 27th ACM International Conference on Multimedia. **2019**: 2292-2295.（**代码**：https://github.com/xdspacelab/openvslam ）
   + 👦 **Shuji Oishi**：[谷歌学术](https://scholar.google.com/citations?user=wlPYSDgAAAAJ&hl=zh-CN&oi=sra)
   + 📜 极稠密特征点建图：Yokozuka M, Oishi S, Thompson S, et al. [**VITAMIN-E: visual tracking and MappINg with extremely dense feature points**](http://openaccess.thecvf.com/content_CVPR_2019/papers/Yokozuka_VITAMIN-E_VIsual_Tracking_and_MappINg_With_Extremely_Dense_Feature_Points_CVPR_2019_paper.pdf)[C]//Proceedings of the IEEE conference on computer vision and pattern recognition. **2019**: 9641-9650.
   + 📜 Oishi S, Inoue Y, Miura J, et al. [**SeqSLAM++: View-based robot localization and navigation**](https://staff.aist.go.jp/shuji.oishi/assets/papers/published/SeqSLAM++_RAS2019.pdf)[J]. Robotics and Autonomous Systems, **2019**, 112: 13-21.

**30. Pyojin Kim（韩国首尔大学自主机器人实验室）**

.. NOTE::

   + **研究方向**：视觉里程计，定位，AR/VR
   + 👦 [个人主页](http://pyojinkim.com/)，[谷歌学术](https://scholar.google.com/citations?user=NHpe_8IAAAAJ&hl=en)
   + 📜 **平面 SLAM**：ECCV 2018：[**Linear RGB-D SLAM for planar environments**](http://openaccess.thecvf.com/content_ECCV_2018/papers/Pyojin_Kim_Linear_RGB-D_SLAM_ECCV_2018_paper.pdf)
   + 📜 **光照变化下的鲁棒 SLAM**：ICRA 2017：[**Robust visual localization in changing lighting conditions**](https://www.nasa.gov/sites/default/files/atoms/files/kim2017robust.pdf)
   + 📜 **线面 SLAM**：CVPR 2018：[**Indoor RGB-D Compass from a Single Line and Plane**](http://openaccess.thecvf.com/content_cvpr_2018/papers/Kim_Indoor_RGB-D_Compass_CVPR_2018_paper.pdf)
   + 📜 **博士学位论文**：[**Low-Drift Visual Odometry for Indoor Robotics**](http://pyojinkim.com/download/papers/2019_pjinkim_PhDthesis_low.pdf)

**31. 香港科技大学空中机器人实验室**

.. NOTE::

   + **研究方向**：空中机器人在复杂环境下的自主运行，包括状态估计、建图、运动规划、多机器人协同以及低成本传感器和计算组件的实验平台开发。
   + **实验室主页**：http://uav.ust.hk/
   + **发表论文**：http://uav.ust.hk/publications/ 
   + 👦 沈邵劼教授[**谷歌学术**](https://scholar.google.com/citations?user=u8Q0_xsAAAAJ&hl=zh-CN)
   + **代码公开地址**：https://github.com/HKUST-Aerial-Robotics
   + 📜 Qin T, Li P, Shen S. [**Vins-mono: A robust and versatile monocular visual-inertial state estimator**](https://arxiv.org/pdf/1708.03852.pdf)[J]. IEEE Transactions on Robotics, **2018**, 34(4): 1004-1020.（**代码**：https://github.com/HKUST-Aerial-Robotics/VINS-Mono ）
   + 📜 Wang K, Gao F, Shen S. [**Real-time scalable dense surfel mapping**](https://arxiv.org/pdf/1909.04250)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 6919-6925.（**代码**：https://github.com/HKUST-Aerial-Robotics/DenseSurfelMapping ）

**32. 香港科技大学机器人与多感知实验室 RAM-LAB**

.. NOTE::

   + **研究方向**：无人车；无人船；室内定位；机器学习。
   + **实验室主页**：https://www.ram-lab.com/
   + **发表论文**：https://www.ram-lab.com/publication/ 
   + 👦 刘明教授[**谷歌学术**](https://scholar.google.com/citations?user=CdV5LfQAAAAJ&hl=zh-CN&oi=sra)
   + 📜 Ye H, Chen Y, Liu M. [**Tightly coupled 3d lidar inertial odometry and mapping**](https://arxiv.org/pdf/1904.06993.pdf)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 3144-3150.（**代码**：https://github.com/hyye/lio-mapping ）
   + 📜 Zhang J, Tai L, Boedecker J, et al. [**Neural slam: Learning to explore with external memory**]()[J]. arXiv preprint arXiv:1706.09520, **2017**.

**33. 香港中文大学天石机器人实验室**

.. NOTE::

   + **研究方向**：工业、物流、手术机器人，三维影像，机器学习
   + **实验室主页**：http://ri.cuhk.edu.hk/
   + 👦 **刘云辉教授**：http://ri.cuhk.edu.hk/yhliu
   + 👦 **李浩昂**：[个人主页](https://sites.google.com/view/haoangli/homepage)，[谷歌学术](https://scholar.google.com/citations?user=KnnPc0YAAAAJ&hl=zh-CN&oi=sra)
   + 📜 Li H, Yao J, Bazin J C, et al. [**A monocular SLAM system leveraging structural regularity in Manhattan world**](http://cvrs.whu.edu.cn/projects/Struct-PL-SLAM/source/file/Struct_PL_SLAM.pdf)[C]//2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2018**: 2518-2525.
   + 📜 Li H, Yao J, Lu X, et al. [**Combining points and lines for camera pose estimation and optimization in monocular visual odometry**](https://ieeexplore.ieee.org/abstract/document/8202304/)[C]//2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2017**: 1289-1296.
   + 📜 消失点检测：Lu X, Yaoy J, Li H, et al. [**2-Line Exhaustive Searching for Real-Time Vanishing Point Estimation in Manhattan World**](https://www.computer.org/csdl/proceedings/wacv/2017/4822/00/07926628.pdf)[C]//Applications of Computer Vision (WACV), 2017 IEEE Winter Conference on. IEEE, **2017**: 345-353.（代码：https://github.com/xiaohulugo/VanishingPointDetection ）
   + 👦 **郑帆**：[个人主页](https://fzheng.me/cnabout/)，[谷歌学术](https://scholar.google.com/citations?user=PZOTyfIAAAAJ&hl=zh-CN&oi=sra)
   + 📜 Zheng F, Tang H, Liu Y H. [**Odometry-vision-based ground vehicle motion estimation with se (2)-constrained se (3) poses**](https://ieeexplore.ieee.org/abstract/document/8357438/)[J]. IEEE transactions on cybernetics, **2018**, 49(7): 2652-2663.（代码：https://github.com/izhengfan/se2clam ）
   + 📜 Zheng F, Liu Y H. [**Visual-Odometric Localization and Mapping for Ground Vehicles Using SE (2)-XYZ Constraints**](https://ieeexplore.ieee.org/abstract/document/8793928/)[C]//2019 International Conference on Robotics and Automation (ICRA). IEEE, **2019**: 3556-3562.（代码：https://github.com/izhengfan/se2lam ）

**34. 浙江大学 CAD&CG 国家重点实验室**

.. NOTE::

   + **研究方向**：SFM/SLAM，三维重建，增强现实
   + **实验室主页**：http://www.zjucvg.net/
   + **Github 代码地址**：https://github.com/zju3dv
   + 👦 **章国峰教授**：[个人主页](http://www.cad.zju.edu.cn/home/gfzhang/)，[谷歌学术](https://scholar.google.com/citations?user=F0xfpXAAAAAJ&hl=zh-CN&oi=sra)
   + 📜 **ICE-BA**：Liu H, Chen M, Zhang G, et al. [**Ice-ba: Incremental, consistent and efficient bundle adjustment for visual-inertial slam**](http://openaccess.thecvf.com/content_cvpr_2018/papers/Liu_ICE-BA_Incremental_Consistent_CVPR_2018_paper.pdf)[C]//Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition. **2018**: 1974-1982.（代码：https://github.com/zju3dv/EIBA ）
   + 📜 **RK-SLAM**：Liu H, Zhang G, Bao H. [**Robust keyframe-based monocular SLAM for augmented reality**](https://ieeexplore.ieee.org/abstract/document/7781760/)[C]//2016 IEEE International Symposium on Mixed and Augmented Reality (ISMAR). IEEE, **2016**: 1-10.（项目主页：http://www.zjucvg.net/rkslam/rkslam.html ）
   + 📜 **RD-SLAM**：Tan W, Liu H, Dong Z, et al. [**Robust monocular SLAM in dynamic environments**](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.431.8137&rep=rep1&type=pdf)[C]//2013 IEEE International Symposium on Mixed and Augmented Reality (ISMAR). IEEE, **2013**: 209-218.

**35. 邹丹平（上海交通大学）**

.. NOTE::

   + **研究方向**：视觉 SLAM，SFM，多源导航，微型无人机
   + 👦 **个人主页**：http://drone.sjtu.edu.cn/dpzou/index.php ， [谷歌学术](https://scholar.google.com/citations?user=y6FsLDQAAAAJ&hl=en&oi=ao)
   + 📜 **Co-SLAM**：Zou D, Tan P. [**Coslam: Collaborative visual slam in dynamic environments**](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.463.8135&rep=rep1&type=pdf)[J]. IEEE transactions on pattern analysis and machine intelligence, **2012**, 35(2): 354-366.（**代码**：https://github.com/danping/CoSLAM ）
   + 📜 **StructSLAM**：Zhou H, Zou D, Pei L, et al. [**StructSLAM: Visual SLAM with building structure lines**]()[J]. IEEE Transactions on Vehicular Technology, **2015**, 64(4): 1364-1375.（**项目主页**：http://drone.sjtu.edu.cn/dpzou/project/structslam.php ）
   + 📜 **StructVIO**：Zou D, Wu Y, Pei L, et al. [**StructVIO: visual-inertial odometry with structural regularity of man-made environments**](https://arxiv.org/pdf/1810.06796)[J]. IEEE Transactions on Robotics, **2019**, 35(4): 999-1013.

**36. 布树辉教授（西北工业大学智能系统实验室**

.. NOTE::

   + **研究方向**：语义定位与建图、SLAM、在线学习与增量学习
   + 👦 **个人主页**：http://www.adv-ci.com/blog/ &emsp; [谷歌学术](https://scholar.google.com/citations?user=spwZ6b4AAAAJ&hl=zh-CN&oi=ao)
   + **布老师的课件**：http://www.adv-ci.com/blog/course/
   + 实验室 2018 年暑期培训资料：https://github.com/zdzhaoyong/SummerCamp2018
   + 📜 **开源的通用 SLAM 框架**：Zhao Y, Xu S, Bu S, et al. [**GSLAM: A general SLAM framework and benchmark**](http://openaccess.thecvf.com/content_ICCV_2019/papers/Zhao_GSLAM_A_General_SLAM_Framework_and_Benchmark_ICCV_2019_paper.pdf)[C]//Proceedings of the IEEE International Conference on Computer Vision. **2019**: 1110-1120.（**代码**：https://github.com/zdzhaoyong/GSLAM ）
   + 📜 Bu S, Zhao Y, Wan G, et al. [**Map2DFusion: Real-time incremental UAV image mosaicing based on monocular slam**](http://www.adv-ci.com/publications/2016_IROS.pdf)[C]//2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2016**: 4564-4571.（**代码**：https://github.com/zdzhaoyong/Map2DFusion ）
   + 📜 Wang W, Zhao Y, Han P, et al. [**TerrainFusion: Real-time Digital Surface Model Reconstruction based on Monocular SLAM**](https://ieeexplore.ieee.org/abstract/document/8967663/)[C]//2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2019**: 7895-7902.

**+1 Cyrill Stachniss（德国波恩大学摄影测量与机器人实验室）**

.. NOTE::

   + **研究方向**：概率机器人、SLAM、自主导航、视觉激光感知、场景分析与分配、无人飞行器
   + **实验室主页**：https://www.ipb.uni-bonn.de/
   + 👦 **个人主页**：https://www.ipb.uni-bonn.de/people/cyrill-stachniss/ [谷歌学术](https://scholar.google.com/citations?user=8vib2lAAAAAJ&hl=zh-CN&authuser=1&oi=ao)
   + 发表论文：https://www.ipb.uni-bonn.de/publications/
   + 开源代码：https://github.com/PRBonn
   + 📜 IROS 2019 激光语义 SLAM：Chen X, Milioto A, Palazzolo E, et al. [**SuMa++: Efficient LiDAR-based semantic SLAM**](https://ieeexplore.ieee.org/abstract/document/8967704/)[C]//2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, **2019**: 4530-4537.（代码：https://github.com/PRBonn/semantic_suma/ ）
   + Cyrill Stachniss 教授 SLAM 公开课：[youtube](https://www.youtube.com/watch?v=4QG0y0pIOBE&list=PLgnQpQtFTOGQh_J16IMwDlji18SWQ2PZ6) ； [bilibili](https://space.bilibili.com/16886998/channel/detail?cid=118821)
   + 波恩大学另外一个**智能自主系统实验室**：http://www.ais.uni-bonn.de/research.html

**+1 上海科技大学**

.. NOTE::

   + **Mobile Perception Lab**：http://mpl.sist.shanghaitech.edu.cn/
   + 👦 Laurent Kneip：[个人主页](https://www.laurentkneip.com/)；[谷歌学术](https://scholar.google.com.au/citations?user=lTmh1e0AAAAJ&hl=en)
   + 📜 Zhou Y, Li H, Kneip L. [**Canny-vo: Visual odometry with rgb-d cameras based on geometric 3-d–2-d edge alignment**](https://ieeexplore.ieee.org/abstract/document/8510917/)[J]. IEEE Transactions on Robotics, **2018**, 35(1): 184-199.
   + **自主移动机器人实验室**：https://robotics.shanghaitech.edu.cn/zh
   + 👦 Sören Schwertfeger：[个人主页](https://robotics.shanghaitech.edu.cn/zh/people/soeren)；[谷歌学术](https://scholar.google.com.au/citations?user=Y2olJ9kAAAAJ&hl=en&oi=ao)
   + 📜 Shan Z, Li R, Schwertfeger S. [**RGBD-Inertial Trajectory Estimation and Mapping for Ground Robots**](https://link.zhihu.com/?target=https%3A//www.mdpi.com/1424-8220/19/10/2251)[J]. Sensors, **2019**, 19(10): 2251.（代码：https://github.com/STAR-Center/VINS-RGBD ）

**+1 美国密歇根大学机器人研究所**

.. NOTE::

   + **学院官网**：https://robotics.umich.edu/
   + **研究方向**：https://robotics.umich.edu/research/focus-areas/
   + **感知机器人实验室（PeRL）**
   + 实验室主页：http://robots.engin.umich.edu/About/
   + 👦 **Ryan M. Eustice** [谷歌学术](https://scholar.google.com/citations?user=WroYmiAAAAAJ&hl=en&oi=ao)
   + 📜 激光雷达数据集 Pandey G, McBride J R, Eustice R M. [**Ford campus vision and lidar data set**](https://journals.sagepub.com/doi/abs/10.1177/0278364911400640)[J]. The International Journal of Robotics Research, **2011**, 30(13): 1543-1552. | [数据集](http://robots.engin.umich.edu/SoftwareData/Ford)
   + **APRIL robotics lab**
   + 实验室主页：https://april.eecs.umich.edu/
   + 👦 **Edwin Olson** [个人主页](https://april.eecs.umich.edu/people/ebolson/) | [谷歌学术](https://scholar.google.com/citations?user=GwtVjKYAAAAJ&hl=en&oi=ao)
   + 📜 Olson E. [**AprilTag: A robust and flexible visual fiducial system**](https://april.eecs.umich.edu/pdfs/olson2010tags.pdf)[C]//2011 IEEE International Conference on Robotics and Automation. IEEE, **2011**: 3400-3407. | [**代码**](https://github.com/AprilRobotics/apriltag)
   + 📜 Wang X, Marcotte R, Ferrer G, et al. [**ApriISAM: Real-time smoothing and mapping**](https://april.eecs.umich.edu/papers/details.php?name=wang2018aprilsam)[C]//2018 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2018**: 2486-2493. | [**代码**](https://github.com/xipengwang/AprilSAM)

**+1 瑞士苏黎世联邦理工自主系统实验室**

.. NOTE::

   + **研究方向**：复杂多样环境中自主运行的机器人和智能系统
   + **实验室主页**：https://asl.ethz.ch/
   + 发表论文：https://asl.ethz.ch/publications-and-sources/publications.html
   + [youtube](https://www.youtube.com/channel/UCgqwlRBPdDL2k4OWvH6Oppg) | [Github](https://github.com/ethz-asl)
   + 👦 **Cesar Cadena** [个人主页](http://n.ethz.ch/~cesarc/)
   + 📜 Schneider T, Dymczyk M, Fehr M, et al. [**maplab: An open framework for research in visual-inertial mapping and localization**](https://arxiv.org/pdf/1711.10250)[J]. IEEE Robotics and Automation Letters, **2018**, 3(3): 1418-1425. | [**代码**](https://github.com/ethz-asl/maplab)
   + 📜 Dubé R, Cramariuc A, Dugas D, et al. [**SegMap: 3d segment mapping using data-driven descriptors**](https://arxiv.org/pdf/1804.09557)[J]. arXiv preprint arXiv:1804.09557, **2018**. | [**代码**](https://github.com/ethz-asl/segmap)
   + 📜 Millane A, Taylor Z, Oleynikova H, et al. [**C-blox: A scalable and consistent tsdf-based dense mapping approach**](https://arxiv.org/pdf/1710.07242)[C]//2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018: 995-1002. | [**代码**](https://github.com/ethz-asl/cblox)

**+1 美国麻省理工学院 Robust Robotics Group**

.. NOTE::

   + **研究方向**：MAV 导航与控制；人机交互的自然语言理解；自主海洋机器人的语义理解
   + **实验室主页**：http://groups.csail.mit.edu/rrg/index.php
   + 👦 Nicholas Roy：[Google Scholar](https://scholar.google.com/citations?user=aM3i_9oAAAAJ&hl=zh-CN&oi=ao)
   + 📜 Greene W N, Ok K, Lommel P, et al. [**Multi-level mapping: Real-time dense monocular SLAM**](https://ieeexplore.ieee.org/abstract/document/7487213/)[C]//2016 IEEE International Conference on Robotics and Automation (**ICRA**). IEEE, **2016**: 833-840. [video](https://www.youtube.com/watch?v=qk2ViPVxmq0&feature=youtu.be)
   + 📜 ICRA 2020 [Metrically-Scaled Monocular SLAM using Learned Scale Factors." International Conference on Robotics and Automation](http://groups.csail.mit.edu/rrg/papers/greene_icra20.pdf) | [video](https://www.youtube.com/watch?v=qk2ViPVxmq0&feature=youtu.be)       
   + 📜 ICRA 2019 [Robust Object-based SLAM for High-speed Autonomous Navigation](http://groups.csail.mit.edu/rrg/papers/OkLiu19icra.pdf)

**+1 瑞士苏黎世联邦理工 Vision for Robotics Lab**

.. NOTE::

   + **研究方向**：机器人视觉，无人机，自主导航，多机器人协同
   + **实验室主页**：https://v4rl.ethz.ch/the-group.html
   + 👦 **Margarita Chli**：[个人主页](http://www.margaritachli.com/)  | [Google Scholar](https://scholar.google.com/citations?user=C0UhwEIAAAAJ&hl=zh-CN&oi=ao)
   + 📜 Schmuck P, Chli M. [**CCM‐SLAM: Robust and efficient centralized collaborative monocular simultaneous localization and mapping for robotic teams**](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21854)[J]. Journal of Field Robotics, **2019**, 36(4): 763-781. [**code**](https://github.com/VIS4ROB-lab/ccm_slam) | [video](https://www.youtube.com/watch?v=P3b7UiTlmbQ&feature=youtu.be)       
   + 📜 Bartolomei L, Karrer M, Chli M. [**Multi-robot Coordination with Agent-Server Architecture for Autonomous Navigation in Partially Unknown Environments**](https://www.research-collection.ethz.ch/handle/20.500.11850/441280)[C]//IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2020)(virtual). **2020**. [**code**](https://github.com/VIS4ROB-lab/multi_robot_coordination) | [video](https://www.youtube.com/watch?v=ATQiTsbaSOw)       
   + 📜 Schmuck P, Chli M. [**Multi-uav collaborative monocular slam**](https://ieeexplore.ieee.org/abstract/document/7989445/)[C]//2017 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2017**: 3863-3870.

**+1 谢立华教授（南洋理工大学**

.. NOTE::

   + **研究方向**：控制，多智能体，定位
   + **个人主页**：https://personal.ntu.edu.sg/elhxie/research.html | [Google Scholar](https://scholar.google.com.hk/citations?user=Fmrv3J8AAAAJ&hl=zh-CN&oi=ao)
   + 👦 **Wang Han**：[个人主页](https://wanghan.pro/)  | [Github](https://github.com/wh200720041)
   + 📜 Wang H, Wang C, Xie L. [**Intensity scan context: Coding intensity and geometry relations for loop closure detection**](https://ieeexplore.ieee.org/abstract/document/9196764/)[C]//2020 IEEE International Conference on Robotics and Automation (ICRA). IEEE, **2020**: 2095-2101. | [Code](https://github.com/wh200720041/iscloam)
   + 📜 Wang H, Wang C, Xie L. [**Lightweight 3-D Localization and Mapping for Solid-State LiDAR**](https://ieeexplore.ieee.org/abstract/document/9357899/)[J]. IEEE Robotics and Automation Letters, **2021**, 6(2): 1801-1807. | [Code](https://github.com/wh200720041/ssl_slam)
   + 📜 Wang C, Yuan J, Xie L. [**Non-iterative SLAM**](https://ieeexplore.ieee.org/abstract/document/8023500/)[C]//2017 18th International Conference on Advanced Robotics (ICAR). IEEE, **2017**: 83-90.


**3.SLAM 学习资料**

1 国内资料
----------

.. NOTE::

   + **1)** SLAMcn：http://www.slamcn.org/index.php/
   + **2)** SLAM 最新研究更新 Recent_SLAM_Research ：https://github.com/YiChenCityU/Recent_SLAM_Research
   + **3)** **西北工大智能系统实验室 SLAM 培训**：https://github.com/zdzhaoyong/SummerCamp2018
   + 布树辉老师课件：http://www.adv-ci.com/blog/course/
   + **4)** IROS 2019 **视觉惯导导航**的挑战与应用研讨会：http://udel.edu/~ghuang/iros19-vins-workshop/index.html
   + **5)** 泡泡机器人 **VIO** 相关资料：https://github.com/PaoPaoRobot/Awesome-VIO
   + **6)** 崔华坤：主流 **VIO 论文推导及代码解析**：https://github.com/StevenCui/VIO-Doc
   + **7)** 李言：[SLAM 中的几何与学习方法](https://github.com/yanyan-li/SLAM-BOOK)
   + **8)** 黄山老师状态估计视频：[bilibili](https://www.bilibili.com/video/av66258275)
   + **9)** 谭平老师-SLAM 6小时课程：[bilibili](https://www.bilibili.com/video/BV1v4411p735)
   + **10)** 2020 年 SLAM 技术及应用暑期学校：[视频-bilibili](https://www.bilibili.com/video/BV1Hf4y1X7P5/) | [课件](http://www.cad.zju.edu.cn/home/gfzhang/download/2020-SLAM-Summer-School-slides.zip)

2 国外资料
---------------

.. NOTE::

   + **1)** **事件相机**相关研究与发展：https://github.com/uzh-rpg/event-based_vision_resources
   + **2)** 加州大学圣地亚哥分校**语境机器人研究所** Nikolay Atanasov 教授**机器人状态估计与感知课程** ppt：https://natanaso.github.io/ece276a2019/schedule.html
   + **3)** 波恩大学 **Mobile Sensing and Robotics Course** 公开课 ：[youtube](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQJXx-x0t23RmRbjp_yMb4v) ，[bilibili](https://space.bilibili.com/16886998/channel/detail?cid=118821)

3 公众号
----------

.. NOTE::

   + **泡泡机器人 SLAM**：paopaorobot_slam

4 代码注释
----------

.. NOTE::

   今天（2020.04.25）刚想到的一个点，就算前面整理了大量的开源工作，但是看原版的代码还是会有很大的困难，**感谢国内 SLAM 爱好者的将自己的代码注释分享出来，促进交流，共同进步**。这一小节的内容陆续发掘，期待大家的推荐代码注释（可以在 issue 中留言）。

5 数据集
----------

.. NOTE::

   + [泡泡机器人 - SLAM 数据集合集](https://mp.weixin.qq.com/s/zRjwus68Kf4unIqPIubraw)
   + [计算机视觉life - SLAM、重建、语义相关数据集大全](https://zhuanlan.zhihu.com/p/68294012)
   + [水下 SLAM 相关研究 - 代码、数据集](http://note.youdao.com/s/GjCXvWFR)   

**4.Optimization**

   * 后端优化库 `GTSAM <https://github.com/borglab/gtsam>`_ `g2o <https://github.com/RainerKuemmerle/g2o>`_ `ceres <http://ceres-solver.org/>`_
   * `ICE-BA <https://github.com/baidu/ICE-BA>`_
   * 因子图最小二乘优化框架 `minisam <https://github.com/dongjing3309/minisam>`_
   * 几何基元图优化 `SA-SHAGO <https://srrg.gitlab.io/sashago-website/index.html#>`_
   * SLAM优化器 `MH-iSAM2 <https://bitbucket.org/rpl_cmu/mh-isam2_lib/src/master/>`_
   * 用于定位和建图的模块化优化框架  `MOLA <https://github.com/MOLAorg/mola>`_

开源公告
==========

   * 既授人以鱼，亦授人以渔
   * 鱼渔皆俱，实则授业解惑也
   * 愿众薪拾火，成大业之道不久矣


Contact Me
==========

If you use learn more xslam knowledge, please contact me::

    author: Du Yongquan
    email: quandy2020@126.com


哔哩哔哩
========

`哔哩哔哩 <https://space.bilibili.com/478832908>`_ 视频网站上同步更新。


曾国藩语录
============
躬身入局挺膺负责，方有成事之可冀

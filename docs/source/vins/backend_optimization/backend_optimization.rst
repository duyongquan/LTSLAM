.. highlight:: c++

.. default-domain:: cpp

====================
Backend Optimization
====================

状态向量


目标函数:

.. math::

    \min_{\small{\chi}}
        \left \{
                {||r_{P} - H_{P}\chi||^{2} + 
                \sum_{k \in B}{|| r_{B}(\hat{z}_{b_{k+1}}^{b_k})||}_{p_{b_{k+1}}^{b_k}}^{2} + 
                \sum_{(l, j)\in{C}} ||r_C(\hat{z}_{l}^{c_j}), \chi)||_{P_{l}^{c_j}}^{2}}
        \right \}
    
IMU测量约束 

视觉测量约束
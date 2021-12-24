.. highlight:: c++

.. default-domain:: cpp

.. _chapter-ceres_solver_tutorial:

=====================
Ceres Solver Tutorial
=====================

.. _section-hello-world:

Hello World!
============

求取函数最小值：

.. math:: \frac{1}{2}(10 -x)^2.

demo源码 [#f1]_ [#f2]_ [#f3]_ :

.. code-block:: c++

    struct CostFunctor 
    {
        template <typename T>
        bool operator()(const T* const x, T* residual) const 
        {
            residual[0] = T(10.0) - x[0];
            return true;
        }
    };


.. code-block:: c++

    // f(x) = 10 - x
    void HelloWorld::RunDemo()
    {
        // The variable to solve for with its initial value. It will be
        // mutated in place by the solver.
        double x = 0.5;
        const double initial_x = x;

        // Build the problem.
        ::ceres::Problem problem;


        // Set up the only cost function (also known as residual). This uses
        // auto-differentiation to obtain the derivative (jacobian).
        ::ceres::CostFunction* cost_function =
                new ::ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
        problem.AddResidualBlock(cost_function, nullptr, &x);

        // Run the solver!
        ::ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        ::ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "x : " << initial_x << " -> " << x << "\n";
    }


.. code-block:: bash

    iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
      0  4.512500e+01    0.00e+00    9.50e+00   0.00e+00   0.00e+00  1.00e+04       0    5.33e-04    3.46e-03
      1  4.511598e-07    4.51e+01    9.50e-04   9.50e+00   1.00e+00  3.00e+04       1    5.00e-04    4.05e-03
      2  5.012552e-16    4.51e-07    3.17e-08   9.50e-04   1.00e+00  9.00e+04       1    1.60e-05    4.09e-03
    Ceres Solver Report: Iterations: 2, Initial cost: 4.512500e+01, Final cost: 5.012552e-16, Termination: CONVERGENCE
    x : 0.5 -> 10

.. rubric:: Footnotes

.. [#f1] `hello_world.cpp
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/ceres/hello_world.cpp>`_
.. [#f2] `hello_world.h
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/ceres/hello_world.h>`_
.. [#f3] `hello_world_test.cpp
    <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/ceres/hello_world_test.cpp>`_


Curve Fitting
=============

曲线函数 :

.. math::  y = e^{mx + c}.

demo源码 [#f4]_ [#f5]_ [#f6]_ :

.. code-block:: c++

    struct ExponentialResidual 
    {
    public:
        ExponentialResidual(double x, double y)
            : x_(x), y_(y) {}

        template <typename T>
        bool operator()(const T* const m, const T* const c, T* residual) const 
        {
            residual[0] = T(y_) - exp(m[0] * T(x_) + c[0]);
            return true;
        }

    private:
        // Observations for a sample.
        const double x_;
        const double y_;
    };


.. code-block:: c++

    void CurveFitting::RunDemo()
    {
        double m = 0.0;
        double c = 0.0;
        ::ceres::Problem problem;
        for (int i = 0; i < kNumObservations; ++i)
        {
            problem.AddResidualBlock(
                    new ::ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                            new ExponentialResidual(data[2 * i], data[2 * i + 1])),
                    NULL,
                    &m,
                    &c);
        }

        ::ceres::Solver::Options options;
        options.max_num_iterations = 25;
        // QR Ax= b
        options.linear_solver_type = ::ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        // Result
        ::ceres::Solver::Summary summary;
        ::ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
        std::cout << "Final   m: " << m << " c: " << c << "\n";
    }


.. figure:: ./images/least_squares_fit.png
   :figwidth: 500px
   :height: 400px
   :align: center

   Least squares curve fitting.

.. rubric:: Footnotes

.. [#f4] `curve_fitting.cpp
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/ceres/curve_fitting.cpp>`_
.. [#f5] `curve_fitting.h
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/ceres/curve_fitting.h>`_
.. [#f6] `curve_fittingtest.cpp
    <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/ceres/curve_fitting_test.cpp>`_






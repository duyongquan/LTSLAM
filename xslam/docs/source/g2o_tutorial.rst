.. highlight:: c++

.. default-domain:: cpp

.. _chapter-g2o_tutorial:

============
G2O Tutorial
============

Circle Fit
==========

.. _section-circle_fit:

circle function：

.. math:: 

    (x - a)^2 + (y - b)^2 = r^2

circle极坐标的方程表示:

.. math:: 

    \begin{cases}
        x = x_0 + r\cos{\theta} \\
        y = y_0 + r\sin{\theta}
    \end{cases}

demo源码 [#f1]_ [#f2]_ [#f3]_ :

VertexCircle :

.. code-block:: c++

    /**
     * A circle located at x,y with radius r
     */
    class VertexCircle : public ::g2o::BaseVertex<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexCircle(){}

        virtual bool read(std::istream& /*is*/);
        virtual bool write(std::ostream& /*os*/) const;

        virtual void setToOriginImpl();
        virtual void oplusImpl(const double* update);
    };


EdgePointOnCircle :

.. code-block:: c++

    /**
     * measurement for a point on the circle
     *
     * Here the measurement is the point which is on the circle.
     * The error function computes the distance of the point to
     * the center minus the radius of the circle.
     */
    class EdgePointOnCircle : public ::g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgePointOnCircle() {}

        virtual bool read(std::istream & /*is*/);
        virtual bool write(std::ostream & /*os*/) const;

        void computeError();
    };

.. code-block:: c++

    void CircleFit::RunDemo()
    {
        int numPoints = 100;
        int maxIterations = 10;
        bool verbose = true;

        // --1 generate random data
        Eigen::Vector2d center(4.0, 2.0);
        double radius = 2.0;
        Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];

        ::g2o::Sampler::seedRand();
        for (int i = 0; i < numPoints; ++i) {
            double r = ::g2o::Sampler::gaussRand(radius, 0.05);
            double angle = ::g2o::Sampler::uniformRand(0.0, 2.0 * M_PI);
            points[i].x() = center.x() + r * cos(angle);
            points[i].y() = center.y() + r * sin(angle);
        }

        // --2 setup the solver
        ::g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        ::g2o::OptimizationAlgorithmLevenberg* solver = new ::g2o::OptimizationAlgorithmLevenberg(
                ::g2o::make_unique<MyBlockSolver>(::g2o::make_unique<MyLinearSolver>()));
        optimizer.setAlgorithm(solver);

        // --3 build the optimization problem given the points
        // 3.1. add the circle vertex
        VertexCircle* circle = new VertexCircle();
        circle->setId(0);
        circle->setEstimate(Eigen::Vector3d(3.0, 3.0, 3.0)); // some initial value for the circle
        optimizer.addVertex(circle);
        // 3.2. add the points we measured
        for (int i = 0; i < numPoints; ++i) {
            EdgePointOnCircle* e = new EdgePointOnCircle;
            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
            e->setVertex(0, circle);
            e->setMeasurement(points[i]);
            optimizer.addEdge(e);
        }

        // --4 perform the optimization
        optimizer.initializeOptimization();
        optimizer.setVerbose(verbose);
        optimizer.optimize(maxIterations);

        if (verbose) {
            std::cout << std::endl;
        }


        // --5 print out the result
        std::cout << "Iterative least squares solution" << std::endl;
        std::cout << "center of the circle " << circle->estimate().head<2>().transpose() << std::endl;
        std::cout << "radius of the cirlce " << circle->estimate()(2) << std::endl;
        std::cout << "error " << ErrorOfSolution(numPoints, points, circle->estimate()) << std::endl;
        std::cout << std::endl;
        
        // clean up
        delete[] points;
    }

.. rubric:: Footnotes

.. [#f1] `circle_fit.cpp
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/g2o/circle_fit.cpp>`_
.. [#f2] `circle_fit.h
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/g2o/circle_fit.h>`_
.. [#f3] `circle_fit_test.cpp
    <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/g2o/circle_fit_test.cpp>`_



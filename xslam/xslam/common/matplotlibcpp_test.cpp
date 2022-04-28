#include "xslam/common/matplotlibcpp.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

#include <cmath>
#include <vector>

namespace xslam {
namespace g2o {

namespace plt = matplotlibcpp;

TEST(matplotlibcpp, contour)
{
    std::vector<std::vector<double>> x, y, z;
    for (double i = -5; i <= 5;  i += 0.25) 
    {
        std::vector<double> x_row, y_row, z_row;
        for (double j = -5; j <= 5; j += 0.25) {
            x_row.push_back(i);
            y_row.push_back(j);
            z_row.push_back(::std::sin(::std::hypot(i, j)));
        }
        x.push_back(x_row);
        y.push_back(y_row);
        z.push_back(z_row);
    }

    plt::xlim(-10, 10);
    plt::ylim(-10, 10);
    matplotlibcpp::contour(x, y, z);
    matplotlibcpp::show();
}

TEST(matplotlibcpp, colorbar)
{
    // Prepare data
    int ncols = 500, nrows = 300;
    std::vector<float> z(ncols * nrows);
    for (int j=0; j<nrows; ++j) {
        for (int i=0; i<ncols; ++i) {
            z.at(ncols * j + i) = std::sin(std::hypot(i - ncols/2, j - nrows/2));
        }
    }

    const float* zptr = &(z[0]);
    const int colors = 1;

    plt::title("My matrix");
    PyObject* mat;
    plt::imshow(zptr, nrows, ncols, colors, {}, &mat);
    plt::colorbar(mat);

    // Show plots
    plt::show();
    plt::close();
    Py_DECREF(mat);
}

TEST(matplotlibcpp, basic)
{
    // Prepare data.
    int n = 5000;
    std::vector<double> x(n), y(n), z(n), w(n,2);
    for(int i=0; i<n; ++i) {
        x.at(i) = i*i;
        y.at(i) = sin(2*M_PI*i/360.0);
        z.at(i) = log(i);
    }
    
    // Set the size of output image = 1200x780 pixels
    plt::figure_size(1200, 780);

    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(x, y);

    // Plot a red dashed line from given x and y data.
    plt::plot(x, w,"r--");

    // Plot a line whose name will show up as "log(x)" in the legend.
    plt::named_plot("log(x)", x, z);

    // Set x-axis to interval [0,1000000]
    plt::xlim(0, 1000*1000);

    // Add graph title
    plt::title("Sample figure");

    // Enable legend.
    plt::legend();

    // save figure
    const char* filename = "./basic.png";
    std::cout << "Saving result to " << filename << std::endl;;
    plt::save(filename);
}

TEST(matplotlibcpp, bar)
{
    std::vector<int> test_data;
    for (int i = 0; i < 20; i++) {
        test_data.push_back(i);
    }

    plt::bar(test_data);
    plt::show();
}

TEST(matplotlibcpp, animation)
{
    int n = 1000;
    std::vector<double> x, y, z;

    for(int i=0; i<n; i++) {
      x.push_back(i*i);
      y.push_back(sin(2*M_PI*i/360.0));
      z.push_back(log(i));

      if (i % 10 == 0) {
        // Clear previous plot
        plt::clf();
        // Plot line from given x and y data. Color is selected automatically.
        plt::plot(x, y);
        // Plot a line whose name will show up as "log(x)" in the legend.
        plt::named_plot("log(x)", x, z);

        // Set x-axis to interval [0,1000000]
        plt::xlim(0, n*n);

        // Add graph title
        plt::title("Sample figure");
        // Enable legend.
        plt::legend();
        // Display plot continuously
        plt::pause(0.01);
      }
    }
}

TEST(matplotlibcpp, quiver)
{
    // u and v are respectively the x and y components of the arrows we're plotting
    std::vector<int> x, y, u, v;
    for (int i = -5; i <= 5; i++) {
        for (int j = -5; j <= 5; j++) {
            x.push_back(i);
            u.push_back(-i);
            y.push_back(j);
            v.push_back(-j);
        }
    }

    plt::quiver(x, y, u, v);
    plt::show();
}

TEST(matplotlibcpp, spy)
{
    const int n = 20;
    std::vector<std::vector<double>> matrix;

    for (int i = 0; i < n; ++i) {
        std::vector<double> row;
        for (int j = 0; j < n; ++j) {
            if (i == j)
                row.push_back(-2);
            else if (j == i - 1 || j == i + 1)
                row.push_back(1);
            else
                row.push_back(0);
        }
        matrix.push_back(row);
    }

    plt::spy(matrix, 5, {{"marker", "o"}});
    plt::show();
}

TEST(matplotlibcpp, subplot)
{
    // Prepare data
    int n = 500;
    std::vector<double> x(n), y(n), z(n), w(n,2);
    for(int i=0; i<n; ++i) {
      x.at(i) = i;
      y.at(i) = sin(2*M_PI*i/360.0);
      z.at(i) = 100.0 / i;
    }

    // Set the "super title"
    plt::suptitle("My plot");
    plt::subplot(1, 2, 1);
	  plt::plot(x, y, "r-");
    plt::subplot(1, 2, 2);
    plt::plot(x, z, "k-");
    // Add some text to the plot
    plt::text(100, 90, "Hello!");
    plt::show();
}

TEST(matplotlibcpp, subplot2grid)
{
     // Prepare data
    int n = 500;
    std::vector<double> x(n), u(n), v(n), w(n);
    for(int i=0; i<n; ++i) {
      x.at(i) = i;
      u.at(i) = sin(2*M_PI*i/500.0);
      v.at(i) = 100.0 / i;
      w.at(i) = sin(2*M_PI*i/1000.0);
    }

    // Set the "super title"
    plt::suptitle("My plot");

    const long nrows=3, ncols=3;
    long row = 2, col = 2;

    plt::subplot2grid(nrows, ncols, row, col);
	  plt::plot(x, w, "g-");

    long spanr = 1, spanc = 2;
    col = 0;
    plt::subplot2grid(nrows, ncols, row, col, spanr, spanc);
	  plt::plot(x, v, "r-");

    spanr = 2, spanc = 3;
    row = 0, col = 0;
    plt::subplot2grid(nrows, ncols, row, col, spanr, spanc);
    plt::plot(x, u, "b-");
    // Add some text to the plot
    plt::text(100., -0.5, "Hello!");


    // Show plots
	  plt::show();
}

TEST(matplotlibcpp, surface)
{
    std::vector<std::vector<double>> x, y, z;
    for (double i = -5; i <= 5;  i += 0.25) {
        std::vector<double> x_row, y_row, z_row;
        for (double j = -5; j <= 5; j += 0.25) {
            x_row.push_back(i);
            y_row.push_back(j);
            z_row.push_back(::std::sin(::std::hypot(i, j)));
        }
        x.push_back(x_row);
        y.push_back(y_row);
        z.push_back(z_row);
    }

    plt::plot_surface(x, y, z);
    plt::show();
}

TEST(matplotlibcpp, xkcd)
{
    std::vector<double> t(1000);
    std::vector<double> x(t.size());

    for(size_t i = 0; i < t.size(); i++) {
        t[i] = i / 100.0;
        x[i] = sin(2.0 * M_PI * 1.0 * t[i]);
    }

    plt::xkcd();
    plt::plot(t, x);
    plt::title("AN ORDINARY SIN WAVE");
    plt::show();
}

} // namespace g2o
} // namespace xslam


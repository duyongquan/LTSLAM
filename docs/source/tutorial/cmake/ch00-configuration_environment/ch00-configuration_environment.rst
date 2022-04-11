.. highlight:: c++

.. default-domain:: cpp


===============
第0章 配置环境
===============


0.1 获取代码
----------------

为了测试源码，需要使用Git获取代码

.. code-block:: bash

    git clone https://github.com/dev-cafe/cmake-cookbook.git


本书内容与源码的章节对应，书中章节的编号和源码的顺序相同。
在GNU/Linux、MacOS和Windows上，使用最新的持续集成进行测试。我们会在之后讨论测试的设置。

0.2 Docker镜像
------------------

在Docker中进行环境搭建，无疑是非常方便的(依赖项都已经安装好了)。我们的Docker镜像是基于Ubuntu 18.04的镜像制作，
您可以按照 `官方文档 <https://docs.docker.com>`_ 在您的操作系统上安装Docker。


Docker安装好后，您可以下载并运行我们的镜像，然后可以对本书示例进行测试:

.. code-block:: bash

    docker run -it devcafe/cmake-cookbook_ubuntu-18.04
    git clone https://github.com/dev-cafe/cmake-cookbook.git
    cd cmake-cookbook
    pipenv install --three
    pipenv run python testing/collect_tests.py 'chapter-*/recipe-*'


0.3 安装必要的软件
--------------------

您必须安装以下组件：

  * CMake
  * 编译器
  * 自动化构建工具
  * Python

我们还会详细介绍，如何安装所需的某些依赖项。

.. code-block:: bash

    sudo apt-get install g++ gcc gfortran
    sudo apt-get install python3.5-dev
    sudo apt-get install libatlas-dev liblapack-dev liblapacke-dev # 依赖软件
    sudo apt-get install openmpi-bin libopenmpi-dev # 消息传递接口(MPI)
    sudo apt-get install libboost-filesystem-dev libboost-python-dev libboost-test-dev # Boost库
    sudo apt-get install gcc-mingw-w64 g++-mingw-w64 gfortran-mingw-w64 # 交叉编译器
    sudo apt-get install pkg-config libzmq3-dev doxygen graphviz-dev uuid-dev # ZeroMQ, pkg-config, UUID和Doxygen

线性代数模板库

.. code-block:: bash

    eigen_version="3.3.4"
    mkdir -p eigen
    curl -Ls http://bitbucket.org/eigen/eigen/get/${eigen_version}.tar.gz | tar -xz -C eigen --strip-components=1
    cd eigen
    cmake -H. -Bbuild_eigen -DCMAKE_INSTALL_PREFIX="$HOME/Deps/eigen" &> /dev/null
    cmake --build build_eigen -- install &> /dev/null

Conda的构建和部署

.. code-block:: bash

    conda install --yes --quiet conda-build anaconda-client jinja2 setuptools
    conda clean -tipsy
    conda info -a


0.4 测试环境
-------------------

示例在下列持续集成(CI)上进行过测试：

  * Travis( https://travis-ci.org )用于GNU/Linux和macOS
  * Appveyor( https://www.appveyor.com )用于Windows
  * CircleCI ( https://circleci.com )用于附加的GNU/Linux测试和商业编译器


CI服务的配置文件可以在示例库中找到( https://github.com/dev-cafe/cmake-cookbook/ ):

  * Travis的配置文件为travis.yml
  * Appveyor的配置文件为.appveyor.yml
  * CircleCI的配置文件为.circleci/config.yml

  
0.5 上报问题并提出改进建议
-----------------------------
请将遇到的问题反馈到 https://github.com/dev-cafe/cmake-cookbook/issues。

要对源码库进行贡献，我们建议对原始库 https://github.com/dev-cafe/cmake-cookbook 进行Fork，并使用Pull Request提交更改，可以参考这个页面 https://help.github.com/articles/creating-a-pull-request-from-a-fork/ 。

对于非重要更改，我们建议在发送Pull Request之前，首先在 https://github.com/devcafe/cmake-cookbook/issues 上创建一个问题进行描述，并讨论所要更改的问题。


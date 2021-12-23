# LTSLAM
## Introduction

lean slam step by step

project directory:

* xlsam
* xlsam_ros

download LTSLAM source:

```shell
https://github.com/quanduyong/LTSLAM.git
```

## Install

* 安装Sphinx

  ```shell
  pip install -U sphinx
  ```

* 安装主题

  ```shell
  pip install sphinx_rtd_theme
  ```

* 安装markdown插件

  ```shell
  pip install recommonmark
  pip install sphinx_markdown_tables
  ```

## Build

```shell
cd LTSLAM
mkdir build
cd build 
cmake ..
make -j6
```


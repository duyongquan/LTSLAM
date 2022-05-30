<center> <font color="green" size=10> X-SLAM Docker 部署</font></center>

# 1 docker安装

```bash
cd docker
./scripts/install_docker.sh
```



# 2 X-SLAM环境部署和安装

```bash
cd docker
./build_dev.sh standalone.x86_64.dockerfile
```



# 3 运行X-SLAM的demos案例

X-SLAM的demo有很多，一下简单运行几个demo

```bash
docker run -it xslam/ltslam
```



## 3.1 ceres

```bash
./xslam.ceres.helloworld_numeric_diff_test
```

## 3.2 g2o

```bash
./xslam.g2o.curve_fitting_test
```

## 3.3 dbow3

```bash
 ./xslam.dbow3.loop_closure_detect_test
```

## 3.4 opencv

```bash
./xslam.opencv.feature_detection.orb_feature_detector_test
```

## 3.5 sophus

```bash
./xslam.sophus.basic_test
```


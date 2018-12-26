cd Thirdparty/DBoW2/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../sophus
mkdir build
cd build
cmake ..
make -j4 

cd ../../fast
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo "Building ROS nodes"
cd ../../../ROS/ORB_YGZ_SLAM
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -l

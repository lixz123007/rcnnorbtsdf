echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j3

echo "Building ROS nodes"

cd ../Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j

cd ../../../..
ls
rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/ICI2.yaml
matlab -nodesktop -nosplash -nojvm -r "tsdf2mesh;quit;"

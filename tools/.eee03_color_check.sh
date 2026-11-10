#!/bin/bash
source /opt/ros/humble/setup.bash
echo "=== ntu_viral/raw/eee_03 内容（找相机图像）"
find /home/ywj/slam_data/ntu_viral/raw/eee_03/ -maxdepth 3 2>/dev/null | head -40
echo "=== eee_03 zip 内文件名"
unzip -l /home/ywj/slam_data/ntu_viral/raw/eee_03.zip 2>/dev/null | grep -iE 'cam|image|\.h264|\.bayer|calib|\.csv' | head -20
echo "=== eee_03_a bag 里 cloud_body_dense 的字段（是否带 rgb）"
python3 - <<'PY'
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri="/home/ywj/slam_data/ntu_odom_eee_03_a",
                                 storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
got=0
while r.has_next() and got<2:
    tp,pay,_=r.read_next()
    if "cloud_body_dense" in tp and isinstance(pay,(bytes,bytearray)):
        m=deserialize_message(pay,PointCloud2); n=m.width*m.height
        print("  ",tp,"n=",n,"fields=",[f.name for f in m.fields])
        got+=1
PY
echo "=== pcd 全目录（找任何上色云成品）"
ls -la /home/ywj/slam_data/pcd/ 2>/dev/null
for d in /home/ywj/slam_data/pcd/*/; do echo "-- $d"; ls "$d" 2>/dev/null | head -6; done
echo "=== eee_03 是否有 visual 上色产物"
find /home/ywj/slam_data -iname '*eee_03*' 2>/dev/null | grep -iE 'colou|rgb|visual|\.pcd' | head
echo "=== gt eee_03 行数与首末时间"
wc -l /home/ywj/slam_data/ntu_viral/gt/eee_03/ground_truth.csv
tail -1 /home/ywj/slam_data/ntu_viral/gt/eee_03/ground_truth.csv | cut -d, -f1-6
echo "=== ntu_odom_eee_03_a odom 首末时间"
python3 - <<'PY'
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri="/home/ywj/slam_data/ntu_odom_eee_03_a",
                                 storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
st=[]; op=[]
while r.has_next():
    tp,pay,_=r.read_next()
    if tp=="/lio/odom" and isinstance(pay,(bytes,bytearray)):
        m=deserialize_message(pay,Odometry); p=m.pose.pose.position
        st.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9); op.append([p.x,p.y,p.z])
st=np.array(st);op=np.array(op)
print("odom n=",len(st),"t=[%.2f,%.2f] span=%.1fs"%(st[0],st[-1],st[-1]-st[0]))
print("pos 范围 x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]"%(op[:,0].min(),op[:,0].max(),op[:,1].min(),op[:,1].max(),op[:,2].min(),op[:,2].max()))
print("路径长=%.1fm 首末距=%.1fm"%(np.linalg.norm(np.diff(op,axis=0),axis=1).sum(),np.linalg.norm(op[-1]-op[0])))
PY

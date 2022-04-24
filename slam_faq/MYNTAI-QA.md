1. 下面哪一种对表示三维空间刚体旋转变换的描述不准确（）  
   A. 旋转矩阵  
   B. 四元数  
   C. 旋转向量  
   D. 欧拉角  
   答案：B，应为 单位四元数 

2. 基础矩阵F、本质矩阵E 和 单应矩阵H 的自由度分别为（）  
   A. 8, 6, 9  
   B. 7, 6, 8  
   C. 8, 5, 9  
   D. 7, 5, 8  
   答案：D  

3. 下列关于SLAM优化问题的表述中，与其他三种表述不一致的是（）  
   A. optimization in the tangent space  
   B. optimization in the error-space  
   C. optimization in the euclidean space  
   D. optimization through local parametrization  
   答案：C  

4. 下面没有使用线段特征的为（）  
   A. LIBVISO2  
   B. PL-SLAM  
   C. PL-VIO  
   D. SVO 2.0  
   答案：A  

5. 下列贝叶斯滤波中，由正则参数表示其高斯分布的是（）  
   A. 卡尔曼滤波  
   B. 信息滤波  
   C. 直方图滤波  
   D. 粒子滤波  
   答案：B  

1. 下列与ORB特征无关的是（）  
   A. FAST  
   B. Image Moment   
   C. BRIEF   
   D. BRISK   
   答案：D，ORB全称Oriented FAST and Rotated BRIEF，FAST主方向的计算是通过图像矩(Image Moment)实现的；BRISK是另一种特征。

2. Lucas-Kanade光流法使用了下列Image Alignment中的哪一种（）  
   A. FAIA  
   B. FCIA  
   C. IAIA  
   D. ICIA  
   答案：A，Lucas-Kanade光流法用的FAIA。

3. 单目SLAM初始化时，下列哪种场景适用于基础矩阵F（）  
   A. 视差大、非平面  
   B. 视差小、非平面    
   C. 视差大、平面   
   D. 视差小、平面  
   答案：A，当场景为平面或近似平面，并且有足够视差时，一般选择基础矩阵F。

4. VINS-Mono计算IMU预积分用了Runge-Kutta数值积分法，其代码实现中用的是下列哪一种（）  
   A. 欧拉法  
   B. 中值法  
   C. RK3   
   D. RK4  
   答案：B，VINS-Mono论文中用的欧拉法，代码实现中用的中值法。

5. 基于EKF的传感器融合，预测状态量维度为4x1，观测量为3x1，则观测雅克比矩阵H维度为（）  
   A. 3x4  
   B. 4x3  
   C. 3x3  
   D. 4x4  
   答案：A，预测观测量 = 观测雅克比矩阵H x 预测状态量

6. 单目ORB-SLAM2中通过卡方检验区分外点，其卡方分布的自由度为（）  
   A. 1  
   B. 2  
   C. 3  
   D. 4  
   答案：B，单目ORB-SLAM重投影误差的卡方为 $e^{T} {\Sigma}^{-1} e$，两个独立的服从标准正太分布随机变量的平方和，即服从2自由度的卡方分布

7. LSD-SLAM和SVO中深度值的概率模型分别为（）  
   A. 高斯分布，高斯分布  
   B. 均匀分布，高斯-Beta分布  
   C. 卡方分布，均匀分布   
   D. 高斯分布，高斯-均匀分布    
   答案：D，LSD-SLAM深度值服从高斯分布，而SVO中深度值服从高斯-均匀分布  

8. 关于VI-SLAM的滑动窗口，下列与其无关的是（）   
   A. Vision-Only SFM   
   B. Marginalization  
   C. Schur Complement     
   D. FEJ  
   答案：A   

9. 下列哪一项不是用于闭环检测（）  
   A. DBoW  
   B. FBoW  
   C. iSAM  
   D. FAB-MAP  
   答案：C，iSAM是一个优化库

10. 下列哪一种不适用于机器人导航（）  
   A. 2D Costmap  
   B. OctoMap  
   C. Point Cloud  
   D. Robot-Centric Elevation Mapping   
   答案：C

1. 单目VO和VIO不可观的自由度分别为  
A. 7, 4  
B. 6, 4  
C. 7, 3  
D. 6, 3  
答案：A，单目VO的全局位置、旋转和平移共7个自由度不可观，VIO全局位置和yaw角共4自由度不可观  

1. 已知空间中不相等的两点p和q定义一条线l，则l的表达式为  
A. p + q  
B. p x q  
C. p * q  
D. p - q  
答案：B

1. 下列SLAM中用到深度学习的是  
A. ORB-SLAM2  
B. RTAB-Map  
C. CodeSLAM  
D. OKVIS  
答案：C

1. VIO后端优化求解最终化为Hx=b的形式，其中海塞矩阵H对应概率图中的哪一种  
A. 因子图  
B. 马尔科夫随机场  
C. 贝叶斯网络  
答案：B，稀疏雅克比矩阵J对应因子图，海塞矩阵H对应马尔科夫随机场  

1. 相机A、B是一对矫正后的双目相机，基线长度为50cm，相机焦距为900 pixel，某个空间点在相机A拍摄的图像中出现的位置为(233,213)，在相机B拍摄的图像中出现的位置为(213,213)，则可推测空间点的实际深度为  
A. 22.5 m  
B. 25.5 cm  
C. 27 m  
D. 27.5 cm   
答案：A，根据公式z=fb/d，z=50x900/(233-213)=2250cm=22.5m

1. 下列哪种特征没有描述子  
A. FAST  
B. ORB  
C. SIFT  
D. SURF  
答案：A

1. 两条彼此平行的直线，经过下列哪种变换后，可能不再平行  
A. 欧式变换  
B. 相似变换  
C. 仿射变换  
D. 射影变换  
答案：D

1. 下列增量式BA方法中，哪种是基于贝叶斯推断的  
A. SLAM++  
B. iSAM2  
C. EIBA  
D. ICE-BA  
答案：B  

1. 欧式3D空间中直线的自由度为   
A. 1  
B. 2  
C. 3  
D. 4  
答案：D

1. 下列不属于图像的频率特性的是  
A. 振幅  
B. 相位  
C. 分辨率  
D. 直流分量  
答案：C

1. 在计算Harris特征时，计算得到自相关矩阵$M$，其特征值为$\lambda_1$和$\lambda_2$，根据$M$获取得分方程 $R=\det(M)-k(\text{trace}(M))^2$，则当$R$比较大时，判断该特征为  
A. 角点  
B. 边缘  
C. 平面  
答案：A  

2. 已知镜头的真实焦距为$f$，相机sensor宽高分别为$W_s$、$H_s$，图像分辨率为$W_i \times H_i$，则传感器x轴和y轴单位像素的尺寸大小分别为$d_x=\frac{W_s}{W_i}$和$d_y=\frac{H_s}{H_i}$；对于针孔相机模型，其内参中焦距$f_x$的计算公式为  
A. $f_x=\frac{f}{d_x}$   
B. $f_x=\frac{d_x}{f}$  
C. $f_x=\frac{f}{W_i}$  
D. $f_x=\frac{W_i}{f}$  
答案：A



3. PTAM中所使用的ATAN相机模型对应Kalibr中的下列哪种  
  A. pinhole-equi  
  B. pinhole-fov  
  C. pinhole-radtan  
  D. ds-none  
  答案：B   

4. 三维点$P$ 经过旋转后变为$P'$，使用旋转矩阵表达为$P'=RP$，若使用$R$对应的四元数表示，则对应的表达式为  
A. $P'=qP$  
B. $P'=q^{-1}P$   
C. $P'=q^{-1}Pq$  
D. $P'=qPq^{-1}$  
答案：D   

5. 在对SLAM生成的轨迹做轨迹评估时，我们需要指定合适的轨迹对齐方式。对于mono-vSLAM生成的轨迹，我们需要选择下列哪种对齐方式  
A. a translation plus a rotation around gravity  
B. a rigid body transformation  
C. a similarity transformation  
答案：C   

6. SIFT和SURF特征描述子的维度分别为  
A. 128, 64  
B. 128, 256  
C. 64, 128  
D. 256, 64   
答案： A  

7. SLAM检测回环的方法有image-to-image, map-to-map, image-to-map三种，请问RTAB-Map中检测回环使用哪一种  
A. image-to-image  
B. map-to-map  
C. image-to-map  
答案：A  

8. 下列哪一种不是深度相机的原理  
A. 双目  
B. 结构光   
C. TOF  
D. RGB  
答案：D  

9. 下列库中的哪一个是是基于左手系的  
A. Eigen  
B. OpenGL  
C. Unity3D  
D. ROS TF  
答案：C  

10. 关于图像格式或颜色空间，NV21属于下列中的哪一种  
A. YUV444  
B. YUV422  
C. YUV420  
答案：C
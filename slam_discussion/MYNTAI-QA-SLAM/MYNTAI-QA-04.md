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
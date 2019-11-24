
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
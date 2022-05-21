# SLAM FAQ by GaoXiang

* https://www.zhihu.com/question/532565032/answer/2483264411

---

***Q: 写一个求两点的距离函数。要求支持常见的2维、3维至任意有限高维点。***


***Q: pose的时间插值是常见需求。写一个6自由度pose的插值函数。pose和时间戳是自定义的结构体，例如 std::vector<TimedPose> 之类。TimedPose.time_stamp为时间，TimedPose.pose是位姿。***

ref: https://robotacademy.net.au/lesson/interpolating-pose-in-3d/

interpolate translation

$$
t(s) = (1-s) t_0 + s t_1
$$

interpolate rotation

$$
q(s) = \frac{\sin((1-s)\theta) q_0 + \sin(s \theta) q_1}{\sin \theta}
$$




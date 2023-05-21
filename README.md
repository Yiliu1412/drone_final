DroneFinal
---

上海交通大学电子信息与电气工程学院第八届新生杯科技创新大赛

---

# 2023年5月10日22:21:29

1. 学习了tello相关的api，学习如何使用opencv识别ArUco和手部；
2. 初步了解了所有API的调用和实现。

# 2023年5月12日21:42:22

1. [教育无人机系列连接方式](https://robomaster-dev.readthedocs.io/zh_CN/latest/python_sdk/connection.html#telloconn)
2. [Tello用户手册](https://manuals.plus/zh-CN/%E7%91%9E%E6%B3%BD%E7%A7%91%E6%8A%80/%E7%89%B9%E4%B9%90%E9%A3%9E%E6%9C%BA%E6%89%8B%E5%86%8C#aircraft_diagram)

# 2023年5月17日21:07:24
1. [vision](https://robomaster-dev.readthedocs.io/zh_CN/latest/python_sdk/robomaster.html#module-robomaster.vision)
2. [mediapipe实现手势识别](https://google.github.io/mediapipe/)

# 2023年5月19日20:34:32
1. [椭圆识别](https://blog.csdn.net/qq_33950926/article/details/111409635)

# 2023年5月19日23:41:00
1. 大概能跑完全程
2. step 2 添加黄色圈识别
3. 还需研究降落部分

# 2023年5月20日13:20:56
1. 找到无人机的下置摄像机调用接口，开始实现地面ArUco码识别

# 2023年5月21日04:09:30
1. [PID算法调用的库](https://github.com/m-lundberg/simple-pid)
2. 实现了使用PID算法+timer+counter的地面ArUco码识别，大大提高了精度

# 2023年5月21日14:06:51
正式比赛顺利完赛。
还被主持人调戏了x


喜获二等奖。


All Last
---

赛后反思：
- 时间安排不合适，赛程后半才开始发力调试
- 从算法鲁棒性出发，对于不同情况的适应力强
- 未考虑完成时间，算法求稳
- 码力欠佳，代码风格规范性差
- 太肝了，不适合可持续发展
- 没写代码进度跟进，版本控制有点失控

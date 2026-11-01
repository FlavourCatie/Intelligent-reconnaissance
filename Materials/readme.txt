Autonomous_Vehicle_PnC_Tutorial-master：是五讲内容
demo测试。
1：A*规划。
2：PID控制
3：stanley控制
4：局部规划（很难。不讲）
5：整理流程



practice（原名925文件）
practice1_Scene：场景布置文件。运行subject4.exe生成车和机。plan.py，plan_control.py 对应这个文件。
practice2_Scene：场景布置文件。运行subject4.exe生成车和机。uav_control.py 对应这个文件。
easy_follow：
	click.py  ：辅助确认像素坐标
	plan.py  :   路径规划
	plan_control.py  :   规划然后控制
	uav_control.py  ：控制无人机  



！！！
整个联动的运行流程：
simu_driver_windows->start.bat 启动飞控。
打开CarlaUE4   
点击practice1_Scene->subject4.exe   (不同文件要求的不一致)
运行py文件
！！！


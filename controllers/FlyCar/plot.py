import pickle
import matplotlib.pyplot as plt

[time_data, LQR_distance_data, LQR_speed_data, LQR_angle_data, LQR_gyro_data, angle_control_data, gyro_control_data,
 distance_control_data, speed_control_data, LQR_u_data, angle_zeropoint_data] = pickle.load(open('data.pkl', 'rb'))


plt.figure(figsize=(10, 8))
# 绘制每条曲线
plt.plot(time_data, LQR_distance_data, label='LQR Distance')
plt.plot(time_data, LQR_speed_data, label='LQR Speed')
plt.plot(time_data, LQR_angle_data, label='LQR Angle')
plt.plot(time_data, LQR_gyro_data, label='LQR Gyro')
plt.plot(time_data, angle_control_data, label='Angle Control')
plt.plot(time_data, gyro_control_data, label='Gyro Control')
plt.plot(time_data, distance_control_data, label='Distance Control')
plt.plot(time_data, speed_control_data, label='Speed Control')
plt.plot(time_data, angle_zeropoint_data, label='Angle Zeropoint')


# 设置图例、标题和标签
plt.legend(loc='best')
plt.title('Various Data over Time')
plt.xlabel('Time')
plt.ylabel('Values')

# 显示网格
plt.grid(True)

# 显示图形
plt.show()

plt.figure(figsize=(10, 8))
plt.plot(time_data, LQR_u_data, label='LQR U')
# 设置图例、标题和标签
plt.legend(loc='best')
plt.title('LQR U')
plt.xlabel('Time')
plt.ylabel('Values')

# 显示网格
plt.grid(True)

# 显示图形
plt.show()
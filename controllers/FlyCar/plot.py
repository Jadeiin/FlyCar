import pickle
import matplotlib.pyplot as plt

[time_data, pitch_data, velocity_data, balance_torque_data,
 turn_torque_data, walk_torque_data] = pickle.load(open('data.pkl', 'rb'))

plt.figure(figsize=(10, 8))
# 绘制每条曲线
plt.plot(time_data, pitch_data, label='pitch_data')
plt.plot(time_data, velocity_data, label='velocity_data')
# plt.plot(time_data, balance_torque_data, label='balance_torque_data')
# plt.plot(time_data, turn_torque_data, label='turn_torque_data')
# plt.plot(time_data, walk_torque_data, label='walk_torque_data')

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
# 绘制每条曲线
plt.plot(time_data, balance_torque_data, label='balance_torque_data')
plt.plot(time_data, turn_torque_data, label='turn_torque_data')
plt.plot(time_data, walk_torque_data, label='walk_torque_data')

# 设置图例、标题和标签
plt.legend(loc='best')
plt.title('Various Data over Time')
plt.xlabel('Time')
plt.ylabel('Values')

# 显示网格
plt.grid(True)

# 显示图形
plt.show()
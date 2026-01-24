from rplidar import RPLidar, RPLidarException
import time

PORT = '/dev/ttyUSB0'
BAUDRATE = 256000 # S1用256000，S2用1000000

def get_robust_tuple_list():
    lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=1)
    
    # 存储一圈数据的临时列表
    current_scan = []
    
    try:
        print("开始鲁棒性数据采集...")
        # 使用 iter_measurements，它的粒度更细
        iterator = lidar.iter_measurments()
        
        while True:
            try:
                # 获取一个点的数据
                new_scan, quality, angle, distance = next(iterator)
                
                # 如果是新的一圈的开始，且之前已经有数据了，就输出上一圈
                if new_scan and current_scan:
                    # 转换为你需要的格式: ((a1, d1), (a2, d2)...)
                    output = tuple(current_scan)
                    print(f"成功捕获一圈: {len(output)} 点")
                    # 这里进行你的后续逻辑
                    current_scan = []
                
                # 过滤无效点并添加到当前圈
                if distance > 0:
                    current_scan.append((round(angle, 2), round(distance, 2)))
                    
            except RPLidarException as re:
                # 关键：捕获 Wrong body size，但不退出，而是重置迭代器
                print(f"捕捉到硬件丢包(已跳过): {re}")
                lidar.clear_input() # 清空串口缓存防止粘包
                time.sleep(0.02)
                iterator = lidar.iter_measurments()
                continue
            except StopIteration:
                break
                
    except KeyboardInterrupt:
        print("停止采集")
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    get_robust_tuple_list()
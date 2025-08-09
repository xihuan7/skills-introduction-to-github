# K230 优化版寻找最大矩形框示例
# 功能：检测图像中的矩形，找到最大的矩形框，并打印四个角点和中心点坐标
# 优化：简化滤波算法，提升性能，增强错误处理

import time, os, gc, sys, image
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, Pin, FPIOA
import cv_lite
import math

# 图像检测尺寸配置
DETECT_WIDTH = 320  # 宽度对齐到16的倍数
DETECT_HEIGHT = 240  # 高度
IMAGE_CENTER_X = DETECT_WIDTH // 2  # 图像中线x坐标

# 显示模式选择
DISPLAY_MODE = "LCD"  # 可选: "VIRT", "LCD"

# 根据显示模式设置分辨率
if DISPLAY_MODE == "VIRT":
    DISPLAY_WIDTH = DETECT_WIDTH
    DISPLAY_HEIGHT = DETECT_HEIGHT
elif DISPLAY_MODE == "LCD":
    DISPLAY_WIDTH = 640
    DISPLAY_HEIGHT = 480
else:
    raise ValueError("Unknown DISPLAY_MODE, please select 'VIRT', 'LCD'")

# 全局变量
sensor = None
uart1 = None
coord_filter = None

# 优化版简化滤波器类
class SimpleMovingAverageFilter:
    """简化的移动平均滤波器，性能更好，适合微控制器"""
    
    def __init__(self, window_size=5, alpha=0.3):
        self.window_size = window_size
        self.alpha = alpha  # 指数平滑因子
        self.initialized = False
        self.smooth_x = 0.0
        self.smooth_y = 0.0
        
    def update(self, x, y):
        """更新滤波器状态"""
        if not self.initialized:
            self.smooth_x = float(x)
            self.smooth_y = float(y)
            self.initialized = True
        else:
            # 指数移动平均
            self.smooth_x = self.alpha * x + (1 - self.alpha) * self.smooth_x
            self.smooth_y = self.alpha * y + (1 - self.alpha) * self.smooth_y
    
    def get_position(self):
        """获取滤波后的位置"""
        if not self.initialized:
            return None
        return (int(self.smooth_x), int(self.smooth_y))
    
    def reset(self):
        """重置滤波器"""
        self.initialized = False
        self.smooth_x = 0.0
        self.smooth_y = 0.0

class OptimizedCoordinateFilter:
    """优化的坐标滤波器，使用简化的移动平均算法"""
    
    def __init__(self, alpha=0.3):
        # 为四个角点和中心点创建滤波器
        self.corner_filters = [SimpleMovingAverageFilter(alpha=alpha) for _ in range(4)]
        self.center_filter = SimpleMovingAverageFilter(alpha=alpha)
        self.data_count = 0
        
    def add_corners(self, corners):
        """添加角点坐标"""
        self.data_count += 1
        for i, (x, y) in enumerate(corners):
            self.corner_filters[i].update(x, y)
    
    def add_center(self, center):
        """添加中心点坐标"""
        x, y = center
        self.center_filter.update(x, y)
    
    def get_filtered_corners(self):
        """获取滤波后的角点"""
        if self.data_count < 2:
            return None
        
        filtered_corners = []
        for corner_filter in self.corner_filters:
            pos = corner_filter.get_position()
            if pos is None:
                return None
            filtered_corners.append(pos)
        return filtered_corners
    
    def get_filtered_center(self):
        """获取滤波后的中心点"""
        if self.data_count < 2:
            return None
        return self.center_filter.get_position()
    
    def reset(self):
        """重置所有滤波器"""
        for corner_filter in self.corner_filters:
            corner_filter.reset()
        self.center_filter.reset()
        self.data_count = 0

def uart_init():
    """初始化UART串口"""
    global uart1
    
    try:
        fpioa = FPIOA()
        fpioa.set_function(3, FPIOA.UART1_TXD)
        fpioa.set_function(4, FPIOA.UART1_RXD)
        uart1 = UART(UART.UART1, 115200)
        return True
    except Exception as e:
        print(f"UART初始化失败: {e}")
        return False

def camera_init():
    """初始化摄像头"""
    global sensor
    
    try:
        # 构造传感器对象
        sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
        sensor.reset()
        
        # 设置输出参数
        sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)
        
        # 初始化显示器
        if DISPLAY_MODE == "VIRT":
            Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, fps=100, to_ide=True)
        elif DISPLAY_MODE == "LCD":
            Display.init(Display.ST7701, width=800, height=480, to_ide=True)
        
        # 初始化媒体管理器
        MediaManager.init()
        sensor.run()
        
        return True
    except Exception as e:
        print(f"摄像头初始化失败: {e}")
        return False

def uart_deinit():
    """释放UART串口资源"""
    global uart1
    if uart1:
        try:
            uart1.deinit()
            uart1 = None
        except:
            pass

def camera_deinit():
    """释放摄像头资源"""
    global sensor
    try:
        if sensor:
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
    except:
        pass

def calculate_center_fast(corners):
    """快速计算矩形中心点"""
    if len(corners) != 4:
        return None
    
    # 简单的几何中心计算，速度更快
    cx = sum(corner[0] for corner in corners) / 4
    cy = sum(corner[1] for corner in corners) / 4
    return (int(cx), int(cy))

def send_uart_data(x_error):
    """发送UART数据，包含错误处理"""
    if not uart1:
        return False
    
    try:
        # 限制误差范围
        error_int = max(-100, min(100, int(x_error)))
        
        # 构造数据帧
        if error_int < 0:
            error_bytes = (error_int + 65536).to_bytes(2, 'little')
        else:
            error_bytes = error_int.to_bytes(2, 'little')
        
        frame_data = bytes([0x66, 0x66]) + error_bytes + bytes([0xf6, 0xf6])
        uart1.write(frame_data)
        return True
    except Exception as e:
        print(f"UART发送失败: {e}")
        return False

def process_rectangles(rects_data):
    """处理矩形检测结果，优化筛选逻辑"""
    if not rects_data or len(rects_data) < 4:
        return None
    
    # 预分配列表大小，提升性能
    max_rects = len(rects_data) // 4
    filtered_rects = []
    
    # 优化的矩形筛选
    for i in range(0, len(rects_data), 4):
        if i + 3 >= len(rects_data):
            break
            
        x, y, w, h = rects_data[i:i+4]
        
        # 快速面积和宽高比检查
        area = w * h
        if area < 1500:  # 提前过滤小面积
            continue
            
        if h == 0:  # 避免除零错误
            continue
            
        aspect_ratio = w / h
        
        # 宽高比过滤
        if 0.6 <= aspect_ratio <= 1.6:
            rect_info = {
                'x': x, 'y': y, 'w': w, 'h': h,
                'area': area, 'aspect_ratio': aspect_ratio
            }
            filtered_rects.append(rect_info)
    
    # 返回最大面积的矩形
    if filtered_rects:
        return max(filtered_rects, key=lambda r: r['area'])
    return None

def draw_detection_info(img, max_rect, center, x_error, fps_val):
    """绘制检测信息，优化绘制性能"""
    if max_rect:
        # 绘制矩形框
        x, y, w, h = max_rect['x'], max_rect['y'], max_rect['w'], max_rect['h']
        img.draw_rectangle([x, y, w, h], color=(0, 255, 0), thickness=2)
        
        # 绘制四个角点
        corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        for corner in corners:
            img.draw_circle(corner[0], corner[1], 5, color=(255, 0, 0), thickness=2)
        
        # 绘制中心点
        if center:
            img.draw_circle(center[0], center[1], 8, color=(0, 0, 255), thickness=2)
    
    # 绘制图像中心线
    img.draw_circle(IMAGE_CENTER_X, DETECT_HEIGHT // 2, 3, color=(255, 255, 0), thickness=2)
    
    # 绘制信息文本
    img.draw_string_advanced(5, 5, 16, f"FPS: {fps_val:.1f}", color=(255, 255, 255, 0))
    img.draw_string_advanced(5, 25, 16, f"Mode: {DISPLAY_MODE}", color=(255, 255, 255, 0))
    
    if 'x_error' in locals() and x_error is not None:
        img.draw_string_advanced(DETECT_WIDTH - 150, 5, 16, f"Error: {x_error:.1f}", color=(0, 255, 255, 0))

def capture_picture():
    """主要的图像捕获和处理函数"""
    global coord_filter
    
    # 初始化滤波器
    coord_filter = OptimizedCoordinateFilter(alpha=0.3)
    
    fps = time.clock()
    
    # cv_lite检测参数
    detection_params = {
        'canny_thresh1': 50,
        'canny_thresh2': 150,
        'approx_epsilon': 0.04,
        'area_min_ratio': 0.01,
        'max_angle_cos': 0.3,
        'gaussian_blur_size': 5
    }
    
    while True:
        fps.tick()
        
        try:
            os.exitpoint()
            
            # 获取图像
            img = sensor.snapshot()
            img_gray = img.to_grayscale()
            
            # OTSU二值化
            hist = img_gray.get_histogram()
            otsu_threshold_obj = hist.get_threshold()
            threshold_value = (otsu_threshold_obj.value(), 255)
            img_binary = img_gray.binary([threshold_value])
            
            # 矩形检测
            image_shape = [DETECT_HEIGHT, DETECT_WIDTH]
            img_gray_np = img_gray.to_numpy_ref()
            
            rects_data = cv_lite.grayscale_find_rectangles(
                image_shape, img_gray_np,
                detection_params['canny_thresh1'],
                detection_params['canny_thresh2'],
                detection_params['approx_epsilon'],
                detection_params['area_min_ratio'],
                detection_params['max_angle_cos'],
                detection_params['gaussian_blur_size']
            )
            
            # 处理检测结果
            max_rect = process_rectangles(rects_data)
            
            if max_rect:
                # 计算角点和中心点
                x, y, w, h = max_rect['x'], max_rect['y'], max_rect['w'], max_rect['h']
                corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
                center = calculate_center_fast(corners)
                
                # 计算误差
                x_error = center[0] - IMAGE_CENTER_X
                x_error = max(-100, min(100, x_error))  # 限制范围
                
                # 更新滤波器
                coord_filter.add_corners(corners)
                coord_filter.add_center(center)
                
                # 发送UART数据
                send_uart_data(x_error)
                
                # 获取滤波后的坐标用于显示
                filtered_corners = coord_filter.get_filtered_corners()
                filtered_center = coord_filter.get_filtered_center()
                
                # 使用滤波后的坐标绘制（如果可用）
                display_rect = max_rect
                display_center = filtered_center if filtered_center else center
                
                if filtered_corners:
                    # 重新计算滤波后的矩形边界
                    min_x = min(corner[0] for corner in filtered_corners)
                    max_x = max(corner[0] for corner in filtered_corners)
                    min_y = min(corner[1] for corner in filtered_corners)
                    max_y = max(corner[1] for corner in filtered_corners)
                    
                    display_rect = {
                        'x': min_x, 'y': min_y, 
                        'w': max_x - min_x, 'h': max_y - min_y
                    }
                
                # 绘制检测结果
                draw_detection_info(img, display_rect, display_center, x_error, fps.fps())
                
            else:
                # 没有检测到矩形，重置滤波器
                coord_filter.reset()
                draw_detection_info(img, None, None, None, fps.fps())
            
            # 显示图像
            if DISPLAY_MODE == "LCD":
                x = int((800 - DETECT_WIDTH) // 2)
                y = int((480 - DETECT_HEIGHT) // 2)
                Display.show_image(img, x=x, y=y)
            else:
                Display.show_image(img)
            
            # 清理内存
            img = None
            gc.collect()
            
        except KeyboardInterrupt:
            print("用户停止")
            break
        except Exception as e:
            print(f"处理异常: {e}")
            continue

def main():
    """主函数"""
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    uart_is_init = False
    
    try:
        print("初始化UART...")
        uart_is_init = uart_init()
        if not uart_is_init:
            print("UART初始化失败，继续运行...")
        
        print("初始化摄像头...")
        camera_is_init = camera_init()
        if not camera_is_init:
            print("摄像头初始化失败")
            return
        
        print("开始矩形检测...")
        capture_picture()
        
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        if camera_is_init:
            print("释放摄像头资源...")
            camera_deinit()
        if uart_is_init:
            print("释放UART资源...")
            uart_deinit()
        print("程序结束")

if __name__ == "__main__":
    main()
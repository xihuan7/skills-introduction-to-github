# K230 集成配置版矩形检测系统
# 使用外部配置文件进行参数管理

import time, os, gc, sys, image
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, Pin, FPIOA
import cv_lite
import math

# 导入配置
try:
    from k230_config import DetectionConfig, get_detection_params, get_filter_params, get_rectangle_filter_params
    CONFIG_AVAILABLE = True
except ImportError:
    CONFIG_AVAILABLE = False
    print("配置文件未找到，使用默认参数")

# 全局变量
sensor = None
uart1 = None
coord_filter = None

# 如果配置文件不可用，使用默认配置
if not CONFIG_AVAILABLE:
    class DetectionConfig:
        DETECT_WIDTH = 320
        DETECT_HEIGHT = 240
        DISPLAY_MODE = "LCD"
        MIN_AREA = 1500
        MIN_ASPECT_RATIO = 0.6
        MAX_ASPECT_RATIO = 1.6
        FILTER_ALPHA = 0.3
        MIN_FILTER_FRAMES = 2
        UART_BAUDRATE = 115200
        UART_TX_PIN = 3
        UART_RX_PIN = 4
        MAX_ERROR_RANGE = 100
        ENABLE_GC_PER_FRAME = True
        ENABLE_UART_ERROR_PRINT = True

# 使用配置参数
DETECT_WIDTH = DetectionConfig.DETECT_WIDTH
DETECT_HEIGHT = DetectionConfig.DETECT_HEIGHT
IMAGE_CENTER_X = DETECT_WIDTH // 2
DISPLAY_MODE = DetectionConfig.DISPLAY_MODE

# 根据显示模式设置分辨率
if DISPLAY_MODE == "VIRT":
    DISPLAY_WIDTH = DETECT_WIDTH
    DISPLAY_HEIGHT = DETECT_HEIGHT
elif DISPLAY_MODE == "LCD":
    DISPLAY_WIDTH = 640
    DISPLAY_HEIGHT = 480
else:
    raise ValueError("Unknown DISPLAY_MODE, please select 'VIRT', 'LCD'")

class SimpleMovingAverageFilter:
    """简化的移动平均滤波器"""
    
    def __init__(self, alpha=None):
        self.alpha = alpha if alpha else DetectionConfig.FILTER_ALPHA
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
    """优化的坐标滤波器"""
    
    def __init__(self, alpha=None):
        filter_alpha = alpha if alpha else DetectionConfig.FILTER_ALPHA
        self.corner_filters = [SimpleMovingAverageFilter(filter_alpha) for _ in range(4)]
        self.center_filter = SimpleMovingAverageFilter(filter_alpha)
        self.data_count = 0
        self.min_frames = DetectionConfig.MIN_FILTER_FRAMES
        
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
        if self.data_count < self.min_frames:
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
        if self.data_count < self.min_frames:
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
        fpioa.set_function(DetectionConfig.UART_TX_PIN, FPIOA.UART1_TXD)
        fpioa.set_function(DetectionConfig.UART_RX_PIN, FPIOA.UART1_RXD)
        uart1 = UART(UART.UART1, DetectionConfig.UART_BAUDRATE)
        return True
    except Exception as e:
        if DetectionConfig.ENABLE_UART_ERROR_PRINT:
            print(f"UART初始化失败: {e}")
        return False

def camera_init():
    """初始化摄像头"""
    global sensor
    
    try:
        sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
        sensor.reset()
        
        sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)
        
        if DISPLAY_MODE == "VIRT":
            Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, fps=100, to_ide=True)
        elif DISPLAY_MODE == "LCD":
            Display.init(Display.ST7701, width=800, height=480, to_ide=True)
        
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
    
    cx = sum(corner[0] for corner in corners) / 4
    cy = sum(corner[1] for corner in corners) / 4
    return (int(cx), int(cy))

def send_uart_data(x_error):
    """发送UART数据"""
    if not uart1:
        return False
    
    try:
        error_int = max(-DetectionConfig.MAX_ERROR_RANGE, 
                       min(DetectionConfig.MAX_ERROR_RANGE, int(x_error)))
        
        if error_int < 0:
            error_bytes = (error_int + 65536).to_bytes(2, 'little')
        else:
            error_bytes = error_int.to_bytes(2, 'little')
        
        frame_data = bytes([0x66, 0x66]) + error_bytes + bytes([0xf6, 0xf6])
        uart1.write(frame_data)
        return True
    except Exception as e:
        if DetectionConfig.ENABLE_UART_ERROR_PRINT:
            print(f"UART发送失败: {e}")
        return False

def process_rectangles(rects_data):
    """处理矩形检测结果"""
    if not rects_data or len(rects_data) < 4:
        return None
    
    filtered_rects = []
    
    for i in range(0, len(rects_data), 4):
        if i + 3 >= len(rects_data):
            break
            
        x, y, w, h = rects_data[i:i+4]
        
        area = w * h
        if area < DetectionConfig.MIN_AREA:
            continue
            
        if h == 0:
            continue
            
        aspect_ratio = w / h
        
        if DetectionConfig.MIN_ASPECT_RATIO <= aspect_ratio <= DetectionConfig.MAX_ASPECT_RATIO:
            rect_info = {
                'x': x, 'y': y, 'w': w, 'h': h,
                'area': area, 'aspect_ratio': aspect_ratio
            }
            filtered_rects.append(rect_info)
    
    if filtered_rects:
        return max(filtered_rects, key=lambda r: r['area'])
    return None

def draw_detection_info(img, max_rect, center, x_error, fps_val):
    """绘制检测信息"""
    if max_rect:
        x, y, w, h = max_rect['x'], max_rect['y'], max_rect['w'], max_rect['h']
        img.draw_rectangle([x, y, w, h], color=(0, 255, 0), thickness=2)
        
        corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        for corner in corners:
            img.draw_circle(corner[0], corner[1], 5, color=(255, 0, 0), thickness=2)
        
        if center:
            img.draw_circle(center[0], center[1], 8, color=(0, 0, 255), thickness=2)
    
    img.draw_circle(IMAGE_CENTER_X, DETECT_HEIGHT // 2, 3, color=(255, 255, 0), thickness=2)
    
    img.draw_string_advanced(5, 5, 16, f"FPS: {fps_val:.1f}", color=(255, 255, 255, 0))
    img.draw_string_advanced(5, 25, 16, f"Mode: {DISPLAY_MODE}", color=(255, 255, 255, 0))
    
    if x_error is not None:
        img.draw_string_advanced(DETECT_WIDTH - 150, 5, 16, f"Error: {x_error:.1f}", color=(0, 255, 255, 0))

def capture_picture():
    """主要的图像捕获和处理函数"""
    global coord_filter
    
    coord_filter = OptimizedCoordinateFilter()
    fps = time.clock()
    
    # 获取检测参数
    if CONFIG_AVAILABLE:
        detection_params = get_detection_params()
    else:
        detection_params = {
            'canny_thresh1': 50,
            'canny_thresh2': 150,
            'approx_epsilon': 0.04,
            'area_min_ratio': 0.01,
            'max_angle_cos': 0.3,
            'gaussian_blur_size': 5
        }
    
    print(f"使用检测参数: {detection_params}")
    
    while True:
        fps.tick()
        
        try:
            os.exitpoint()
            
            img = sensor.snapshot()
            img_gray = img.to_grayscale()
            
            hist = img_gray.get_histogram()
            otsu_threshold_obj = hist.get_threshold()
            threshold_value = (otsu_threshold_obj.value(), 255)
            img_binary = img_gray.binary([threshold_value])
            
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
            
            max_rect = process_rectangles(rects_data)
            
            if max_rect:
                x, y, w, h = max_rect['x'], max_rect['y'], max_rect['w'], max_rect['h']
                corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
                center = calculate_center_fast(corners)
                
                x_error = center[0] - IMAGE_CENTER_X
                x_error = max(-DetectionConfig.MAX_ERROR_RANGE, 
                             min(DetectionConfig.MAX_ERROR_RANGE, x_error))
                
                coord_filter.add_corners(corners)
                coord_filter.add_center(center)
                
                send_uart_data(x_error)
                
                filtered_corners = coord_filter.get_filtered_corners()
                filtered_center = coord_filter.get_filtered_center()
                
                display_rect = max_rect
                display_center = filtered_center if filtered_center else center
                
                if filtered_corners:
                    min_x = min(corner[0] for corner in filtered_corners)
                    max_x = max(corner[0] for corner in filtered_corners)
                    min_y = min(corner[1] for corner in filtered_corners)
                    max_y = max(corner[1] for corner in filtered_corners)
                    
                    display_rect = {
                        'x': min_x, 'y': min_y, 
                        'w': max_x - min_x, 'h': max_y - min_y
                    }
                
                draw_detection_info(img, display_rect, display_center, x_error, fps.fps())
                
            else:
                coord_filter.reset()
                draw_detection_info(img, None, None, None, fps.fps())
            
            if DISPLAY_MODE == "LCD":
                x = int((800 - DETECT_WIDTH) // 2)
                y = int((480 - DETECT_HEIGHT) // 2)
                Display.show_image(img, x=x, y=y)
            else:
                Display.show_image(img)
            
            if DetectionConfig.ENABLE_GC_PER_FRAME:
                img = None
                gc.collect()
            
        except KeyboardInterrupt:
            print("用户停止")
            break
        except Exception as e:
            print(f"处理异常: {e}")
            continue

def print_config_info():
    """打印配置信息"""
    print("=" * 50)
    print("K230 矩形检测系统配置信息")
    print("=" * 50)
    print(f"图像尺寸: {DETECT_WIDTH}x{DETECT_HEIGHT}")
    print(f"显示模式: {DISPLAY_MODE}")
    print(f"最小面积: {DetectionConfig.MIN_AREA}")
    print(f"宽高比范围: {DetectionConfig.MIN_ASPECT_RATIO}-{DetectionConfig.MAX_ASPECT_RATIO}")
    print(f"滤波系数: {DetectionConfig.FILTER_ALPHA}")
    print(f"UART波特率: {DetectionConfig.UART_BAUDRATE}")
    print(f"配置文件: {'已加载' if CONFIG_AVAILABLE else '未找到，使用默认'}")
    print("=" * 50)

def main():
    """主函数"""
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera_is_init = False
    uart_is_init = False
    
    print_config_info()
    
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
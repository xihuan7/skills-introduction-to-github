# K230 矩形检测配置文件
# 可以根据实际应用场景调整这些参数

class DetectionConfig:
    """矩形检测配置类"""
    
    # 图像尺寸配置
    DETECT_WIDTH = 320
    DETECT_HEIGHT = 240
    
    # 显示配置
    DISPLAY_MODE = "LCD"  # "VIRT" 或 "LCD"
    
    # 矩形筛选参数
    MIN_AREA = 1500  # 最小矩形面积
    MIN_ASPECT_RATIO = 0.6  # 最小宽高比
    MAX_ASPECT_RATIO = 1.6  # 最大宽高比
    
    # cv_lite检测参数
    CANNY_THRESH1 = 50  # Canny边缘检测低阈值
    CANNY_THRESH2 = 150  # Canny边缘检测高阈值
    APPROX_EPSILON = 0.04  # 多边形拟合精度
    AREA_MIN_RATIO = 0.01  # 最小面积比例
    MAX_ANGLE_COS = 0.3  # 最大角度余弦值
    GAUSSIAN_BLUR_SIZE = 5  # 高斯模糊核大小
    
    # 滤波器参数
    FILTER_ALPHA = 0.3  # 指数移动平均滤波系数 (0-1)
    MIN_FILTER_FRAMES = 2  # 开始输出滤波结果的最小帧数
    
    # UART配置
    UART_BAUDRATE = 115200
    UART_TX_PIN = 3
    UART_RX_PIN = 4
    
    # 误差限制
    MAX_ERROR_RANGE = 100  # 最大误差范围 (+/-)
    
    # 性能优化选项
    ENABLE_GC_PER_FRAME = True  # 每帧启用垃圾回收
    ENABLE_UART_ERROR_PRINT = True  # 启用UART错误打印

class AdvancedConfig:
    """高级配置参数"""
    
    # 自适应阈值参数
    ADAPTIVE_THRESHOLD = True  # 启用自适应阈值
    THRESHOLD_BLOCK_SIZE = 11  # 自适应阈值块大小
    THRESHOLD_C = 2  # 自适应阈值常数
    
    # 形态学操作参数
    ENABLE_MORPHOLOGY = False  # 启用形态学操作
    MORPH_KERNEL_SIZE = 3  # 形态学操作核大小
    
    # ROI设置（感兴趣区域）
    ENABLE_ROI = False  # 启用ROI
    ROI_X = 50
    ROI_Y = 50
    ROI_WIDTH = 220
    ROI_HEIGHT = 140
    
    # 多目标跟踪
    ENABLE_MULTI_TARGET = False  # 启用多目标跟踪
    MAX_TARGETS = 3  # 最大跟踪目标数

def get_detection_params():
    """获取cv_lite检测参数字典"""
    return {
        'canny_thresh1': DetectionConfig.CANNY_THRESH1,
        'canny_thresh2': DetectionConfig.CANNY_THRESH2,
        'approx_epsilon': DetectionConfig.APPROX_EPSILON,
        'area_min_ratio': DetectionConfig.AREA_MIN_RATIO,
        'max_angle_cos': DetectionConfig.MAX_ANGLE_COS,
        'gaussian_blur_size': DetectionConfig.GAUSSIAN_BLUR_SIZE
    }

def get_filter_params():
    """获取滤波器参数字典"""
    return {
        'alpha': DetectionConfig.FILTER_ALPHA,
        'min_frames': DetectionConfig.MIN_FILTER_FRAMES
    }

def get_rectangle_filter_params():
    """获取矩形筛选参数字典"""
    return {
        'min_area': DetectionConfig.MIN_AREA,
        'min_aspect_ratio': DetectionConfig.MIN_ASPECT_RATIO,
        'max_aspect_ratio': DetectionConfig.MAX_ASPECT_RATIO
    }

# 预设配置方案
class PresetConfigs:
    """预设配置方案"""
    
    @staticmethod
    def high_accuracy():
        """高精度配置"""
        DetectionConfig.CANNY_THRESH1 = 30
        DetectionConfig.CANNY_THRESH2 = 100
        DetectionConfig.APPROX_EPSILON = 0.02
        DetectionConfig.FILTER_ALPHA = 0.2
        DetectionConfig.MIN_AREA = 2000
    
    @staticmethod
    def high_speed():
        """高速度配置"""
        DetectionConfig.CANNY_THRESH1 = 70
        DetectionConfig.CANNY_THRESH2 = 200
        DetectionConfig.APPROX_EPSILON = 0.06
        DetectionConfig.FILTER_ALPHA = 0.5
        DetectionConfig.MIN_AREA = 1000
    
    @staticmethod
    def balanced():
        """平衡配置（默认）"""
        DetectionConfig.CANNY_THRESH1 = 50
        DetectionConfig.CANNY_THRESH2 = 150
        DetectionConfig.APPROX_EPSILON = 0.04
        DetectionConfig.FILTER_ALPHA = 0.3
        DetectionConfig.MIN_AREA = 1500
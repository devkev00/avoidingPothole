"""
포트홀 회피 시뮬레이션 설정 파일
"""

# 화면 설정
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
FPS = 60

# 색상 정의
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
DARK_GRAY = (64, 64, 64)
LIGHT_GRAY = (192, 192, 192)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
CYAN = (0, 255, 255)

# 도로 설정
LANE_WIDTH = 80  # 차선 폭 (픽셀)
NUM_LANES = 3    # 차선 수
ROAD_LENGTH = 5000  # 도로 길이

# 차량 설정
VEHICLE_LENGTH = 80
VEHICLE_WIDTH = 40
WHEEL_BASE = 60  # 앞뒤 바퀴 간격
TRACK_WIDTH = 35  # 좌우 바퀴 간격

# 차량 동역학 (단순화)
MAX_SPEED = 200  # 최대 속도 (픽셀/초)
MIN_SPEED = 50   # 최소 속도
ACCELERATION = 100  # 가속도
DECELERATION = 150  # 감속도
MAX_STEERING_ANGLE = 0.5  # 최대 조향각 (라디안, 약 28도)

# Pure Pursuit 제어기 설정
LOOKAHEAD_GAIN = 0.3  # 속도 대비 룩어헤드 거리 비율
MIN_LOOKAHEAD = 50    # 최소 룩어헤드 거리
MAX_LOOKAHEAD = 150   # 최대 룩어헤드 거리

# 센서 설정
SENSOR_RANGE = 300  # 포트홀 감지 범위 (픽셀)

# 포트홀 설정
POTHOLE_MIN_RADIUS = 15
POTHOLE_MAX_RADIUS = 40

# 경로 계획 설정
GRID_RESOLUTION = 10  # A* 그리드 해상도
SAFETY_MARGIN = 20    # 포트홀 주변 안전 마진

# 미니맵 설정
MINIMAP_WIDTH = 200
MINIMAP_HEIGHT = 150
MINIMAP_POS = (10, 10)  # 왼쪽 상단

# 디버그 모드
DEBUG_MODE = True  # True면 경로, 센서 범위 등 표시
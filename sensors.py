"""
가상 센서 시스템 - 포트홀 탐지
"""
import pygame
import math
from config import *


class PotholeSensor:
    def __init__(self, vehicle):
        """
        포트홀 센서 초기화

        Args:
            vehicle: Vehicle 객체
        """
        self.vehicle = vehicle
        self.detection_range = SENSOR_RANGE
        self.detected_potholes = []

    def detect_potholes(self, potholes):
        """
        범위 내 포트홀 탐지

        Args:
            potholes: Pothole 객체 리스트

        Returns:
            list: 탐지된 Pothole 객체 리스트 (거리순 정렬)
        """
        self.detected_potholes = []

        # 차량 앞 방향 벡터
        front_x = self.vehicle.x + self.detection_range * math.cos(self.vehicle.angle)
        front_y = self.vehicle.y + self.detection_range * math.sin(self.vehicle.angle)

        for pothole in potholes:
            # 차량으로부터의 거리 계산
            distance = math.sqrt(
                (pothole.x - self.vehicle.x) ** 2 +
                (pothole.y - self.vehicle.y) ** 2
            )

            if distance <= self.detection_range:
                # 포트홀이 차량 진행 방향 앞쪽에 있는지 확인
                # 차량 방향 벡터와 포트홀 방향 벡터의 내적으로 판단
                to_pothole_x = pothole.x - self.vehicle.x
                to_pothole_y = pothole.y - self.vehicle.y

                direction_x = math.cos(self.vehicle.angle)
                direction_y = math.sin(self.vehicle.angle)

                dot_product = to_pothole_x * direction_x + to_pothole_y * direction_y

                # 앞쪽에 있으면 탐지 (내적 > 0)
                if dot_product > 0:
                    self.detected_potholes.append({
                        'pothole': pothole,
                        'distance': distance
                    })

        # 거리순 정렬
        self.detected_potholes.sort(key=lambda x: x['distance'])

        return self.detected_potholes

    def get_nearest_pothole(self):
        """
        가장 가까운 포트홀 반환

        Returns:
            dict 또는 None: {'pothole': Pothole, 'distance': float}
        """
        if self.detected_potholes:
            return self.detected_potholes[0]
        return None

    def draw(self, screen, camera):
        """
        센서 범위 및 감지 포트홀 시각화

        Args:
            screen: Pygame surface
            camera: Camera 객체
        """
        if not DEBUG_MODE:
            return

        # 센서 범위를 부채꼴로 표시
        screen_pos = camera.world_to_screen((self.vehicle.x, self.vehicle.y))

        # 센서 범위 원 그리기
        pygame.draw.circle(screen, (0, 255, 0, 50), screen_pos,
                           int(self.detection_range), 2)

        # 탐지된 포트홀 시각화
        for i, detected in enumerate(self.detected_potholes):
            pothole = detected['pothole']
            distance = detected['distance']
            pothole_screen = camera.world_to_screen((pothole.x, pothole.y))

            # 거리에 따른 색상 (가까울수록 빨강)
            if distance < 100:
                line_color = RED
                line_width = 3
            elif distance < 200:
                line_color = ORANGE
                line_width = 2
            else:
                line_color = YELLOW
                line_width = 1

            # 차량과 포트홀 연결선
            pygame.draw.line(screen, line_color, screen_pos, pothole_screen, line_width)

            # 가장 가까운 포트홀에 표시
            if i == 0:
                # 거리 텍스트
                font = pygame.font.Font(None, 18)
                dist_text = font.render(f"{int(distance)}px", True, WHITE)
                text_pos = (pothole_screen[0] + 15, pothole_screen[1] - 10)
                screen.blit(dist_text, text_pos)
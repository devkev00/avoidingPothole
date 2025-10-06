"""
NPC 차량 - 주변 교통 시뮬레이션
"""
import pygame
import math
import random
from config import *


class NPCVehicle:
    def __init__(self, x, y, lane_id, road, speed=None):
        """
        NPC 차량 초기화

        Args:
            x, y: 초기 위치
            lane_id: 차선 번호
            road: Road 객체
            speed: 속도 (None이면 랜덤)
        """
        self.x = x
        self.y = y
        self.angle = 0
        self.lane_id = lane_id
        self.road = road

        # 속도 설정 (랜덤 또는 지정)
        if speed is None:
            self.target_speed = random.uniform(60, 140)  # 60~140 px/s
        else:
            self.target_speed = speed

        self.speed = self.target_speed  # 현재 속도

        # 차량 크기
        self.length = VEHICLE_LENGTH * 0.9  # 플레이어보다 약간 작게
        self.width = VEHICLE_WIDTH * 0.9
        self.wheel_base = WHEEL_BASE * 0.9

        # 경로 추종
        self.path_update_distance = 100  # 경로 업데이트 간격
        self.last_path_update_x = x

        # ACC 파라미터
        self.front_vehicle = None  # 전방 차량
        self.safe_distance = VEHICLE_LENGTH * 1.5  # 안전 거리

    def update(self, dt, front_vehicle=None):
        """
        NPC 차량 업데이트

        Args:
            dt: 시간 간격 (초)
            front_vehicle: 전방 차량 (Vehicle 또는 NPCVehicle)
        """
        # 속도 조정 (ACC)
        self._adjust_speed(front_vehicle, dt)

        # 도로 중심선을 따라 이동
        current_lane_y = self.road.get_lane_center_at_x(self.x, self.lane_id)

        if current_lane_y is not None:
            # 목표 y 위치로 부드럽게 이동
            dy = current_lane_y - self.y
            self.y += dy * 0.1  # 부드러운 보정

        # 앞으로 이동
        self.x += self.speed * dt

        # 각도 계산 (도로 방향)
        future_x = self.x + 50
        future_y = self.road.get_lane_center_at_x(future_x, self.lane_id)

        if future_y is not None:
            dx = future_x - self.x
            dy = future_y - self.y
            self.angle = math.atan2(dy, dx)

    def _adjust_speed(self, front_vehicle, dt):
        """
        전방 차량에 따른 속도 조정 (ACC)

        Args:
            front_vehicle: 전방 차량
            dt: 시간 간격
        """
        if front_vehicle is None:
            # 전방 차량 없음 - 목표 속도로 복귀
            target_speed = self.target_speed
        else:
            # 전방 차량과의 거리 계산
            distance = front_vehicle.x - self.x

            # 시간 기반 안전 거리 (2초 법칙)
            time_gap = 2.0
            safe_distance = self.speed * time_gap + self.length * 1.5
            min_safe_distance = self.length * 1.2

            if distance < min_safe_distance:
                # 너무 가까움 - 급감속
                target_speed = front_vehicle.speed * 0.6
            elif distance < safe_distance:
                # 안전 거리 이하 - 감속
                speed_factor = (distance - min_safe_distance) / (safe_distance - min_safe_distance)
                target_speed = front_vehicle.speed * (0.6 + 0.4 * speed_factor)
            else:
                # 안전 거리 확보 - 순항
                target_speed = self.target_speed

        # 부드러운 가속/감속
        acceleration = 100  # px/s^2
        speed_diff = target_speed - self.speed

        if abs(speed_diff) < acceleration * dt:
            self.speed = target_speed
        else:
            if speed_diff > 0:
                self.speed += acceleration * dt
            else:
                self.speed -= acceleration * dt

        # 속도 제한
        self.speed = max(0, min(self.speed, 150))

    def get_bounding_box(self):
        """
        차량의 바운딩 박스 반환 (충돌 감지용)

        Returns:
            dict: {'x': float, 'y': float, 'width': float, 'height': float}
        """
        return {
            'x': self.x - self.length / 2,
            'y': self.y - self.width / 2,
            'width': self.length,
            'height': self.width
        }

    def check_collision_with_vehicle(self, other_vehicle):
        """
        다른 차량과의 충돌 체크

        Args:
            other_vehicle: Vehicle 또는 NPCVehicle 객체

        Returns:
            bool: 충돌 여부
        """
        # 간단한 원형 충돌 감지
        distance = math.sqrt(
            (self.x - other_vehicle.x) ** 2 +
            (self.y - other_vehicle.y) ** 2
        )

        collision_distance = (self.length + other_vehicle.length) / 2 * 0.8

        return distance < collision_distance

    def is_in_danger_zone(self, target_x, target_y, radius):
        """
        특정 위치가 차량의 위험 지대에 있는지 확인

        Args:
            target_x, target_y: 확인할 위치
            radius: 위험 반경

        Returns:
            bool: 위험 지대 포함 여부
        """
        distance = math.sqrt(
            (self.x - target_x) ** 2 +
            (self.y - target_y) ** 2
        )

        return distance < radius + self.length

    def draw(self, screen, camera):
        """
        NPC 차량 그리기

        Args:
            screen: Pygame surface
            camera: Camera 객체
        """
        # 카메라 좌표로 변환
        screen_pos = camera.world_to_screen((self.x, self.y))

        # 차량 몸체를 사각형으로 그리기
        cos_a = math.cos(self.angle)
        sin_a = math.sin(self.angle)

        # 차량의 네 모서리 계산
        half_length = self.length / 2
        half_width = self.width / 2

        corners = [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        ]

        # 회전 및 화면 좌표 변환
        screen_corners = []
        for cx, cy in corners:
            world_x = self.x + cx * cos_a - cy * sin_a
            world_y = self.y + cx * sin_a + cy * cos_a
            screen_corners.append(camera.world_to_screen((world_x, world_y)))

        # 차량 그리기 (회색 계열)
        pygame.draw.polygon(screen, GRAY, screen_corners)
        pygame.draw.polygon(screen, LIGHT_GRAY, screen_corners, 2)

        # 디버그 모드: 속도 표시
        if DEBUG_MODE:
            font = pygame.font.Font(None, 16)
            speed_text = font.render(f"{int(self.speed)}", True, WHITE)
            text_pos = (screen_pos[0] - 10, screen_pos[1] - 25)
            screen.blit(speed_text, text_pos)

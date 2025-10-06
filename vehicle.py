"""
차량 모델 - Ackermann 조향 기하학 구현
"""
import pygame
import math
from config import *


class Vehicle:
    def __init__(self, x, y, angle=0):
        """
        차량 초기화

        Args:
            x, y: 초기 위치 (월드 좌표)
            angle: 초기 방향 (라디안)
        """
        self.x = x
        self.y = y
        self.angle = angle  # 차량의 방향 (0 = 오른쪽, π/2 = 위)

        self.speed = 0  # 현재 속도
        self.steering_angle = 0  # 현재 조향각

        # 차량 크기
        self.length = VEHICLE_LENGTH
        self.width = VEHICLE_WIDTH
        self.wheel_base = WHEEL_BASE
        self.track_width = TRACK_WIDTH

        # 제어 입력
        self.target_speed = MIN_SPEED
        self.target_steering = 0

    def update(self, dt):
        """
        차량 상태 업데이트 (Ackermann 조향 모델)

        Args:
            dt: 시간 간격 (초)
        """
        # 속도 업데이트 (간단한 PID 제어)
        speed_diff = self.target_speed - self.speed
        if speed_diff > 0:
            self.speed += min(ACCELERATION * dt, speed_diff)
        else:
            self.speed += max(-DECELERATION * dt, speed_diff)

        # 조향각 업데이트 (부드러운 전환)
        steering_diff = self.target_steering - self.steering_angle
        max_change = 2.0 * dt  # 최대 조향 변화율
        self.steering_angle += max(min(steering_diff, max_change), -max_change)

        # 조향각 제한
        self.steering_angle = max(min(self.steering_angle, MAX_STEERING_ANGLE),
                                  -MAX_STEERING_ANGLE)

        # Ackermann 조향 모델에 따른 위치 업데이트
        if abs(self.steering_angle) > 0.001:
            # 회전 반경 계산
            turning_radius = self.wheel_base / math.tan(self.steering_angle)

            # 각속도 계산
            angular_velocity = self.speed / turning_radius

            # 위치 및 방향 업데이트
            self.angle += angular_velocity * dt
            self.x += self.speed * math.cos(self.angle) * dt
            self.y += self.speed * math.sin(self.angle) * dt
        else:
            # 직진
            self.x += self.speed * math.cos(self.angle) * dt
            self.y += self.speed * math.sin(self.angle) * dt

    def get_wheel_positions(self):
        """
        4개 바퀴의 월드 좌표 반환

        Returns:
            list: [(x, y), ...] 형식의 4개 바퀴 위치
        """
        # 차량 중심 기준 바퀴 위치 (로컬 좌표)
        wheels_local = [
            (self.wheel_base / 2, self.track_width / 2),  # 앞-우
            (self.wheel_base / 2, -self.track_width / 2),  # 앞-좌
            (-self.wheel_base / 2, self.track_width / 2),  # 뒤-우
            (-self.wheel_base / 2, -self.track_width / 2),  # 뒤-좌
        ]

        # 회전 변환 적용
        cos_a = math.cos(self.angle)
        sin_a = math.sin(self.angle)

        wheels_world = []
        for wx, wy in wheels_local:
            # 회전 행렬 적용
            rx = wx * cos_a - wy * sin_a
            ry = wx * sin_a + wy * cos_a
            wheels_world.append((self.x + rx, self.y + ry))

        return wheels_world

    def get_front_axle_center(self):
        """
        앞 차축 중심점 반환 (Pure Pursuit용)
        """
        offset_x = (self.wheel_base / 2) * math.cos(self.angle)
        offset_y = (self.wheel_base / 2) * math.sin(self.angle)
        return (self.x + offset_x, self.y + offset_y)

    def draw(self, screen, camera):
        """
        차량 그리기

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

        # 차량 그리기
        pygame.draw.polygon(screen, BLUE, screen_corners)
        pygame.draw.polygon(screen, CYAN, screen_corners, 2)

        # 디버그 모드: 바퀴 위치 표시
        if DEBUG_MODE:
            wheels = self.get_wheel_positions()
            for wx, wy in wheels:
                screen_wheel = camera.world_to_screen((wx, wy))
                pygame.draw.circle(screen, RED, screen_wheel, 5)

            # 차량 방향 표시 (화살표)
            arrow_end = camera.world_to_screen((
                self.x + 50 * cos_a,
                self.y + 50 * sin_a
            ))
            pygame.draw.line(screen, YELLOW, screen_pos, arrow_end, 3)
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
        # 조향각 업데이트 (부드러운 전환)
        steering_diff = self.target_steering - self.steering_angle
        max_change = 0.8 * dt  # 최대 조향 변화율 (1.2 -> 0.8로 감소)
        self.steering_angle += max(min(steering_diff, max_change), -max_change)

        # 조향각 제한
        self.steering_angle = max(min(self.steering_angle, MAX_STEERING_ANGLE),
                                  -MAX_STEERING_ANGLE)

        # 속도 업데이트 (간단한 PID 제어)
        speed_diff = self.target_speed - self.speed
        if speed_diff > 0:
            self.speed += min(ACCELERATION * dt, speed_diff)
        else:
            self.speed += max(-DECELERATION * dt, speed_diff)

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

    def get_corners(self):
        """
        차량의 네 모서리 좌표 반환 (충돌 감지용)

        Returns:
            list: [(x, y), ...] 네 모서리 좌표
        """
        cos_a = math.cos(self.angle)
        sin_a = math.sin(self.angle)

        half_length = self.length / 2
        half_width = self.width / 2

        corners_local = [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        ]

        corners_world = []
        for cx, cy in corners_local:
            world_x = self.x + cx * cos_a - cy * sin_a
            world_y = self.y + cx * sin_a + cy * cos_a
            corners_world.append((world_x, world_y))

        return corners_world

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

        # 바퀴 그리기 (직사각형)
        self._draw_wheels(screen, camera)

        # 디버그 모드: 차량 방향 표시 (화살표)
        if DEBUG_MODE:
            arrow_end = camera.world_to_screen((
                self.x + 50 * cos_a,
                self.y + 50 * sin_a
            ))
            pygame.draw.line(screen, YELLOW, screen_pos, arrow_end, 3)

    def _draw_wheels(self, screen, camera):
        """
        바퀴를 직사각형으로 그리기

        Args:
            screen: Pygame surface
            camera: Camera 객체
        """
        # 바퀴 크기
        wheel_length = 12  # 바퀴 길이 (진행 방향)
        wheel_width = 6    # 바퀴 폭 (측면)

        wheels = self.get_wheel_positions()
        cos_a = math.cos(self.angle)
        sin_a = math.sin(self.angle)

        for i, (wx, wy) in enumerate(wheels):
            # 바퀴 조향각 계산 (앞바퀴만 조향)
            if i in [0, 1]:  # 앞바퀴
                wheel_angle = self.angle + self.steering_angle
            else:  # 뒷바퀴
                wheel_angle = self.angle

            # 바퀴의 네 모서리 계산 (로컬 좌표)
            half_length = wheel_length / 2
            half_width = wheel_width / 2

            wheel_cos = math.cos(wheel_angle)
            wheel_sin = math.sin(wheel_angle)

            wheel_corners_local = [
                (half_length, half_width),
                (half_length, -half_width),
                (-half_length, -half_width),
                (-half_length, half_width),
            ]

            # 회전 및 화면 좌표 변환
            wheel_screen_corners = []
            for cx, cy in wheel_corners_local:
                world_x = wx + cx * wheel_cos - cy * wheel_sin
                world_y = wy + cx * wheel_sin + cy * wheel_cos
                wheel_screen_corners.append(camera.world_to_screen((world_x, world_y)))

            # 바퀴 그리기
            pygame.draw.polygon(screen, BLACK, wheel_screen_corners)
            pygame.draw.polygon(screen, DARK_GRAY, wheel_screen_corners, 1)
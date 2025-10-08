"""
포트홀 객체
"""
import pygame
import math
from config import *


class Pothole:
    _id_counter = 0  # 클래스 변수로 고유 ID 생성

    def __init__(self, x, y, radius):
        """
        포트홀 초기화

        Args:
            x, y: 중심 위치
            radius: 반경
        """
        self.x = x
        self.y = y
        self.radius = radius

        # 고유 ID 부여
        Pothole._id_counter += 1
        self.id = Pothole._id_counter

    def check_collision(self, wheel_positions):
        """
        바퀴와 포트홀의 충돌 검사

        Args:
            wheel_positions: [(x, y), ...] 바퀴 위치 리스트

        Returns:
            dict: {
                'collision': bool,
                'wheel_index': int (충돌한 바퀴 인덱스, 없으면 -1),
                'distance': float (가장 가까운 바퀴까지의 거리)
            }
        """
        min_distance = float('inf')
        collision_wheel = -1

        for i, (wx, wy) in enumerate(wheel_positions):
            distance = math.sqrt((wx - self.x) ** 2 + (wy - self.y) ** 2)

            if distance < min_distance:
                min_distance = distance

            # 바퀴가 포트홀에 들어갔는지 확인 (경계선 접촉은 허용)
            if distance < self.radius - 3:  # 경계선 닿는 것은 OK, 3px 안으로 들어가야 충돌
                collision_wheel = i

        return {
            'collision': collision_wheel != -1,
            'wheel_index': collision_wheel,
            'distance': min_distance
        }

    def classify_position(self, vehicle):
        """
        포트홀이 차량의 어느 위치에 있는지 분류

        Args:
            vehicle: Vehicle 객체

        Returns:
            str: 'SAFE_PASS', 'AVOID_LEFT', 'AVOID_RIGHT', 'AVOID_CENTER'
        """
        wheels = vehicle.get_wheel_positions()

        # 각 바퀴와 포트홀 간의 거리 계산
        distances = []
        for wx, wy in wheels:
            dist = math.sqrt((wx - self.x) ** 2 + (wy - self.y) ** 2)
            distances.append(dist)

        min_distance = min(distances)
        min_wheel_idx = distances.index(min_distance)

        # 안전 마진 고려
        safety_threshold = self.radius + SAFETY_MARGIN

        # 모든 바퀴가 안전 거리 밖이면 SAFE_PASS
        if all(d > safety_threshold for d in distances):
            return 'SAFE_PASS'

        # 바퀴 인덱스: 0=앞우, 1=앞좌, 2=뒤우, 3=뒤좌
        if min_wheel_idx in [1, 3]:  # 좌측 바퀴
            return 'AVOID_RIGHT'
        elif min_wheel_idx in [0, 2]:  # 우측 바퀴
            return 'AVOID_LEFT'
        else:
            return 'AVOID_CENTER'

    def is_between_wheels(self, vehicle):
        """
        포트홀이 바퀴 사이에 안전하게 위치하는지 확인

        Returns:
            bool: 바퀴 사이 안전하게 있으면 True (회피 불필요)
        """
        wheels = vehicle.get_wheel_positions()

        # 바퀴 간격 계산 (좌우 바퀴 사이 거리)
        from config import TRACK_WIDTH
        wheel_gap = TRACK_WIDTH

        # 1. 포트홀 크기 확인: 바퀴 간격의 80%보다 작으면 통과 가능성 있음
        max_safe_radius = wheel_gap * 0.8  # 바퀴 간격의 80%
        if self.radius > max_safe_radius:
            return False  # 너무 큰 포트홀 - 회피 필요

        # 2. 포트홀이 바퀴 경로와 겹치는지 확인
        # 바퀴 인덱스: 0=앞우, 1=앞좌, 2=뒤우, 3=뒤좌
        front_right = wheels[0]
        front_left = wheels[1]
        rear_right = wheels[2]
        rear_left = wheels[3]

        # 차량 좌표계로 변환 (전방 방향을 x축으로)
        cos_a = math.cos(-vehicle.angle)
        sin_a = math.sin(-vehicle.angle)

        # 포트홀의 차량 좌표계 위치
        dx = self.x - vehicle.x
        dy = self.y - vehicle.y
        local_x = dx * cos_a - dy * sin_a
        local_y = dx * sin_a + dy * cos_a

        # 좌우 바퀴의 차량 좌표계 y 위치 계산
        left_wheel_y = TRACK_WIDTH / 2
        right_wheel_y = -TRACK_WIDTH / 2

        # 안전 마진 포함한 바퀴 경로 폭 (경계선 접촉 허용)
        safety_margin = -3  # 음수로 설정하여 경계선 접촉 허용
        left_path_boundary = left_wheel_y + safety_margin
        right_path_boundary = right_wheel_y - safety_margin

        # 포트홀이 바퀴 경로 범위 내에 있는지 확인
        pothole_left = local_y + self.radius
        pothole_right = local_y - self.radius

        # 포트홀이 바퀴 경로와 겹치면 회피 필요
        if pothole_right < left_path_boundary and pothole_left > right_path_boundary:
            # 포트홀이 좌우 바퀴 경로에 걸쳐 있음
            # 중앙에 있으면 통과 가능
            if abs(local_y) < wheel_gap * 0.3:  # 중앙 30% 범위 (더 관대하게)
                return True
            else:
                return False  # 중앙이 아니면 회피 필요

        # 바퀴 경로 밖에 있음 - 통과 가능
        return True

    def draw(self, screen, camera, is_detected=False, vehicle=None):
        """
        포트홀 그리기

        Args:
            screen: Pygame surface
            camera: Camera 객체
            is_detected: 센서에 감지되었는지 여부
            vehicle: Vehicle 객체 (통과 가능 여부 표시용)
        """
        screen_pos = camera.world_to_screen((self.x, self.y))

        # 포트홀 그리기 (어두운 원)
        pygame.draw.circle(screen, BLACK, screen_pos, int(self.radius))

        # 감지 여부에 따른 테두리 색상
        if is_detected:
            # 크기에 따른 위험도 표시
            if self.radius > 30:
                border_color = RED  # 큰 포트홀 (차선 변경 필요)
                border_width = 4
            else:
                border_color = ORANGE  # 중간 포트홀 (차선 내 회피)
                border_width = 3
        else:
            border_color = (100, 100, 100)  # 미감지
            border_width = 2

        pygame.draw.circle(screen, border_color, screen_pos, int(self.radius), border_width)

        # 디버그 모드: 상세 정보 표시
        if DEBUG_MODE:
            safety_radius = int(self.radius + SAFETY_MARGIN)
            pygame.draw.circle(screen, (255, 100, 0), screen_pos, safety_radius, 1)

            # 포트홀 크기 및 통과 가능 여부 텍스트
            font = pygame.font.Font(None, 16)

            # 통과 가능 여부 확인
            can_pass = False
            if vehicle is not None and is_detected:
                can_pass = self.is_between_wheels(vehicle)

            # 텍스트 색상
            if can_pass:
                text_color = (0, 255, 0)  # 녹색 - 통과 가능
                status = "PASS"
            else:
                text_color = WHITE
                status = f"R:{int(self.radius)}"

            size_text = font.render(status, True, text_color)
            text_pos = (screen_pos[0] - 15, screen_pos[1] + int(self.radius) + 5)
            screen.blit(size_text, text_pos)
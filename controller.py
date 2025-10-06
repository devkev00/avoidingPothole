"""
Pure Pursuit 제어기 - 경로 추종
"""
import pygame
import math
from config import *


class PurePursuitController:
    def __init__(self, vehicle):
        """
        Pure Pursuit 제어기 초기화

        Args:
            vehicle: Vehicle 객체
        """
        self.vehicle = vehicle
        self.path = []  # 추종할 경로 [(x, y), ...]
        self.current_target_idx = 0

    def set_path(self, path):
        """
        추종할 경로 설정

        Args:
            path: [(x, y), ...] 경로 포인트 리스트
        """
        self.path = path
        self.current_target_idx = 0

    def get_lookahead_distance(self):
        """
        속도에 따른 룩어헤드 거리 계산

        Returns:
            float: 룩어헤드 거리
        """
        # 속도에 비례한 룩어헤드 거리
        ld = LOOKAHEAD_GAIN * self.vehicle.speed
        return max(MIN_LOOKAHEAD, min(ld, MAX_LOOKAHEAD))

    def find_target_point(self):
        """
        룩어헤드 거리에 있는 목표 지점 찾기

        Returns:
            (x, y) 또는 None: 목표 지점 좌표
        """
        if not self.path:
            return None

        lookahead = self.get_lookahead_distance()

        # 차량의 뒷바퀴 중심 (Pure Pursuit의 기준점)
        vehicle_pos = (self.vehicle.x, self.vehicle.y)

        # 경로 상에서 룩어헤드 거리보다 먼 점 중 가장 가까운 점 찾기
        best_point = None
        min_diff = float('inf')

        for i in range(self.current_target_idx, len(self.path)):
            px, py = self.path[i]
            distance = math.sqrt(
                (px - vehicle_pos[0]) ** 2 +
                (py - vehicle_pos[1]) ** 2
            )

            # 차량보다 앞쪽에 있는 점만 고려 (뒤쪽 경로 포인트 무시)
            if px <= vehicle_pos[0]:
                continue

            # 룩어헤드 거리 이상인 점 중 가장 가까운 점 선택
            if distance >= lookahead:
                diff = abs(distance - lookahead)
                if diff < min_diff:
                    min_diff = diff
                    best_point = (px, py)
                    self.current_target_idx = i

        # 적절한 점을 못 찾은 경우, 가장 먼 점 선택
        if best_point is None and len(self.path) > 0:
            # 앞쪽 경로 중 가장 먼 점
            for i in range(len(self.path) - 1, -1, -1):
                if self.path[i][0] > vehicle_pos[0]:
                    best_point = self.path[i]
                    self.current_target_idx = i
                    break

        return best_point

    def calculate_steering_angle(self, target_point):
        """
        목표 지점으로 향하는 조향각 계산 (Pure Pursuit 공식)

        Args:
            target_point: (x, y) 목표 지점

        Returns:
            float: 조향각 (라디안)
        """
        if target_point is None:
            return 0.0

        # 차량 좌표계에서 목표 지점의 위치 계산
        dx = target_point[0] - self.vehicle.x
        dy = target_point[1] - self.vehicle.y

        # 차량 방향으로 회전 (좌표 변환)
        cos_a = math.cos(-self.vehicle.angle)
        sin_a = math.sin(-self.vehicle.angle)

        local_x = dx * cos_a - dy * sin_a
        local_y = dx * sin_a + dy * cos_a

        # 목표 지점까지의 거리
        ld = math.sqrt(local_x ** 2 + local_y ** 2)

        if ld < 0.1:  # 목표에 거의 도달
            return 0.0

        # Pure Pursuit 공식
        # curvature = 2 * local_y / ld^2
        # steering_angle = atan(wheelbase * curvature)

        curvature = 2.0 * local_y / (ld * ld)
        steering_angle = math.atan(self.vehicle.wheel_base * curvature)

        return steering_angle

    def update(self):
        """
        제어기 업데이트 - 조향각 계산 및 적용
        """
        if not self.path:
            return

        # 목표 지점 찾기
        target_point = self.find_target_point()

        if target_point is None:
            return

        # 조향각 계산
        steering_angle = self.calculate_steering_angle(target_point)

        # 차량에 조향각 적용
        self.vehicle.target_steering = steering_angle

    def is_path_complete(self):
        """
        경로 추종 완료 여부 확인

        Returns:
            bool: 경로 끝에 도달했으면 True
        """
        if not self.path:
            return True

        # 마지막 경로 지점과의 거리 확인
        last_point = self.path[-1]
        distance = math.sqrt(
            (last_point[0] - self.vehicle.x) ** 2 +
            (last_point[1] - self.vehicle.y) ** 2
        )

        return distance < 30  # 30픽셀 이내면 도달로 간주

    def draw(self, screen, camera):
        """
        경로 및 목표 지점 시각화

        Args:
            screen: Pygame surface
            camera: Camera 객체
        """
        if not DEBUG_MODE or not self.path:
            return

        # 경로 그리기
        if len(self.path) > 1:
            screen_path = [camera.world_to_screen(p) for p in self.path]
            pygame.draw.lines(screen, CYAN, False, screen_path, 2)

        # 목표 지점 표시
        target = self.find_target_point()
        if target:
            screen_target = camera.world_to_screen(target)
            pygame.draw.circle(screen, GREEN, screen_target, 10)
            pygame.draw.circle(screen, YELLOW, screen_target, 5)

            # 차량에서 목표까지 선 그리기
            vehicle_screen = camera.world_to_screen((self.vehicle.x, self.vehicle.y))
            pygame.draw.line(screen, GREEN, vehicle_screen, screen_target, 2)
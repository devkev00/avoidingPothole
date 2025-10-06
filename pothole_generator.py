"""
포트홀 자동 생성 시스템
"""
import random
from pothole import Pothole
from config import *


class PotholeGenerator:
    def __init__(self, road):
        """
        포트홀 생성기 초기화

        Args:
            road: Road 객체
        """
        self.road = road
        self.potholes = []
        self.last_generated_x = 0
        self.generation_interval = 200  # 포트홀 생성 간격 (픽셀) - 250에서 감소
        self.spawn_probability = 0.5  # 각 구간에 포트홀이 생성될 확률 - 0.4에서 증가

    def generate_potholes(self, vehicle_x):
        """
        차량 위치에 따라 앞쪽에 포트홀 생성

        Args:
            vehicle_x: 차량의 x 좌표
        """
        # 차량 앞쪽 1000픽셀까지 포트홀 생성
        target_x = vehicle_x + 1500

        while self.last_generated_x < target_x:
            # 포트홀 생성 여부 결정
            if random.random() < self.spawn_probability:
                self._spawn_pothole(self.last_generated_x)

            self.last_generated_x += self.generation_interval

    def _spawn_pothole(self, x):
        """
        특정 x 위치에 포트홀 생성

        크기: 바퀴 사이 간격의 50% ~ 차선 간격의 50%
        위치: 차선 내 왼쪽, 가운데, 오른쪽 중 랜덤

        Args:
            x: 포트홀의 x 좌표
        """
        # 2번 차선에만 생성
        lane_id = 2

        # 해당 차선의 중심 y 좌표
        lane_center_y = self.road.get_lane_center_at_x(x, lane_id)

        if lane_center_y is None:
            return

        # 포트홀 크기: 바퀴 간격의 50% ~ 차선 폭의 50%
        from config import TRACK_WIDTH
        min_radius = TRACK_WIDTH * 0.25  # 바퀴 간격의 50% (직경 기준이므로 반경은 25%)
        max_radius = self.road.lane_width * 0.25  # 차선 폭의 50% (직경 기준이므로 반경은 25%)

        # 랜덤 크기 선택
        radius = random.uniform(min_radius, max_radius)

        # 위치 선택 (왼쪽, 가운데, 오른쪽)
        position = random.choice(['left', 'center', 'right'])

        # 위치에 따른 오프셋 계산
        if position == 'left':
            # 차선 왼쪽 (차선 폭의 25% 왼쪽)
            y_offset = -self.road.lane_width * 0.25
        elif position == 'center':
            # 차선 중앙
            y_offset = 0
        else:  # right
            # 차선 오른쪽 (차선 폭의 25% 오른쪽)
            y_offset = self.road.lane_width * 0.25

        y = lane_center_y + y_offset

        # 포트홀 생성
        pothole = Pothole(x, y, radius)
        self.potholes.append(pothole)

    def cleanup_old_potholes(self, vehicle_x):
        """
        차량 뒤쪽의 오래된 포트홀 제거 (메모리 절약)

        Args:
            vehicle_x: 차량의 x 좌표
        """
        # 차량 뒤쪽 500픽셀 이전의 포트홀 제거
        self.potholes = [
            p for p in self.potholes
            if p.x > vehicle_x - 500
        ]

    def get_potholes(self):
        """
        현재 생성된 모든 포트홀 반환

        Returns:
            list: Pothole 객체 리스트
        """
        return self.potholes

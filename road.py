"""
도로 환경 - 무한 생성 곡선 도로 구현
"""
import pygame
import math
import random
from config import *


class Road:
    def __init__(self):
        """
        도로 초기화 - 곡선 도로 무한 생성
        """
        self.lane_width = LANE_WIDTH
        self.num_lanes = NUM_LANES
        self.total_width = self.lane_width * self.num_lanes

        # 도로 중심선 (무한 생성)
        self.centerline = []
        self.segment_length = 100  # 세그먼트 길이
        self.last_x = 0
        self.last_y = 400  # 시작 y 위치

        # 곡률 제어
        self.current_curve = 0  # 현재 곡률 각도
        self.curve_change_distance = 500  # 곡률 변경 거리
        self.distance_since_curve_change = 0

        # 초기 도로 생성 (5000픽셀)
        self._generate_road_segments(5000)

        # 각 차선의 기본 오프셋 (중심선 기준)
        self.lane_offsets = []
        for i in range(self.num_lanes):
            offset_y = -self.total_width / 2 + self.lane_width / 2 + i * self.lane_width
            self.lane_offsets.append(offset_y)

    def _generate_road_segments(self, length):
        """
        도로 세그먼트 생성

        Args:
            length: 생성할 도로 길이
        """
        segments_to_generate = int(length / self.segment_length)

        for _ in range(segments_to_generate):
            # 곡률 변경 체크
            if self.distance_since_curve_change >= self.curve_change_distance:
                # 새로운 곡률 설정 (과한 커브)
                self.current_curve = random.uniform(-0.4, 0.4)  # 라디안 (-0.15 -> -0.4로 증가)
                self.curve_change_distance = random.randint(250, 600)  # 더 자주 변경
                self.distance_since_curve_change = 0

            # 다음 포인트 계산
            angle = math.atan2(1, 8) + self.current_curve  # 기본 진행 방향 + 곡률 (기울기 증가)

            self.last_x += self.segment_length * math.cos(angle)
            self.last_y += self.segment_length * math.sin(angle)

            # y 좌표 제한 (화면을 너무 벗어나지 않도록)
            self.last_y = max(250, min(550, self.last_y))

            self.centerline.append((self.last_x, self.last_y))
            self.distance_since_curve_change += self.segment_length

    def extend_road(self, vehicle_x):
        """
        차량 위치에 따라 도로 확장

        Args:
            vehicle_x: 차량의 x 좌표
        """
        # 차량이 도로 끝에서 2000픽셀 이내로 접근하면 도로 확장
        if self.centerline:
            last_point_x = self.centerline[-1][0]

            if vehicle_x > last_point_x - 2000:
                # 1000픽셀 추가 생성
                self._generate_road_segments(1000)

    def get_road_point_at_x(self, x):
        """
        특정 x 좌표에서 도로 중심선의 y 좌표 반환 (보간)

        Args:
            x: x 좌표

        Returns:
            float: y 좌표
        """
        if not self.centerline:
            return 400

        # x보다 큰 첫 번째 포인트 찾기
        for i in range(len(self.centerline) - 1):
            x1, y1 = self.centerline[i]
            x2, y2 = self.centerline[i + 1]

            if x1 <= x <= x2:
                # 선형 보간
                t = (x - x1) / (x2 - x1) if x2 != x1 else 0
                return y1 + (y2 - y1) * t

        # 범위 밖이면 마지막/첫 번째 포인트 반환
        if x < self.centerline[0][0]:
            return self.centerline[0][1]
        return self.centerline[-1][1]

    def get_lane_center_at_x(self, x, lane_id):
        """
        특정 x 좌표에서 특정 차선의 중심 y 좌표 반환

        Args:
            x: x 좌표
            lane_id: 차선 번호 (1부터 시작)

        Returns:
            float: 차선 중심 y 좌표
        """
        if not (1 <= lane_id <= self.num_lanes):
            return None

        road_center_y = self.get_road_point_at_x(x)
        lane_offset = self.lane_offsets[lane_id - 1]

        return road_center_y + lane_offset

    def get_path_for_lane(self, lane_id, start_x, length):
        """
        특정 차선을 따라가는 경로 생성

        Args:
            lane_id: 차선 번호 (1부터 시작)
            start_x: 시작 x 좌표
            length: 경로 길이

        Returns:
            list: [(x, y), ...] 경로 포인트
        """
        path = []
        step = 20  # 경로 포인트 간격 (50 -> 20으로 감소)

        for x in range(int(start_x), int(start_x + length), step):
            y = self.get_lane_center_at_x(x, lane_id)
            if y is not None:
                path.append((x, y))

        return path

    def get_current_lane(self, x, y):
        """
        주어진 위치가 속한 차선 번호 반환

        Args:
            x, y: 위치 좌표

        Returns:
            lane_id 또는 None (도로 밖)
        """
        road_center_y = self.get_road_point_at_x(x)

        # 각 차선 확인
        for i in range(self.num_lanes):
            lane_center_y = road_center_y + self.lane_offsets[i]
            top = lane_center_y - self.lane_width / 2
            bottom = lane_center_y + self.lane_width / 2

            if top <= y <= bottom:
                return i + 1

        return None

    def is_in_lane(self, x, y):
        """
        주어진 위치가 도로 안인지 확인
        """
        return self.get_current_lane(x, y) is not None

    def get_road_boundaries(self, x):
        """
        특정 x 좌표에서 도로의 상하 경계 y 좌표 반환

        Args:
            x: x 좌표

        Returns:
            tuple: (top_boundary, bottom_boundary) y 좌표
        """
        road_center_y = self.get_road_point_at_x(x)
        half_width = self.total_width / 2

        top_boundary = road_center_y - half_width
        bottom_boundary = road_center_y + half_width

        return (top_boundary, bottom_boundary)

    def clamp_to_road(self, x, y, margin=0):
        """
        y 좌표를 도로 경계 내로 제한

        Args:
            x: x 좌표
            y: y 좌표
            margin: 경계로부터의 안전 마진 (픽셀)

        Returns:
            float: 제한된 y 좌표
        """
        top_boundary, bottom_boundary = self.get_road_boundaries(x)

        # 안전 마진 적용
        top_boundary += margin
        bottom_boundary -= margin

        # y를 경계 내로 제한
        return max(top_boundary, min(bottom_boundary, y))

    def can_change_lane(self, current_lane, direction):
        """
        차선 변경 가능 여부 확인

        Args:
            current_lane: 현재 차선 번호
            direction: +1(위) 또는 -1(아래)

        Returns:
            bool: 차선 변경 가능 여부
        """
        target_lane = current_lane + direction
        return 1 <= target_lane <= self.num_lanes

    def draw(self, screen, camera):
        """
        곡선 도로 그리기

        Args:
            screen: Pygame surface
            camera: Camera 객체
        """
        # 화면에 보이는 x 범위 계산
        visible_x_start = int(camera.offset[0] - 100)
        visible_x_end = int(camera.offset[0] + SCREEN_WIDTH + 100)

        # 도로 배경 그리기 (세그먼트 단위)
        step = 20
        for x in range(visible_x_start, visible_x_end, step):
            road_center_y1 = self.get_road_point_at_x(x)
            road_center_y2 = self.get_road_point_at_x(x + step)

            # 도로 폭 계산
            half_width = self.total_width / 2

            # 사각형의 네 모서리
            top_left = camera.world_to_screen((x, road_center_y1 - half_width))
            top_right = camera.world_to_screen((x + step, road_center_y2 - half_width))
            bottom_right = camera.world_to_screen((x + step, road_center_y2 + half_width))
            bottom_left = camera.world_to_screen((x, road_center_y1 + half_width))

            pygame.draw.polygon(screen, DARK_GRAY, [
                top_left, top_right, bottom_right, bottom_left
            ])

        # 차선 구분선 그리기 (점선)
        for lane_idx in range(1, self.num_lanes):
            dash_length = 30
            gap_length = 20

            x = visible_x_start
            while x < visible_x_end:
                y1 = self.get_road_point_at_x(x) + self.lane_offsets[lane_idx] - self.lane_width / 2
                y2 = self.get_road_point_at_x(x + dash_length) + self.lane_offsets[lane_idx] - self.lane_width / 2

                start = camera.world_to_screen((x, y1))
                end = camera.world_to_screen((x + dash_length, y2))
                pygame.draw.line(screen, WHITE, start, end, 3)
                x += dash_length + gap_length

        # 도로 경계선 그리기 (실선)
        # 위쪽 경계
        for x in range(visible_x_start, visible_x_end, 10):
            y1 = self.get_road_point_at_x(x) - self.total_width / 2
            y2 = self.get_road_point_at_x(x + 10) - self.total_width / 2

            start = camera.world_to_screen((x, y1))
            end = camera.world_to_screen((x + 10, y2))
            pygame.draw.line(screen, YELLOW, start, end, 4)

        # 아래쪽 경계
        for x in range(visible_x_start, visible_x_end, 10):
            y1 = self.get_road_point_at_x(x) + self.total_width / 2
            y2 = self.get_road_point_at_x(x + 10) + self.total_width / 2

            start = camera.world_to_screen((x, y1))
            end = camera.world_to_screen((x + 10, y2))
            pygame.draw.line(screen, YELLOW, start, end, 4)

        # 디버그: 중심선 표시
        if DEBUG_MODE:
            for i in range(len(self.centerline) - 1):
                x1, y1 = self.centerline[i]
                x2, y2 = self.centerline[i + 1]

                # 화면에 보이는 부분만
                if visible_x_start <= x1 <= visible_x_end:
                    start = camera.world_to_screen((x1, y1))
                    end = camera.world_to_screen((x2, y2))
                    pygame.draw.line(screen, RED, start, end, 2)
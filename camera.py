"""
카메라 시스템 - 차량 추적
"""
from config import *


class Camera:
    def __init__(self, target=None):
        """
        카메라 초기화

        Args:
            target: 추적할 Vehicle 객체
        """
        self.target = target
        self.offset = [0, 0]  # 월드 좌표 오프셋
        self.smoothing = 0.1  # 카메라 부드러움 (0~1, 작을수록 부드러움)

    def update(self):
        """
        카메라 위치 업데이트 - 차량을 화면 중앙에 유지
        """
        if self.target is None:
            return

        # 목표 오프셋: 차량을 화면 중앙에 배치
        target_offset_x = self.target.x - SCREEN_WIDTH // 2
        target_offset_y = self.target.y - SCREEN_HEIGHT // 2

        # 부드러운 카메라 이동 (선형 보간)
        self.offset[0] += (target_offset_x - self.offset[0]) * self.smoothing
        self.offset[1] += (target_offset_y - self.offset[1]) * self.smoothing

    def world_to_screen(self, world_pos):
        """
        월드 좌표를 화면 좌표로 변환

        Args:
            world_pos: (x, y) 월드 좌표

        Returns:
            (x, y) 화면 좌표
        """
        screen_x = int(world_pos[0] - self.offset[0])
        screen_y = int(world_pos[1] - self.offset[1])
        return (screen_x, screen_y)

    def screen_to_world(self, screen_pos):
        """
        화면 좌표를 월드 좌표로 변환

        Args:
            screen_pos: (x, y) 화면 좌표

        Returns:
            (x, y) 월드 좌표
        """
        world_x = screen_pos[0] + self.offset[0]
        world_y = screen_pos[1] + self.offset[1]
        return (world_x, world_y)

    def set_target(self, target):
        """
        추적 대상 설정
        """
        self.target = target

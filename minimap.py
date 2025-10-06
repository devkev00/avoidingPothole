"""
미니맵 - 차량 주변 상황 표시 (동적 범위)
"""
import pygame
from config import *


class MiniMap:
    def __init__(self, world_bounds, size=(MINIMAP_WIDTH, MINIMAP_HEIGHT)):
        """
        미니맵 초기화

        Args:
            world_bounds: (min_x, min_y, max_x, max_y) 초기 월드 범위 (사용 안 함)
            size: (width, height) 미니맵 크기
        """
        self.size = size
        self.surface = pygame.Surface(size)
        self.position = MINIMAP_POS

        # 미니맵 표시 범위 (차량 기준)
        self.view_distance = 1000  # 차량 앞뒤로 표시할 거리
        self.view_height = 300    # 위아래로 표시할 높이

    def world_to_minimap(self, world_pos, vehicle_x, vehicle_y):
        """
        월드 좌표를 미니맵 좌표로 변환 (차량 중심 기준)

        Args:
            world_pos: (x, y) 월드 좌표
            vehicle_x, vehicle_y: 차량 위치

        Returns:
            (x, y) 미니맵 좌표 또는 None (범위 밖)
        """
        # 차량 중심 기준 상대 좌표
        rel_x = world_pos[0] - vehicle_x
        rel_y = world_pos[1] - vehicle_y

        # 표시 범위 체크
        if abs(rel_x) > self.view_distance or abs(rel_y) > self.view_height:
            return None

        # 미니맵 좌표로 변환 (차량이 중앙에 위치)
        scale_x = self.size[0] / (2 * self.view_distance)
        scale_y = self.size[1] / (2 * self.view_height)

        minimap_x = int(self.size[0] / 2 + rel_x * scale_x)
        minimap_y = int(self.size[1] / 2 + rel_y * scale_y)

        # 미니맵 범위 내로 제한
        if 0 <= minimap_x < self.size[0] and 0 <= minimap_y < self.size[1]:
            return (minimap_x, minimap_y)

        return None

    def draw(self, screen, vehicle, potholes, road, npc_vehicles=None):
        """
        미니맵 그리기 (차량 중심 기준)

        Args:
            screen: 메인 Pygame surface
            vehicle: Vehicle 객체
            potholes: Pothole 객체 리스트
            road: Road 객체
            npc_vehicles: NPC 차량 리스트 (선택)
        """
        # 미니맵 배경
        self.surface.fill((30, 30, 30))

        # 도로 중심선 그리기
        if road.centerline:
            # 차량 주변의 중심선만 표시
            prev_point = None
            for cx, cy in road.centerline:
                # 차량과의 거리 체크
                if abs(cx - vehicle.x) > self.view_distance:
                    continue

                point = self.world_to_minimap((cx, cy), vehicle.x, vehicle.y)
                if point and prev_point:
                    pygame.draw.line(self.surface, DARK_GRAY, prev_point, point, 8)
                prev_point = point

        # 도로 경계선 그리기 (중심선 기준 ±total_width/2)
        if road.centerline:
            prev_top = None
            prev_bottom = None

            for cx, cy in road.centerline:
                if abs(cx - vehicle.x) > self.view_distance:
                    continue

                # 위쪽 경계
                top_y = cy - road.total_width / 2
                top_point = self.world_to_minimap((cx, top_y), vehicle.x, vehicle.y)

                # 아래쪽 경계
                bottom_y = cy + road.total_width / 2
                bottom_point = self.world_to_minimap((cx, bottom_y), vehicle.x, vehicle.y)

                if top_point and prev_top:
                    pygame.draw.line(self.surface, YELLOW, prev_top, top_point, 2)
                if bottom_point and prev_bottom:
                    pygame.draw.line(self.surface, YELLOW, prev_bottom, bottom_point, 2)

                prev_top = top_point
                prev_bottom = bottom_point

        # 포트홀 그리기
        for pothole in potholes:
            pos = self.world_to_minimap((pothole.x, pothole.y), vehicle.x, vehicle.y)
            if pos:
                # 미니맵에서는 작은 점으로 표시
                pygame.draw.circle(self.surface, ORANGE, pos, 3)

        # NPC 차량 그리기
        if npc_vehicles:
            for npc in npc_vehicles:
                npc_pos = self.world_to_minimap((npc.x, npc.y), vehicle.x, vehicle.y)
                if npc_pos:
                    pygame.draw.circle(self.surface, LIGHT_GRAY, npc_pos, 3)

        # 플레이어 차량 그리기 (항상 중앙)
        vehicle_pos = (self.size[0] // 2, self.size[1] // 2)
        pygame.draw.circle(self.surface, CYAN, vehicle_pos, 5)
        pygame.draw.circle(self.surface, BLUE, vehicle_pos, 4)

        # 차량 방향 표시
        import math
        direction_length = 15
        end_x = vehicle_pos[0] + direction_length * math.cos(vehicle.angle)
        end_y = vehicle_pos[1] + direction_length * math.sin(vehicle.angle)
        pygame.draw.line(self.surface, WHITE, vehicle_pos, (int(end_x), int(end_y)), 2)

        # 미니맵 테두리
        pygame.draw.rect(self.surface, WHITE, (0, 0, self.size[0], self.size[1]), 2)

        # 메인 화면에 미니맵 표시
        screen.blit(self.surface, self.position)

        # 미니맵 제목
        font = pygame.font.Font(None, 20)
        title = font.render("Mini Map", True, WHITE)
        screen.blit(title, (self.position[0], self.position[1] - 20))
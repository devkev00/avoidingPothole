"""
교통 관리 시스템 - NPC 차량 생성 및 관리
"""
import random
from npc_vehicle import NPCVehicle
from config import *


class TrafficManager:
    def __init__(self, road, player_vehicle):
        """
        교통 관리자 초기화

        Args:
            road: Road 객체
            player_vehicle: 플레이어 차량
        """
        self.road = road
        self.player = player_vehicle
        self.npc_vehicles = []

        # 생성 파라미터
        self.spawn_interval = 2.0  # 차량 생성 간격 (초)
        self.spawn_timer = 0
        self.max_vehicles = 15  # 최대 NPC 차량 수
        self.spawn_distance_ahead = 800  # 플레이어 앞쪽 생성 거리
        self.spawn_distance_behind = 400  # 플레이어 뒤쪽 생성 거리

        # 초기 차량 생성
        self._spawn_initial_vehicles()

    def _spawn_initial_vehicles(self):
        """
        초기 NPC 차량 생성
        """
        # 플레이어 주변에 5~10대 배치
        num_initial = random.randint(5, 10)

        for _ in range(num_initial):
            # 랜덤 위치 (플레이어 기준)
            x_offset = random.uniform(-300, 800)
            x = self.player.x + x_offset

            # 랜덤 차선
            lane_id = random.randint(1, self.road.num_lanes)
            y = self.road.get_lane_center_at_x(x, lane_id)

            if y is not None:
                # 랜덤 속도
                speed = random.uniform(70, 130)
                npc = NPCVehicle(x, y, lane_id, self.road, speed)

                # 플레이어와 너무 가까우면 스킵
                if abs(npc.x - self.player.x) > 100:
                    self.npc_vehicles.append(npc)

    def update(self, dt):
        """
        교통 시스템 업데이트

        Args:
            dt: 시간 간격 (초)
        """
        # NPC 차량들의 전방 차량 확인 및 업데이트
        for npc in self.npc_vehicles:
            # NPC의 전방 차량 찾기 (플레이어 또는 다른 NPC)
            front_vehicle = self._get_front_vehicle_for_npc(npc)
            npc.update(dt, front_vehicle)

        # 화면 밖 차량 제거
        self._cleanup_far_vehicles()

        # 새 차량 생성
        self.spawn_timer += dt
        if self.spawn_timer >= self.spawn_interval:
            self.spawn_timer = 0
            self._try_spawn_vehicle()

    def _get_front_vehicle_for_npc(self, npc):
        """
        NPC 차량의 전방에 있는 차량 찾기 (플레이어 또는 다른 NPC)

        Args:
            npc: NPCVehicle 객체

        Returns:
            Vehicle 또는 NPCVehicle 또는 None
        """
        front_vehicle = None
        min_distance = float('inf')

        # 플레이어 차량 확인
        if self.player.x > npc.x:
            player_lane = self.road.get_current_lane(self.player.x, self.player.y)
            if player_lane == npc.lane_id:
                distance = self.player.x - npc.x
                if distance < 500 and distance < min_distance:
                    min_distance = distance
                    front_vehicle = self.player

        # 다른 NPC 차량 확인
        for other_npc in self.npc_vehicles:
            if other_npc is npc:
                continue

            if other_npc.lane_id == npc.lane_id and other_npc.x > npc.x:
                distance = other_npc.x - npc.x
                if distance < 500 and distance < min_distance:
                    min_distance = distance
                    front_vehicle = other_npc

        return front_vehicle

    def _cleanup_far_vehicles(self):
        """
        플레이어로부터 멀리 떨어진 차량 제거
        """
        cleanup_distance = 1000

        self.npc_vehicles = [
            npc for npc in self.npc_vehicles
            if abs(npc.x - self.player.x) < cleanup_distance
        ]

    def _try_spawn_vehicle(self):
        """
        새 차량 생성 시도
        """
        if len(self.npc_vehicles) >= self.max_vehicles:
            return

        # 생성 위치 결정 (앞쪽 또는 뒤쪽)
        if random.random() < 0.7:
            # 70% 확률로 앞쪽에 생성
            spawn_x = self.player.x + self.spawn_distance_ahead
        else:
            # 30% 확률로 뒤쪽에 생성
            spawn_x = self.player.x - self.spawn_distance_behind

        # 랜덤 차선
        lane_id = random.randint(1, self.road.num_lanes)
        spawn_y = self.road.get_lane_center_at_x(spawn_x, lane_id)

        if spawn_y is None:
            return

        # 해당 위치에 이미 차량이 있는지 확인
        for npc in self.npc_vehicles:
            if abs(npc.x - spawn_x) < 150 and npc.lane_id == lane_id:
                return  # 너무 가까우면 생성 취소

        # 랜덤 속도
        speed = random.uniform(60, 140)

        # 차량 생성
        npc = NPCVehicle(spawn_x, spawn_y, lane_id, self.road, speed)
        self.npc_vehicles.append(npc)

    def get_vehicles_in_range(self, x, y, range_distance):
        """
        특정 위치 주변의 차량 반환

        Args:
            x, y: 중심 위치
            range_distance: 범위

        Returns:
            list: NPCVehicle 리스트
        """
        nearby = []

        for npc in self.npc_vehicles:
            distance = ((npc.x - x) ** 2 + (npc.y - y) ** 2) ** 0.5
            if distance <= range_distance:
                nearby.append(npc)

        return nearby

    def get_vehicles_in_lane_ahead(self, x, lane_id, ahead_distance):
        """
        특정 차선의 앞쪽 차량들 반환

        Args:
            x: 현재 x 좌표
            lane_id: 차선 번호
            ahead_distance: 앞쪽 거리

        Returns:
            list: NPCVehicle 리스트 (가까운 순)
        """
        vehicles = []

        for npc in self.npc_vehicles:
            if npc.lane_id == lane_id and npc.x > x and npc.x < x + ahead_distance:
                vehicles.append(npc)

        # 거리순 정렬
        vehicles.sort(key=lambda v: v.x)

        return vehicles

    def get_front_vehicle(self, player_x, player_y, player_lane):
        """
        플레이어 차량 앞쪽의 가장 가까운 차량 반환

        Args:
            player_x: 플레이어 x 좌표
            player_y: 플레이어 y 좌표
            player_lane: 플레이어 차선

        Returns:
            dict or None: {
                'vehicle': NPCVehicle,
                'distance': float,
                'relative_speed': float (양수: 내가 더 빠름, 음수: 앞차가 더 빠름)
            }
        """
        front_vehicles = self.get_vehicles_in_lane_ahead(player_x, player_lane, 500)

        if not front_vehicles:
            return None

        # 가장 가까운 차량
        nearest = front_vehicles[0]
        distance = nearest.x - player_x

        # 상대 속도 계산
        relative_speed = self.player.speed - nearest.speed

        return {
            'vehicle': nearest,
            'distance': distance,
            'relative_speed': relative_speed
        }

    def check_lane_change_safe(self, target_lane, player_x, player_y):
        """
        차선 변경이 안전한지 확인

        Args:
            target_lane: 목표 차선
            player_x, player_y: 플레이어 위치

        Returns:
            bool: 안전 여부
        """
        safety_distance_ahead = 100
        safety_distance_behind = 80

        for npc in self.npc_vehicles:
            if npc.lane_id != target_lane:
                continue

            # 앞뒤로 안전 거리 확인
            x_diff = npc.x - player_x

            if -safety_distance_behind < x_diff < safety_distance_ahead:
                return False  # 너무 가까움

        return True

    def draw(self, screen, camera):
        """
        모든 NPC 차량 그리기

        Args:
            screen: Pygame surface
            camera: Camera 객체
        """
        for npc in self.npc_vehicles:
            npc.draw(screen, camera)

    def get_all_vehicles(self):
        """
        모든 NPC 차량 반환

        Returns:
            list: NPCVehicle 리스트
        """
        return self.npc_vehicles

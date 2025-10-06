"""
포트홀 회피 경로 계획기
다양한 회피 전략 구현
"""
import math
from config import *


class PathPlanner:
    def __init__(self, vehicle, road, sensor, traffic_manager=None):
        """
        경로 계획기 초기화

        Args:
            vehicle: Vehicle 객체
            road: Road 객체
            sensor: PotholeSensor 객체
            traffic_manager: TrafficManager 객체 (선택)
        """
        self.vehicle = vehicle
        self.road = road
        self.sensor = sensor
        self.traffic_manager = traffic_manager

        # 현재 회피 상태
        self.avoidance_state = "NORMAL"  # NORMAL, IN_LANE_AVOID, LANE_CHANGE, EMERGENCY_STOP
        self.target_lane = 2  # 기본 2번 차선
        self.avoidance_path = []
        self.planned_speed = None  # 계획된 속도

        # 회피 파라미터
        self.safe_distance = 150  # 포트홀 감지 후 회피 시작 거리
        self.lane_change_distance = 200  # 차선 변경에 필요한 거리
        self.emergency_stop_distance = 100  # 긴급 정지 거리

        # 긴급 정지 상태
        self.is_emergency_stopped = False
        self.emergency_stop_pothole = None  # 긴급 정지 원인 포트홀

    def plan_path(self):
        """
        포트홀과 교통 상황을 고려한 경로 및 속도 계획

        Returns:
            list: [(x, y), ...] 경로 포인트
        """
        # 긴급 정지 상태 확인
        if self.is_emergency_stopped:
            return self._handle_emergency_stop_state()

        # 가장 가까운 포트홀 확인
        nearest = self.sensor.get_nearest_pothole()

        if nearest is None or nearest['distance'] > self.safe_distance:
            # 포트홀 없음 - 정상 주행
            self.planned_speed = None  # 순항 속도 사용
            return self._plan_normal_path()

        pothole = nearest['pothole']
        distance = nearest['distance']

        # 포트홀이 현재 차선 밖에 있는지 확인
        if self._is_pothole_outside_lane(pothole):
            # 포트홀이 다른 차선에만 있음 - 무시하고 정상 주행
            self.planned_speed = None
            return self._plan_normal_path()

        # 포트홀 위치 분석
        avoidance_type = self._analyze_pothole_situation(pothole, distance)

        # 회피 전략에 따른 경로 및 속도 계획
        if avoidance_type == "PASS_THROUGH":
            # 상황 1-1-1: 바퀴 사이 통과 가능
            self.planned_speed = None
            return self._plan_normal_path()

        elif avoidance_type == "IN_LANE_LEFT" or avoidance_type == "IN_LANE_RIGHT":
            # 상황 1: 차선 내부 회피
            self.planned_speed = None  # 속도 유지
            return self._plan_in_lane_avoidance(pothole, avoidance_type, distance)

        elif avoidance_type == "LANE_CHANGE":
            # 상황 2/3: 차선 변경 필요
            return self._plan_lane_change_with_traffic(pothole, distance)

        else:
            self.planned_speed = None
            return self._plan_normal_path()

    def _plan_lane_change_with_traffic(self, pothole, distance):
        """
        교통 상황을 고려한 차선 변경 계획

        Args:
            pothole: Pothole 객체
            distance: 포트홀까지 거리

        Returns:
            list: 경로 포인트
        """
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane is None:
            current_lane = self.target_lane

        # 차선 변경 가능 여부 확인
        target_lane = self._select_avoidance_lane(current_lane, pothole)

        if target_lane is None:
            # 차선 변경 불가능
            # 큰 포트홀이면 긴급 정지, 작은 포트홀이면 차선 내부 회피
            if pothole.radius > 30:
                # 긴급 정지
                self.is_emergency_stopped = True
                self.emergency_stop_pothole = pothole
                return self._plan_emergency_stop(pothole, distance)
            else:
                # 차선 내부 회피
                self.planned_speed = None
                if pothole.y < self.vehicle.y:
                    return self._plan_in_lane_avoidance(pothole, "IN_LANE_RIGHT", distance)
                else:
                    return self._plan_in_lane_avoidance(pothole, "IN_LANE_LEFT", distance)

        # 차선 변경 가능 - 정상 차선 변경 (속도 감속)
        # 차선 변경 시 안전을 위해 약간 감속 (최소 60px/s 보장)
        self.planned_speed = max(60, self.vehicle.speed * 0.85)
        return self._plan_lane_change_avoidance(pothole, distance)

    def _check_rear_traffic(self, lane_id):
        """
        후행 차량 위험도 확인

        Args:
            lane_id: 현재 차선

        Returns:
            dict: {'has_danger': bool, 'vehicle': NPCVehicle or None, 'time_to_collision': float}
        """
        if self.traffic_manager is None:
            return {'has_danger': False, 'vehicle': None, 'time_to_collision': float('inf')}

        # 뒤쪽 200m 범위의 차량 확인
        rear_vehicles = []
        for npc in self.traffic_manager.get_all_vehicles():
            if npc.lane_id == lane_id and npc.x < self.vehicle.x:
                distance = self.vehicle.x - npc.x
                if distance < 200:
                    rear_vehicles.append({
                        'vehicle': npc,
                        'distance': distance
                    })

        if not rear_vehicles:
            return {'has_danger': False, 'vehicle': None, 'time_to_collision': float('inf')}

        # 가장 가까운 후행 차량
        nearest_rear = min(rear_vehicles, key=lambda v: v['distance'])
        npc = nearest_rear['vehicle']
        distance = nearest_rear['distance']

        # 상대 속도 계산
        relative_speed = npc.speed - self.vehicle.speed

        # 접근 중인 차량인지 확인
        if relative_speed > 0:
            # 충돌 시간 계산
            time_to_collision = distance / relative_speed if relative_speed > 0 else float('inf')

            # 위험 판단 (3초 이내 충돌 예상 시)
            if time_to_collision < 3.0:
                return {
                    'has_danger': True,
                    'vehicle': npc,
                    'time_to_collision': time_to_collision
                }

        return {'has_danger': False, 'vehicle': None, 'time_to_collision': float('inf')}

    def _handle_emergency_stop_state(self):
        """
        긴급 정지 상태 처리 - 차선 변경 가능해지면 재출발

        Returns:
            list: 경로 포인트
        """
        if self.emergency_stop_pothole is None:
            # 포트홀 정보 없음 - 정지 해제
            self.is_emergency_stopped = False
            self.planned_speed = None
            return self._plan_normal_path()

        # 현재 차선
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane is None:
            current_lane = self.target_lane

        # 차선 변경 가능한지 확인
        target_lane = self._select_avoidance_lane(current_lane, self.emergency_stop_pothole)

        if target_lane is not None:
            # 차선 변경 가능 - 정지 해제하고 차선 변경
            pothole = self.emergency_stop_pothole
            self.is_emergency_stopped = False
            self.emergency_stop_pothole = None
            self.avoidance_state = "LANE_CHANGE"
            self.planned_speed = max(60, self.vehicle.speed * 0.85)

            pothole_distance = math.sqrt(
                (pothole.x - self.vehicle.x) ** 2 +
                (pothole.y - self.vehicle.y) ** 2
            )
            return self._plan_lane_change_avoidance(pothole, pothole_distance)
        else:
            # 여전히 차선 변경 불가 - 정지 유지
            self.avoidance_state = "EMERGENCY_STOP"
            self.planned_speed = 0
            path = self.road.get_path_for_lane(current_lane, self.vehicle.x, 1500)
            return path if path else self._plan_normal_path()

    def _plan_emergency_stop(self, pothole, pothole_distance):
        """
        긴급 정지 계획

        Args:
            pothole: Pothole 객체
            pothole_distance: 포트홀까지 거리

        Returns:
            list: 경로 포인트
        """
        self.avoidance_state = "EMERGENCY_STOP"

        # 정지 거리 계산 (포트홀 앞 안전 거리)
        stop_distance = max(pothole_distance - 50, 20)

        # 현재 속도와 정지 거리 고려하여 목표 속도 계산
        # v^2 = u^2 + 2as -> v = sqrt(2 * deceleration * distance)
        deceleration = 150  # config.py의 DECELERATION

        if stop_distance > 10:
            # 감속
            target_speed = max(0, math.sqrt(2 * deceleration * (stop_distance - 10)))
            self.planned_speed = min(target_speed, self.vehicle.speed * 0.5)
        else:
            # 완전 정지
            self.planned_speed = 0

        # 현재 차선 유지 경로
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane is None:
            current_lane = self.target_lane

        path = self.road.get_path_for_lane(current_lane, self.vehicle.x, 1500)

        return path if path else self._plan_normal_path()

    def _is_pothole_outside_lane(self, pothole):
        """
        포트홀이 현재 차선 밖에 있는지 확인

        Args:
            pothole: Pothole 객체

        Returns:
            bool: 포트홀이 현재 차선 밖에 있으면 True
        """
        # 현재 차량이 있는 차선
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane is None:
            return False

        # 현재 차선의 경계 계산
        lane_center_y = self.road.get_lane_center_at_x(self.vehicle.x, current_lane)
        if lane_center_y is None:
            return False

        lane_half_width = self.road.lane_width / 2

        # 차선 경계
        lane_top = lane_center_y - lane_half_width
        lane_bottom = lane_center_y + lane_half_width

        # 포트홀의 영향 범위 (반경 포함)
        pothole_top = pothole.y - pothole.radius
        pothole_bottom = pothole.y + pothole.radius

        # 포트홀이 현재 차선과 겹치는지 확인
        # 겹치지 않으면 True (무시 가능)
        if pothole_bottom < lane_top or pothole_top > lane_bottom:
            return True  # 포트홀이 차선 밖에 있음

        return False  # 포트홀이 차선과 겹침

    def _analyze_pothole_situation(self, pothole, distance):
        """
        포트홀 상황 분석

        Args:
            pothole: Pothole 객체
            distance: 포트홀까지 거리

        Returns:
            str: 회피 유형
        """
        # 바퀴 사이 통과 가능 여부 확인
        if pothole.is_between_wheels(self.vehicle):
            return "PASS_THROUGH"

        # 포트홀이 어느 쪽에 있는지 확인 (차량 좌표계 기준)
        dx = pothole.x - self.vehicle.x
        dy = pothole.y - self.vehicle.y

        # 차량 좌표계로 변환 (정확한 회전 변환)
        cos_a = math.cos(-self.vehicle.angle)
        sin_a = math.sin(-self.vehicle.angle)

        local_x = dx * cos_a - dy * sin_a  # 차량 전방/후방
        local_y = dx * sin_a + dy * cos_a  # 차량 좌/우

        # 포트홀 크기 확인
        if pothole.radius > 30:
            # 큰 포트홀 - 차선 변경 고려
            # 포트홀이 차선 중앙에 가까운지, 왼쪽/오른쪽에 치우쳤는지 확인
            current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
            if current_lane:
                lane_center_y = self.road.get_lane_center_at_x(pothole.x, current_lane)
                if lane_center_y:
                    # 포트홀이 차선 중앙에서 어느 쪽에 있는지
                    offset_from_lane_center = pothole.y - lane_center_y

                    # 차선의 1/4 이상 치우친 경우 차선 내부 회피 시도
                    if abs(offset_from_lane_center) > self.road.lane_width * 0.25:
                        # 포트홀이 차선 왼쪽에 치우침 -> 오른쪽으로 회피
                        if offset_from_lane_center < 0:
                            return "IN_LANE_RIGHT"
                        # 포트홀이 차선 오른쪽에 치우침 -> 왼쪽으로 회피
                        else:
                            return "IN_LANE_LEFT"

            # 중앙에 있거나 확인 불가 - 차선 변경
            return "LANE_CHANGE"

        # 작은/중간 포트홀 - 차선 내부 회피
        # 왼쪽/오른쪽 판단 (local_y > 0: 좌측, local_y < 0: 우측)
        if local_y > 0:
            return "IN_LANE_RIGHT"  # 포트홀이 좌측 -> 우측으로 회피
        else:
            return "IN_LANE_LEFT"   # 포트홀이 우측 -> 좌측으로 회피

    def _plan_normal_path(self):
        """
        정상 주행 경로 (차선 중앙 추종)

        Returns:
            list: 경로 포인트
        """
        self.avoidance_state = "NORMAL"
        current_x = self.vehicle.x
        path = self.road.get_path_for_lane(self.target_lane, current_x, 1500)
        return path

    def _plan_in_lane_avoidance(self, pothole, avoidance_type, distance):
        """
        차선 내부 회피 경로 생성 (상황 1)

        Args:
            pothole: Pothole 객체
            avoidance_type: "IN_LANE_LEFT" 또는 "IN_LANE_RIGHT"
            distance: 포트홀까지 거리

        Returns:
            list: 경로 포인트
        """
        self.avoidance_state = "IN_LANE_AVOID"

        path = []
        current_x = self.vehicle.x

        # 차선 중심선 가져오기
        lane_center_path = self.road.get_path_for_lane(self.target_lane, current_x, 1500)

        if not lane_center_path:
            return self._plan_normal_path()

        # 회피 오프셋 계산 (포트홀 크기 + 최소 안전 마진)
        from config import VEHICLE_WIDTH

        # 포트홀 반경 + 차량 절반 폭 + 최소 여유
        min_margin = 3  # 최소 안전 마진
        required_clearance = pothole.radius + (VEHICLE_WIDTH / 2) + min_margin

        # 차선 폭 제한 (차선을 벗어나지 않도록)
        max_offset = self.road.lane_width * 0.25
        offset = min(required_clearance, max_offset)

        if avoidance_type == "IN_LANE_RIGHT":
            offset = -offset  # 오른쪽으로 (y 감소)

        # 경로 생성
        for i, (px, py) in enumerate(lane_center_path):
            # 포트홀 전후로 부드럽게 회피
            x_dist_to_pothole = pothole.x - px

            if x_dist_to_pothole > 100:
                # 포트홀 전: 점진적으로 오프셋 증가
                t = max(0, min(1, (150 - x_dist_to_pothole) / 50))
                current_offset = offset * t
            elif x_dist_to_pothole > -100:
                # 포트홀 근처: 최대 오프셋 유지
                current_offset = offset
            else:
                # 포트홀 후: 점진적으로 중앙 복귀
                t = max(0, min(1, (x_dist_to_pothole + 150) / 50))
                current_offset = offset * t

            path.append((px, py + current_offset))

        return path

    def _plan_lane_change_avoidance(self, pothole, distance):
        """
        차선 변경 회피 경로 생성 (상황 2)

        Args:
            pothole: Pothole 객체
            distance: 포트홀까지 거리

        Returns:
            list: 경로 포인트
        """
        self.avoidance_state = "LANE_CHANGE"

        # 현재 차선 확인
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane is None:
            current_lane = self.target_lane

        # 포트홀이 있는 차선 확인
        pothole_lane = self.road.get_current_lane(pothole.x, pothole.y)

        if pothole_lane is None:
            return self._plan_normal_path()

        # 회피할 차선 결정 (위 또는 아래 차선)
        target_lane = self._select_avoidance_lane(current_lane, pothole)

        if target_lane is None:
            # 차선 변경 불가능 - 차선 내부 회피 시도
            if pothole.y < self.vehicle.y:
                return self._plan_in_lane_avoidance(pothole, "IN_LANE_RIGHT", distance)
            else:
                return self._plan_in_lane_avoidance(pothole, "IN_LANE_LEFT", distance)

        # 차선 변경 경로 생성
        path = []
        current_x = self.vehicle.x

        # 3단계 경로: 현재 차선 -> 목표 차선 -> 원래 차선
        for x in range(int(current_x), int(current_x + 1500), 50):
            x_dist_to_pothole = pothole.x - x

            if x_dist_to_pothole > 150:
                # 1단계: 목표 차선으로 이동
                lane = current_lane
                t = max(0, min(1, (200 - x_dist_to_pothole) / 50))
                y_current = self.road.get_lane_center_at_x(x, current_lane)
                y_target = self.road.get_lane_center_at_x(x, target_lane)
                y = y_current + (y_target - y_current) * t

            elif x_dist_to_pothole > -100:
                # 2단계: 목표 차선 유지
                y = self.road.get_lane_center_at_x(x, target_lane)

            else:
                # 3단계: 원래 차선으로 복귀
                t = max(0, min(1, (x_dist_to_pothole + 200) / 100))
                y_target = self.road.get_lane_center_at_x(x, target_lane)
                y_current = self.road.get_lane_center_at_x(x, current_lane)
                y = y_target + (y_current - y_target) * (1 - t)

            if y is not None:
                path.append((x, y))

        # 목표 차선 업데이트
        self.target_lane = current_lane

        return path

    def _select_avoidance_lane(self, current_lane, pothole):
        """
        회피할 차선 선택 (상황 3: 다른 차량 고려)

        Args:
            current_lane: 현재 차선
            pothole: 포트홀 객체

        Returns:
            int: 목표 차선 번호 또는 None
        """
        # 포트홀이 차량보다 위쪽에 있으면 아래 차선으로
        if pothole.y < self.vehicle.y:
            preferred_target = current_lane + 1  # 아래 차선
            alternative_target = current_lane - 1  # 위 차선
        else:
            preferred_target = current_lane - 1  # 위 차선
            alternative_target = current_lane + 1  # 아래 차선

        # 우선 차선 확인
        if self._is_lane_safe(preferred_target):
            return preferred_target

        # 대안 차선 확인
        if self._is_lane_safe(alternative_target):
            return alternative_target

        return None

    def _is_lane_safe(self, lane_id):
        """
        차선 변경이 안전한지 확인

        Args:
            lane_id: 목표 차선 번호

        Returns:
            bool: 안전 여부
        """
        # 차선 범위 확인
        if not (1 <= lane_id <= self.road.num_lanes):
            return False

        # 교통 관리자가 없으면 안전하다고 가정
        if self.traffic_manager is None:
            return True

        # 차선 변경 안전성 확인
        return self.traffic_manager.check_lane_change_safe(
            lane_id,
            self.vehicle.x,
            self.vehicle.y
        )

    def get_avoidance_state(self):
        """
        현재 회피 상태 반환

        Returns:
            str: 회피 상태
        """
        return self.avoidance_state

    def get_planned_speed(self):
        """
        계획된 속도 반환

        Returns:
            float or None: 계획된 속도 (None이면 순항 속도 사용)
        """
        return self.planned_speed

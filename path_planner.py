"""
포트홀 회피 경로 계획기
다양한 회피 전략 구현
"""
import math
from config import *


def smoothstep(t):
    """
    Smoothstep 함수 - 부드러운 S자 곡선 전환

    Args:
        t: 0~1 사이의 값

    Returns:
        float: 부드럽게 보간된 값
    """
    t = max(0, min(1, t))
    return t * t * (3 - 2 * t)


def cubic_bezier(t, p0, p1, p2, p3):
    """
    3차 베지어 곡선 계산

    Args:
        t: 0~1 사이의 매개변수
        p0, p1, p2, p3: 제어점 (x, y) 튜플

    Returns:
        (x, y): 베지어 곡선 상의 점
    """
    t = max(0, min(1, t))
    u = 1 - t

    # 베지어 공식: B(t) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
    x = (u**3 * p0[0] +
         3 * u**2 * t * p1[0] +
         3 * u * t**2 * p2[0] +
         t**3 * p3[0])

    y = (u**3 * p0[1] +
         3 * u**2 * t * p1[1] +
         3 * u * t**2 * p2[1] +
         t**3 * p3[1])

    return (x, y)


def generate_bezier_path(start, end, control1, control2, num_points=50):
    """
    베지어 곡선 경로 생성

    Args:
        start: 시작점 (x, y)
        end: 끝점 (x, y)
        control1: 첫 번째 제어점 (x, y)
        control2: 두 번째 제어점 (x, y)
        num_points: 생성할 점의 개수

    Returns:
        list: [(x, y), ...] 경로 포인트
    """
    path = []
    for i in range(num_points):
        t = i / (num_points - 1)
        point = cubic_bezier(t, start, control1, control2, end)
        path.append(point)
    return path


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

        # 예측 범위
        self.prediction_distance = 400  # 전방 예측 거리

    def _get_potholes_in_prediction_range(self):
        """
        전방 예측 범위 내의 모든 포트홀 가져오기

        Returns:
            list: [{'pothole': Pothole, 'distance': float}, ...]
        """
        detected = self.sensor.detected_potholes
        potholes_in_range = []

        for p_data in detected:
            if p_data['distance'] <= self.prediction_distance:
                # 현재 차선에 있는 포트홀만
                if not self._is_pothole_outside_lane(p_data['pothole']):
                    potholes_in_range.append(p_data)

        # 거리순 정렬
        potholes_in_range.sort(key=lambda x: x['distance'])
        return potholes_in_range

    def _calculate_path_cost(self, avoidance_type, pothole, distance):
        """
        경로 비용 계산 (낮을수록 좋음) - NPC 충돌 회피 최우선

        Args:
            avoidance_type: 회피 유형
            pothole: Pothole 객체
            distance: 포트홀까지 거리

        Returns:
            float: 경로 비용
        """
        cost = 0.0

        # 0. NPC 차량 충돌 위험 비용 (차선 내부 회피는 제외) ⚠️
        if avoidance_type not in ["IN_LANE_LEFT", "IN_LANE_RIGHT"]:
            # 차선 내부 회피는 NPC 비용 무시
            npc_collision_cost = self._calculate_npc_collision_risk(avoidance_type)
            if npc_collision_cost > 0.5:
                # NPC 충돌 위험이 높으면(0.5 이상) 매우 높은 페널티
                cost += npc_collision_cost * 1000.0  # 최우선 고려
            elif npc_collision_cost > 0.2:
                # 중간 위험도(0.2~0.5)면 적당한 페널티
                cost += npc_collision_cost * 200.0
            else:
                # 낮은 위험도(0.2 미만)면 작은 페널티
                cost += npc_collision_cost * 50.0

        # 1. 안전성 비용 (포트홀 크기 고려)
        safety_cost = pothole.radius * 2.0
        if avoidance_type == "PASS_THROUGH":
            safety_cost *= 1.2  # 통과 가능해도 회피가 더 안전
        elif avoidance_type == "EMERGENCY_STOP":
            safety_cost *= 5.0  # 긴급 정지는 위험

        # 2. 승차감 비용 (조향 강도)
        comfort_cost = 0.0
        if avoidance_type == "IN_LANE_LEFT" or avoidance_type == "IN_LANE_RIGHT":
            comfort_cost = 10.0  # 차선 내 회피
        elif avoidance_type == "LANE_CHANGE":
            comfort_cost = 30.0  # 차선 변경
        elif avoidance_type == "EMERGENCY_STOP":
            comfort_cost = 100.0  # 정지

        # 3. 효율성 비용 (거리)
        efficiency_cost = 0.0
        if distance < 100:
            efficiency_cost = 50.0  # 가까우면 대응 어려움
        elif distance < 200:
            efficiency_cost = 20.0

        # 4. 속도 유지 비용
        speed_cost = 0.0
        if avoidance_type == "EMERGENCY_STOP":
            speed_cost = 200.0  # 정지는 비효율적
        elif avoidance_type == "LANE_CHANGE":
            speed_cost = 15.0  # 차선 변경 시 감속

        total_cost = cost + safety_cost + comfort_cost + efficiency_cost + speed_cost
        return total_cost

    def _calculate_npc_collision_risk(self, avoidance_type):
        """
        NPC 차량과의 충돌 위험도 계산 (전방위)

        Args:
            avoidance_type: 회피 유형

        Returns:
            float: 충돌 위험도 (0~1, 높을수록 위험)
        """
        if self.traffic_manager is None:
            return 0.0

        risk = 0.0

        # 현재 차선
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if not current_lane:
            return 0.0

        # 차선 변경 시 목표 차선 확인
        if avoidance_type == "LANE_CHANGE":
            # 현재 차선의 포트홀 확인
            detected = self.sensor.get_nearest_pothole()
            if detected:
                pothole = detected['pothole']
                target_lane = self._select_avoidance_lane(current_lane, pothole)

                if target_lane:
                    # 목표 차선의 NPC 차량 확인
                    if not self._is_lane_safe_detailed(target_lane):
                        risk = 1.0  # 매우 위험
                        return risk

        # NPC 차량 확인 (차선 기반 + 전방 우선)
        all_npcs = self.traffic_manager.get_all_vehicles()

        # 차선 변경 시 목표 차선 파악
        target_lanes = [current_lane]
        if avoidance_type == "LANE_CHANGE":
            detected = self.sensor.get_nearest_pothole()
            if detected:
                pothole = detected['pothole']
                target_lane_id = self._select_avoidance_lane(current_lane, pothole)
                if target_lane_id:
                    target_lanes.append(target_lane_id)

        for npc in all_npcs:
            # NPC 차선 확인
            npc_lane = self.road.get_current_lane(npc.x, npc.y)

            # 차량과의 거리 계산
            dx = npc.x - self.vehicle.x
            dy = npc.y - self.vehicle.y
            distance = math.sqrt(dx**2 + dy**2)

            # 진행 방향 판단
            cos_a = math.cos(self.vehicle.angle)
            sin_a = math.sin(self.vehicle.angle)
            forward_distance = dx * cos_a + dy * sin_a

            # 같은 차선이거나 목표 차선에 있는 NPC만 높은 위험도 부여
            if npc_lane in target_lanes:
                # 전방 차량만 확인
                if forward_distance > 0 and distance < 250:
                    # 거리에 따른 위험도
                    if distance < 80:
                        risk += 1.0
                    elif distance < 120:
                        risk += 0.6
                    elif distance < 180:
                        risk += 0.3
                    elif distance < 250:
                        risk += 0.1

                    # 상대 속도 고려
                    relative_speed = self.vehicle.speed - npc.speed
                    if relative_speed > 20:
                        risk += 0.2
            else:
                # 다른 차선 NPC: 매우 가까운 경우만 낮은 위험도
                if distance < 100:
                    risk += 0.05

        return min(risk, 1.0)

    def _is_lane_safe_detailed(self, lane_id):
        """
        차선 변경 안전성 상세 확인 (더 엄격)

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

        # 목표 차선의 전방/후방 차량 확인
        all_npcs = self.traffic_manager.get_all_vehicles()

        for npc in all_npcs:
            if npc.lane_id != lane_id:
                continue

            # 거리 계산
            distance = abs(npc.x - self.vehicle.x)

            # 전방 차량
            if npc.x > self.vehicle.x:
                # 전방 이내에 차량이 있으면 안전하지 않음 (250 -> 350으로 증가)
                if distance < 350:
                    # 속도 차이 고려
                    if npc.speed < self.vehicle.speed:
                        # 내가 더 빠르면 위험
                        return False
                    elif distance < 200:  # 150 -> 200으로 증가
                        # 너무 가까우면 무조건 위험
                        return False

            # 후방 차량
            else:
                # 후방 이내에 차량이 있으면 안전하지 않음 (200 -> 300으로 증가)
                if distance < 300:
                    # 상대방이 더 빠르면 위험
                    if npc.speed > self.vehicle.speed:
                        return False
                    elif distance < 150:  # 100 -> 150으로 증가
                        # 너무 가까우면 무조건 위험
                        return False

        return True

    def _check_imminent_npc_collision(self):
        """
        NPC 차량과의 임박한 충돌 위험 확인 (전방위)

        Returns:
            bool: 충돌이 임박하면 True (긴급 상황)
        """
        if self.traffic_manager is None:
            return False

        # 모든 NPC 차량 확인 (전방위)
        all_npcs = self.traffic_manager.get_all_vehicles()

        from config import VEHICLE_LENGTH

        for npc in all_npcs:
            # 모든 방향의 차량 확인
            dx = npc.x - self.vehicle.x
            dy = npc.y - self.vehicle.y
            distance = math.sqrt(dx**2 + dy**2)

            # 긴급 거리 (차량 길이의 2.0배)
            emergency_distance = VEHICLE_LENGTH * 2.0

            # 매우 가까운 거리면 긴급 상황
            if distance < emergency_distance:
                return True

            # 진행 방향 확인
            cos_a = math.cos(self.vehicle.angle)
            sin_a = math.sin(self.vehicle.angle)
            forward_distance = dx * cos_a + dy * sin_a

            # 전방 차량의 경우 상대 속도 고려
            if forward_distance > 0:
                relative_speed = self.vehicle.speed - npc.speed
                if relative_speed > 20 and distance < VEHICLE_LENGTH * 3.5:
                    return True

            # 측면 또는 후방 차량의 경우 상대방이 접근 중인지 확인
            else:
                # NPC가 빠르게 접근 중이면 위험
                npc_relative_speed = npc.speed - self.vehicle.speed
                if npc_relative_speed > 20 and distance < VEHICLE_LENGTH * 2.5:
                    return True

        return False

    def plan_path(self):
        """
        포트홀과 교통 상황을 고려한 경로 및 속도 계획 (다중 포트홀 예측)
        NPC 차량 충돌 회피 최우선

        Returns:
            list: [(x, y), ...] 경로 포인트
        """
        # 긴급 정지 상태 확인
        if self.is_emergency_stopped:
            return self._handle_emergency_stop_state()

        # ⚠️ NPC 충돌 임박 확인 (매우 긴급한 경우만)
        # 차선 내부 회피는 NPC 상관없이 수행 가능하므로, 여기서는 차단하지 않음
        # ACC가 속도 조절로 NPC 충돌을 방지함

        # 전방 예측 범위 내의 모든 포트홀 가져오기
        potholes_in_range = self._get_potholes_in_prediction_range()

        if not potholes_in_range:
            # 포트홀 없음 - 정상 주행
            self.planned_speed = None
            return self._plan_normal_path()

        # 가장 가까운 포트홀 (우선 대응)
        primary_pothole_data = potholes_in_range[0]
        pothole = primary_pothole_data['pothole']
        distance = primary_pothole_data['distance']

        # 가능한 회피 전략들 평가
        possible_strategies = self._evaluate_avoidance_strategies(pothole, distance)

        # 비용이 가장 낮은 전략 선택
        best_strategy = min(possible_strategies, key=lambda x: x['cost'])
        avoidance_type = best_strategy['type']

        # 선택된 전략에 따른 경로 및 속도 계획
        if avoidance_type == "PASS_THROUGH":
            # 포트홀 통과 가능
            self.planned_speed = None
            return self._plan_normal_path()

        elif avoidance_type == "IN_LANE_LEFT" or avoidance_type == "IN_LANE_RIGHT":
            # 차선 내부 회피 (베지어 곡선)
            self.planned_speed = None
            return self._plan_in_lane_avoidance_bezier(pothole, avoidance_type, distance)

        elif avoidance_type == "LANE_CHANGE":
            # 차선 변경 (베지어 곡선)
            return self._plan_lane_change_with_traffic(pothole, distance)

        else:
            self.planned_speed = None
            return self._plan_normal_path()

    def _evaluate_avoidance_strategies(self, pothole, distance):
        """
        가능한 모든 회피 전략 평가

        Args:
            pothole: Pothole 객체
            distance: 포트홀까지 거리

        Returns:
            list: [{'type': str, 'cost': float}, ...]
        """
        strategies = []

        # 1. 통과 가능 여부 확인
        if pothole.is_between_wheels(self.vehicle):
            strategies.append({
                'type': 'PASS_THROUGH',
                'cost': self._calculate_path_cost('PASS_THROUGH', pothole, distance)
            })

        # 2. 차선 내부 회피 평가
        avoidance_type = self._analyze_pothole_situation(pothole, distance)
        if avoidance_type in ["IN_LANE_LEFT", "IN_LANE_RIGHT"]:
            strategies.append({
                'type': avoidance_type,
                'cost': self._calculate_path_cost(avoidance_type, pothole, distance)
            })

        # 3. 차선 변경 평가
        if avoidance_type == "LANE_CHANGE":
            current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
            if current_lane:
                target_lane = self._select_avoidance_lane(current_lane, pothole)
                if target_lane is not None:
                    strategies.append({
                        'type': 'LANE_CHANGE',
                        'cost': self._calculate_path_cost('LANE_CHANGE', pothole, distance)
                    })

        # 전략이 없으면 통과 시도
        if not strategies:
            strategies.append({
                'type': 'PASS_THROUGH',
                'cost': self._calculate_path_cost('PASS_THROUGH', pothole, distance)
            })

        return strategies

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
                # 차선 내부 회피 (베지어 곡선)
                self.planned_speed = None
                if pothole.y < self.vehicle.y:
                    return self._plan_in_lane_avoidance_bezier(pothole, "IN_LANE_RIGHT", distance)
                else:
                    return self._plan_in_lane_avoidance_bezier(pothole, "IN_LANE_LEFT", distance)

        # 차선 변경 가능 - 정상 차선 변경
        self.planned_speed = None
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
            self.planned_speed = None

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

    def _analyze_multiple_potholes(self, lane_potholes):
        """
        전방의 여러 포트홀을 종합적으로 분석하여 회피 전략 결정

        Args:
            lane_potholes: [{'pothole': Pothole, 'distance': float}, ...]

        Returns:
            str: 회피 유형
        """
        from config import TRACK_WIDTH
        wheel_gap = TRACK_WIDTH

        # 1. 모든 포트홀이 통과 가능한지 확인
        all_passable = all(
            p['pothole'].is_between_wheels(self.vehicle)
            for p in lane_potholes
        )

        if all_passable:
            return "PASS_THROUGH"

        # 2. 큰 포트홀(바퀴 간격보다 큰) 개수 확인
        large_potholes = [
            p for p in lane_potholes
            if p['pothole'].radius * 2 > wheel_gap
        ]

        # 3. 현재 차선 정보
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if not current_lane:
            # 차선 정보 없으면 가장 가까운 포트홀만 고려
            nearest = lane_potholes[0]
            return self._analyze_pothole_situation(nearest['pothole'], nearest['distance'])

        # 4. 포트홀들의 위치 분포 분석
        left_count = 0
        center_count = 0
        right_count = 0

        for p_data in lane_potholes:
            pothole = p_data['pothole']
            lane_center_y = self.road.get_lane_center_at_x(pothole.x, current_lane)

            if lane_center_y:
                offset = pothole.y - lane_center_y

                if offset < -self.road.lane_width * 0.15:
                    left_count += 1
                elif offset > self.road.lane_width * 0.15:
                    right_count += 1
                else:
                    center_count += 1

        # 5. 종합 판단
        # 큰 포트홀이 2개 이상이거나, 중앙에 포트홀이 많으면 차선 변경
        if len(large_potholes) >= 2 or center_count >= 2:
            return "LANE_CHANGE"

        # 큰 포트홀이 1개 있으면 개별 분석
        if len(large_potholes) == 1:
            nearest = lane_potholes[0]
            return self._analyze_pothole_situation(nearest['pothole'], nearest['distance'])

        # 작은 포트홀들만 있는 경우 - 분포에 따라 결정
        if left_count > 0 and right_count == 0:
            # 모두 왼쪽 - 오른쪽으로 회피
            return "IN_LANE_RIGHT"
        elif right_count > 0 and left_count == 0:
            # 모두 오른쪽 - 왼쪽으로 회피
            return "IN_LANE_LEFT"
        elif center_count > 0:
            # 중앙에 있음 - 차선 변경
            return "LANE_CHANGE"
        else:
            # 양쪽에 분산 - 가장 가까운 포트홀 기준
            nearest = lane_potholes[0]
            return self._analyze_pothole_situation(nearest['pothole'], nearest['distance'])

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

        # 바퀴 간격 가져오기
        from config import TRACK_WIDTH
        wheel_gap = TRACK_WIDTH

        # 포트홀 크기가 바퀴 사이 너비보다 큰지 확인 (더 보수적으로)
        # 직경 기준으로 판단하되 안전 마진 포함
        if pothole.radius * 2.0 > wheel_gap * 0.85:  # 바퀴 간격의 85% 이상이면 큰 포트홀로 간주
            # 큰 포트홀 - 위치별 회피 전략 결정
            current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
            if current_lane:
                lane_center_y = self.road.get_lane_center_at_x(pothole.x, current_lane)
                if lane_center_y:
                    # 포트홀이 차선 중앙에서 어느 쪽에 있는지
                    offset_from_lane_center = pothole.y - lane_center_y
                    lane_half_width = self.road.lane_width / 2

                    # 포트홀이 도로 중앙에 위치한 경우 (차선 중앙 20% 이내)
                    if abs(offset_from_lane_center) < self.road.lane_width * 0.2:
                        # 무조건 차선 외부 회피 (차선 변경)
                        return "LANE_CHANGE"

                    # 포트홀이 좌측 또는 우측에 치우친 경우 - 무조건 차선 내부 회피
                    if offset_from_lane_center < 0:
                        # 포트홀이 차선 왼쪽에 위치 -> 오른쪽으로 회피
                        return "IN_LANE_RIGHT"
                    else:
                        # 포트홀이 차선 오른쪽에 위치 -> 왼쪽으로 회피
                        return "IN_LANE_LEFT"

            # 차선 정보 없으면 차선 변경
            return "LANE_CHANGE"

        # 작은/중간 포트홀 - 차선 내부 회피
        # 차선 중앙 기준으로 판단 (차량 좌표계가 아닌 절대 좌표 기준)
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane:
            lane_center_y = self.road.get_lane_center_at_x(pothole.x, current_lane)
            if lane_center_y:
                # 포트홀이 차선 중앙에서 어느 쪽에 있는지
                offset_from_lane_center = pothole.y - lane_center_y

                if offset_from_lane_center < 0:
                    # 포트홀이 차선 위쪽(왼쪽)에 위치 -> 아래쪽(오른쪽)으로 회피
                    return "IN_LANE_RIGHT"
                else:
                    # 포트홀이 차선 아래쪽(오른쪽)에 위치 -> 위쪽(왼쪽)으로 회피
                    return "IN_LANE_LEFT"

        # 차선 정보 없으면 차량 좌표계 기준으로 폴백
        if local_y > 0:
            return "IN_LANE_LEFT"   # 포트홀이 우측 -> 좌측으로 회피
        else:
            return "IN_LANE_RIGHT"  # 포트홀이 좌측 -> 우측으로 회피

    def _plan_normal_path(self):
        """
        정상 주행 경로 (차선 중앙 추종)py

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
        path = []
        current_x = self.vehicle.x

        # 차선 중심선 가져오기
        lane_center_path = self.road.get_path_for_lane(self.target_lane, current_x, 1500)

        if not lane_center_path:
            return self._plan_normal_path()

        # 회피 방향 결정
        from config import TRACK_WIDTH
        min_margin = 25  # 안전 마진 증가 (15 -> 25)
        max_offset = self.road.lane_width * 0.4  # 최대 오프셋 증가 (0.35 -> 0.4)

        offset_direction = 1 if avoidance_type == "IN_LANE_RIGHT" else -1
        self.avoidance_state = "IN_LANE_AVOID"

        # 필요한 회피 거리 계산 (차량 중심에서 바퀴까지 거리 + 포트홀 반경 + 안전마진)
        base_clearance = pothole.radius + (TRACK_WIDTH / 2) + min_margin
        required_clearance = base_clearance * 1.3  # 1.3배 안전 계수 적용

        # 경로 생성 (max_offset으로 제한) - 부드러운 전환
        approach_distance = 300  # 회피 시작 거리 (200 -> 300)
        approach_transition = 150  # 회피 전환 구간 (100 -> 150)
        exit_distance = 150  # 복귀 시작 거리 (100 -> 150)
        exit_transition = 150  # 복귀 전환 구간 (100 -> 150)

        for i, (px, py) in enumerate(lane_center_path):
            x_dist = pothole.x - px

            # 포트홀과의 x 거리에 따라 오프셋 강도 조절 (smoothstep 사용)
            if x_dist > exit_distance:
                # 포트홀 전: 점진적으로 오프셋 증가
                linear_t = max(0, (approach_distance - x_dist) / approach_transition)
                t = smoothstep(linear_t)
            elif x_dist > -exit_distance:
                # 포트홀 근처: 최대 오프셋 유지
                t = 1.0
            else:
                # 포트홀 후: 점진적으로 감소
                linear_t = max(0, (x_dist + exit_distance + exit_transition) / exit_transition)
                t = smoothstep(linear_t)

            # max_offset으로 제한하여 차선을 벗어나지 않도록
            actual_clearance = min(required_clearance, max_offset)
            offset = actual_clearance * t * offset_direction
            path.append((px, py + offset))

        return path

    def _plan_in_lane_avoidance_bezier(self, pothole, avoidance_type, distance):
        """
        차선 내부 회피 경로 생성 - 베지어 곡선 사용

        Args:
            pothole: Pothole 객체
            avoidance_type: "IN_LANE_LEFT" 또는 "IN_LANE_RIGHT"
            distance: 포트홀까지 거리

        Returns:
            list: 경로 포인트
        """
        self.avoidance_state = "IN_LANE_AVOID"

        # 현재 차량 위치
        current_x = self.vehicle.x
        current_y = self.vehicle.y

        # 차선 중심선 가져오기
        current_lane = self.road.get_current_lane(current_x, current_y)
        if not current_lane:
            current_lane = self.target_lane

        lane_center_y = self.road.get_lane_center_at_x(current_x, current_lane)
        if not lane_center_y:
            return self._plan_normal_path()

        # 회피 파라미터
        from config import TRACK_WIDTH, VEHICLE_WIDTH
        min_margin = 25  # 안전 마진

        # 차선을 밟을 수 있도록 최대 오프셋 설정
        # 차선 중심에서 경계선까지 거리를 최대로 설정 (차선 밟기 허용)
        max_offset = self.road.lane_width * 0.45  # 차선폭의 45% (약 45px)
        offset_direction = 1 if avoidance_type == "IN_LANE_RIGHT" else -1

        # 필요한 회피 거리 계산
        base_clearance = pothole.radius + (TRACK_WIDTH / 2) + min_margin
        required_clearance = min(base_clearance * 1.3, max_offset)  # 1.3배 안전 계수 적용

        # 베지어 곡선 제어점 계산 (더 넓게 회피)
        # P0: 현재 위치
        p0 = (current_x, current_y)

        # P1: 회피 시작 지점 (포트홀 350px 전)
        approach_x = pothole.x - 350
        approach_y = self.road.get_lane_center_at_x(approach_x, current_lane)
        p1 = (approach_x, approach_y if approach_y else lane_center_y)

        # P2: 포트홀 통과 지점 (최대 오프셋 + 추가 여유)
        # 포트홀 약간 앞에서 최대 오프셋 도달
        peak_x = pothole.x - 50  # 포트홀 50px 전에 최대 오프셋
        pothole_y = self.road.get_lane_center_at_x(peak_x, current_lane)
        if pothole_y:
            p2 = (peak_x, pothole_y + required_clearance * offset_direction)
        else:
            p2 = (peak_x, current_y + required_clearance * offset_direction)

        # P2.5: 포트홀 지점 (최대 오프셋 유지)
        pothole_pass_y = self.road.get_lane_center_at_x(pothole.x + 50, current_lane)
        p2_5 = (pothole.x + 50, (pothole_pass_y if pothole_pass_y else current_y) + required_clearance * offset_direction)

        # P3: 복귀 완료 지점 (포트홀 350px 후)
        return_x = pothole.x + 350
        return_y = self.road.get_lane_center_at_x(return_x, current_lane)
        p3 = (return_x, return_y if return_y else lane_center_y)

        # 베지어 곡선 생성 (세그먼트별로)
        # 차량이 이미 지나간 세그먼트는 스킵하여 W자 형태 방지
        path = []

        # 현재 차량 위치를 기준으로 어느 세그먼트에 있는지 판단
        # 세그먼트 1: 현재 -> 회피 시작
        if current_x < approach_x:
            # 아직 회피 시작 전
            seg1 = generate_bezier_path(p0, p1,
                                        (p0[0] + (p1[0] - p0[0]) * 0.5, p0[1]),
                                        (p0[0] + (p1[0] - p0[0]) * 0.5, p1[1]),
                                        num_points=30)
            path.extend(seg1)

            # 세그먼트 2: 회피 시작 -> 최대 오프셋
            seg2 = generate_bezier_path(p1, p2,
                                        (p1[0] + (p2[0] - p1[0]) * 0.25, p1[1]),
                                        (p1[0] + (p2[0] - p1[0]) * 0.75, p2[1]),
                                        num_points=35)
            path.extend(seg2)

            # 세그먼트 2.5: 최대 오프셋 유지
            seg2_5 = generate_bezier_path(p2, p2_5,
                                          (p2[0] + (p2_5[0] - p2[0]) * 0.5, p2[1]),
                                          (p2[0] + (p2_5[0] - p2[0]) * 0.5, p2_5[1]),
                                          num_points=15)
            path.extend(seg2_5)

            # 세그먼트 3: 복귀
            seg3 = generate_bezier_path(p2_5, p3,
                                        (p2_5[0] + (p3[0] - p2_5[0]) * 0.25, p2_5[1]),
                                        (p2_5[0] + (p3[0] - p2_5[0]) * 0.75, p3[1]),
                                        num_points=45)
            path.extend(seg3)

        elif current_x < peak_x:
            # 회피 시작과 최대 오프셋 사이
            # 현재 위치에서 최대 오프셋까지
            current_y_adjusted = current_y
            seg2_partial = generate_bezier_path((current_x, current_y_adjusted), p2,
                                               (current_x + (p2[0] - current_x) * 0.25, current_y_adjusted),
                                               (current_x + (p2[0] - current_x) * 0.75, p2[1]),
                                               num_points=25)
            path.extend(seg2_partial)

            # 세그먼트 2.5: 최대 오프셋 유지
            seg2_5 = generate_bezier_path(p2, p2_5,
                                          (p2[0] + (p2_5[0] - p2[0]) * 0.5, p2[1]),
                                          (p2[0] + (p2_5[0] - p2[0]) * 0.5, p2_5[1]),
                                          num_points=15)
            path.extend(seg2_5)

            # 세그먼트 3: 복귀
            seg3 = generate_bezier_path(p2_5, p3,
                                        (p2_5[0] + (p3[0] - p2_5[0]) * 0.25, p2_5[1]),
                                        (p2_5[0] + (p3[0] - p2_5[0]) * 0.75, p3[1]),
                                        num_points=45)
            path.extend(seg3)

        elif current_x < p2_5[0]:
            # 최대 오프셋과 포트홀 통과 사이
            current_y_adjusted = current_y
            seg2_5_partial = generate_bezier_path((current_x, current_y_adjusted), p2_5,
                                                 (current_x + (p2_5[0] - current_x) * 0.5, current_y_adjusted),
                                                 (current_x + (p2_5[0] - current_x) * 0.5, p2_5[1]),
                                                 num_points=10)
            path.extend(seg2_5_partial)

            # 세그먼트 3: 복귀
            seg3 = generate_bezier_path(p2_5, p3,
                                        (p2_5[0] + (p3[0] - p2_5[0]) * 0.25, p2_5[1]),
                                        (p2_5[0] + (p3[0] - p2_5[0]) * 0.75, p3[1]),
                                        num_points=45)
            path.extend(seg3)

        elif current_x < return_x:
            # 포트홀 통과 후 복귀 중
            current_y_adjusted = current_y
            seg3_partial = generate_bezier_path((current_x, current_y_adjusted), p3,
                                               (current_x + (p3[0] - current_x) * 0.25, current_y_adjusted),
                                               (current_x + (p3[0] - current_x) * 0.75, p3[1]),
                                               num_points=35)
            path.extend(seg3_partial)

        else:
            # 회피 완료 - 정상 경로로 복귀
            path.append(p0)

        # 나머지 경로 (차선 중심)
        remaining_path = self.road.get_path_for_lane(current_lane, return_x, 1500)
        if remaining_path:
            path.extend(remaining_path)

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
            # 차선 변경 불가능 - 정상 주행
            return self._plan_normal_path()

        # 베지어 곡선을 사용한 차선 변경 경로 생성
        path = []
        current_x = self.vehicle.x
        current_y = self.vehicle.y

        # 주요 지점 계산
        lane_change_start_x = pothole.x - 280  # 차선 변경 시작
        pothole_x = pothole.x
        lane_change_end_x = pothole.x + 280  # 복귀 완료

        # Y 좌표 계산
        current_lane_y_start = self.road.get_lane_center_at_x(current_x, current_lane)
        target_lane_y_mid = self.road.get_lane_center_at_x(pothole_x, target_lane)
        current_lane_y_end = self.road.get_lane_center_at_x(lane_change_end_x, current_lane)

        if not all([current_lane_y_start, target_lane_y_mid, current_lane_y_end]):
            return self._plan_normal_path()

        # 세그먼트 1: 현재 -> 차선 변경 시작
        if lane_change_start_x > current_x:
            seg1_start = (current_x, current_y)
            seg1_end = (lane_change_start_x, current_lane_y_start)
            seg1 = generate_bezier_path(
                seg1_start, seg1_end,
                (seg1_start[0] + (seg1_end[0] - seg1_start[0]) * 0.5, seg1_start[1]),
                (seg1_start[0] + (seg1_end[0] - seg1_start[0]) * 0.5, seg1_end[1]),
                num_points=30
            )
            path.extend(seg1)

        # 세그먼트 2: 차선 변경 (현재 차선 -> 목표 차선)
        seg2_start = (lane_change_start_x, current_lane_y_start)
        seg2_end = (pothole_x, target_lane_y_mid)
        seg2 = generate_bezier_path(
            seg2_start, seg2_end,
            (seg2_start[0] + (seg2_end[0] - seg2_start[0]) * 0.4, seg2_start[1]),
            (seg2_start[0] + (seg2_end[0] - seg2_start[0]) * 0.6, seg2_end[1]),
            num_points=35
        )
        path.extend(seg2)

        # 세그먼트 3: 목표 차선 유지
        maintain_end_x = pothole_x + 130
        maintain_end_y = self.road.get_lane_center_at_x(maintain_end_x, target_lane)
        if maintain_end_y:
            seg3_start = (pothole_x, target_lane_y_mid)
            seg3_end = (maintain_end_x, maintain_end_y)
            seg3 = generate_bezier_path(
                seg3_start, seg3_end,
                (seg3_start[0] + (seg3_end[0] - seg3_start[0]) * 0.5, seg3_start[1]),
                (seg3_start[0] + (seg3_end[0] - seg3_start[0]) * 0.5, seg3_end[1]),
                num_points=20
            )
            path.extend(seg3)

        # 세그먼트 4: 복귀 (목표 차선 -> 원래 차선)
        seg4_start = (maintain_end_x, maintain_end_y if maintain_end_y else target_lane_y_mid)
        seg4_end = (lane_change_end_x, current_lane_y_end)
        seg4 = generate_bezier_path(
            seg4_start, seg4_end,
            (seg4_start[0] + (seg4_end[0] - seg4_start[0]) * 0.4, seg4_start[1]),
            (seg4_start[0] + (seg4_end[0] - seg4_start[0]) * 0.6, seg4_end[1]),
            num_points=35
        )
        path.extend(seg4)

        # 나머지 경로
        remaining_path = self.road.get_path_for_lane(current_lane, lane_change_end_x, 1500)
        if remaining_path:
            path.extend(remaining_path)

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

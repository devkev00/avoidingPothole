"""
포트홀 회피 시뮬레이션 - 메인 프로그램
"""
import pygame
import sys
import random
import math
from config import *
from vehicle import Vehicle
from road import Road
from pothole import Pothole
from camera import Camera
from sensors import PotholeSensor
from controller import PurePursuitController
from minimap import MiniMap
from pothole_generator import PotholeGenerator
from path_planner import PathPlanner
from traffic_manager import TrafficManager


class Simulation:
    def __init__(self):
        """
        시뮬레이션 초기화
        """
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("포트홀 회피 시뮬레이션")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

        # 도로 생성
        self.road = Road()

        # 차량 생성 (2번 차선 중앙에서 시작)
        start_x = 100
        start_y = self.road.get_lane_center_at_x(start_x, 2)  # 2번 차선
        self.vehicle = Vehicle(start_x, start_y, angle=0)

        # 가변 속도 설정
        self.cruise_speed = 100  # 순항 속도
        self.min_speed = 0  # 정지
        self.max_speed = 150  # 최대 속도
        self.vehicle.target_speed = self.cruise_speed

        # 카메라 생성
        self.camera = Camera(self.vehicle)

        # 센서 생성
        self.sensor = PotholeSensor(self.vehicle)

        # 제어기 생성
        self.controller = PurePursuitController(self.vehicle)

        # 포트홀 생성기
        self.pothole_generator = PotholeGenerator(self.road)

        # 교통 관리자 생성 (NPC 차량)
        self.traffic_manager = TrafficManager(self.road, self.vehicle)

        # 경로 계획기 생성 (교통 관리자 연결)
        self.path_planner = PathPlanner(self.vehicle, self.road, self.sensor, self.traffic_manager)

        # 미니맵 생성 (동적으로 확장되는 월드)
        world_bounds = (0, 250, 5000, 550)
        self.minimap = MiniMap(world_bounds)

        # 초기 경로 설정
        self.update_path()

        # 시뮬레이션 상태
        self.running = True
        self.paused = False

    def update_path(self):
        """
        포트홀을 회피하는 경로 업데이트
        """
        # 경로 계획기를 통해 회피 경로 생성
        path = self.path_planner.plan_path()

        if path:
            self.controller.set_path(path)

    def _calculate_acc_speed(self):
        """
        적응형 크루즈 컨트롤(ACC) 속도 계산

        Returns:
            float: 목표 속도
        """
        # 현재 차선 확인
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane is None:
            return self.cruise_speed

        # 전방 차량 확인
        front_vehicle_info = self.traffic_manager.get_front_vehicle(
            self.vehicle.x,
            self.vehicle.y,
            current_lane
        )

        if front_vehicle_info is None:
            # 전방에 차량 없음 - 순항 속도
            return self.cruise_speed

        distance = front_vehicle_info['distance']
        front_vehicle = front_vehicle_info['vehicle']
        relative_speed = front_vehicle_info['relative_speed']

        # 안전 거리 계산 (차량 하나 들어갈 정도 = 차량 길이 + 여유)
        # 시간 기반 안전 거리: 2초 법칙
        time_gap = 2.0  # 초
        safe_distance = self.vehicle.speed * time_gap + VEHICLE_LENGTH * 1.5

        # 최소 안전 거리
        min_safe_distance = VEHICLE_LENGTH * 1.2

        # 안전 거리와 실제 거리 비교
        if distance < min_safe_distance:
            # 너무 가까움 - 급감속
            target_speed = front_vehicle.speed * 0.7
        elif distance < safe_distance:
            # 안전 거리 이하 - 감속
            # 거리에 비례한 속도 조정
            speed_factor = (distance - min_safe_distance) / (safe_distance - min_safe_distance)
            target_speed = front_vehicle.speed * (0.7 + 0.3 * speed_factor)
        else:
            # 안전 거리 확보 - 전방 차량 속도 매칭 또는 순항
            if relative_speed > 10:
                # 내가 훨씬 빠름 - 서서히 감속
                target_speed = min(self.cruise_speed, front_vehicle.speed + 20)
            else:
                # 속도 비슷 - 순항 속도
                target_speed = self.cruise_speed

        # 속도 제한 적용
        target_speed = max(self.min_speed, min(self.max_speed, target_speed))

        return target_speed

    def handle_events(self):
        """
        이벤트 처리
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                # 스페이스: 일시정지
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused

                # ESC: 종료
                elif event.key == pygame.K_ESCAPE:
                    self.running = False

                # 화살표 위/아래: 순항 속도 조절
                elif event.key == pygame.K_UP:
                    self.cruise_speed = min(self.max_speed, self.cruise_speed + 10)
                elif event.key == pygame.K_DOWN:
                    self.cruise_speed = max(30, self.cruise_speed - 10)

                # 1, 2, 3: 차선 변경
                elif event.key == pygame.K_1:
                    self.path_planner.target_lane = 1
                elif event.key == pygame.K_2:
                    self.path_planner.target_lane = 2
                elif event.key == pygame.K_3:
                    self.path_planner.target_lane = 3

                # D: 디버그 모드 토글
                elif event.key == pygame.K_d:
                    global DEBUG_MODE
                    DEBUG_MODE = not DEBUG_MODE

    def update(self, dt):
        """
        시뮬레이션 업데이트

        Args:
            dt: 시간 간격 (초)
        """
        if self.paused:
            return

        # 도로 확장 (무한 생성)
        self.road.extend_road(self.vehicle.x)

        # 포트홀 생성 및 정리
        self.pothole_generator.generate_potholes(self.vehicle.x)
        self.pothole_generator.cleanup_old_potholes(self.vehicle.x)

        # NPC 차량 업데이트
        self.traffic_manager.update(dt)

        # 서브스텝을 사용한 더 빠른 경로 업데이트
        # 한 프레임을 2개의 서브스텝으로 나눠서 경로를 더 자주 업데이트
        num_substeps = 4
        substep_dt = dt / num_substeps

        for _ in range(num_substeps):
            # 포트홀 감지
            potholes = self.pothole_generator.get_potholes()
            self.sensor.detect_potholes(potholes)

            # 경로 업데이트 (서브스텝마다)
            self.update_path()

            # 속도 계획 적용
            planned_speed = self.path_planner.get_planned_speed()
            if planned_speed is not None:
                # 긴급 정지 또는 감속 (최우선)
                self.vehicle.target_speed = planned_speed
            else:
                # 적응형 크루즈 컨트롤 (ACC)
                acc_speed = self._calculate_acc_speed()
                self.vehicle.target_speed = acc_speed

            # 제어기 업데이트
            self.controller.update()

            # 차량 업데이트
            self.vehicle.update(substep_dt)

        # 카메라 업데이트
        self.camera.update()

        # 포트홀 충돌 검사
        wheels = self.vehicle.get_wheel_positions()
        for pothole in potholes:
            collision_info = pothole.check_collision(wheels)
            if collision_info['collision']:
                print(f"포트홀 충돌! 바퀴 {collision_info['wheel_index']}")

        # NPC 차량 충돌 검사
        for npc in self.traffic_manager.get_all_vehicles():
            if npc.check_collision_with_vehicle(self.vehicle):
                print(f"차량 충돌! NPC 차량과 충돌")

    def draw(self):
        """
        화면 그리기
        """
        self.screen.fill(BLACK)

        # 도로 그리기
        self.road.draw(self.screen, self.camera)

        # 포트홀 그리기
        potholes = self.pothole_generator.get_potholes()
        detected_potholes = [d['pothole'] for d in self.sensor.detected_potholes]

        for pothole in potholes:
            is_detected = pothole in detected_potholes
            pothole.draw(self.screen, self.camera, is_detected, self.vehicle)

        # NPC 차량 그리기
        self.traffic_manager.draw(self.screen, self.camera)

        # 제어기 (경로) 그리기
        self.controller.draw(self.screen, self.camera)

        # 센서 그리기
        self.sensor.draw(self.screen, self.camera)

        # 차량 그리기 (플레이어)
        self.vehicle.draw(self.screen, self.camera)

        # ACC 시각화 (디버그 모드)
        if DEBUG_MODE:
            self._draw_acc_indicator()

        # 미니맵 그리기
        npc_vehicles = self.traffic_manager.get_all_vehicles()
        self.minimap.draw(self.screen, self.vehicle, potholes, self.road, npc_vehicles)

        # UI 정보 표시
        self.draw_ui()

        pygame.display.flip()

    def _draw_acc_indicator(self):
        """
        ACC 전방 차량 표시기
        """
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane is None:
            return

        front_info = self.traffic_manager.get_front_vehicle(
            self.vehicle.x, self.vehicle.y, current_lane
        )

        if front_info is None:
            return

        # 플레이어와 전방 차량 연결선
        player_pos = self.camera.world_to_screen((self.vehicle.x, self.vehicle.y))
        front_vehicle = front_info['vehicle']
        front_pos = self.camera.world_to_screen((front_vehicle.x, front_vehicle.y))

        # 거리에 따른 색상
        distance = front_info['distance']
        safe_distance = self.vehicle.speed * 2.0 + VEHICLE_LENGTH * 1.5

        if distance < VEHICLE_LENGTH * 1.2:
            line_color = RED  # 위험
        elif distance < safe_distance:
            line_color = YELLOW  # 주의
        else:
            line_color = GREEN  # 안전

        # 연결선 그리기
        pygame.draw.line(self.screen, line_color, player_pos, front_pos, 2)

        # 안전 거리 표시
        safe_dist_x = self.vehicle.x + safe_distance
        safe_dist_y = self.vehicle.y
        safe_dist_pos = self.camera.world_to_screen((safe_dist_x, safe_dist_y))

        pygame.draw.circle(self.screen, CYAN, safe_dist_pos, 5)

    def draw_ui(self):
        """
        UI 정보 표시
        """
        # 속도 표시
        speed_text = self.font.render(
            f"Speed: {int(self.vehicle.speed)} px/s",
            True, WHITE
        )
        self.screen.blit(speed_text, (SCREEN_WIDTH - 200, 20))

        # 조향각 표시
        steering_deg = math.degrees(self.vehicle.steering_angle)
        steering_text = self.font.render(
            f"Steering: {steering_deg:.1f}°",
            True, WHITE
        )
        self.screen.blit(steering_text, (SCREEN_WIDTH - 200, 50))

        # 현재 차선 표시
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        lane_text = self.font.render(
            f"Lane: {current_lane if current_lane else 'N/A'}",
            True, WHITE
        )
        self.screen.blit(lane_text, (SCREEN_WIDTH - 200, 80))

        # 주행 거리 표시
        distance_text = self.font.render(
            f"Distance: {int(self.vehicle.x)}m",
            True, WHITE
        )
        self.screen.blit(distance_text, (SCREEN_WIDTH - 200, 140))

        # 감지된 포트홀 수
        detected = len(self.sensor.detected_potholes)
        pothole_text = self.font.render(
            f"Detected Potholes: {detected}",
            True, ORANGE
        )
        self.screen.blit(pothole_text, (SCREEN_WIDTH - 200, 110))

        # 회피 상태 표시
        avoidance_state = self.path_planner.get_avoidance_state()
        state_colors = {
            "NORMAL": GREEN,
            "IN_LANE_AVOID": YELLOW,
            "LANE_CHANGE": RED,
            "EMERGENCY_STOP": (255, 0, 255)  # 자홍색
        }
        state_text = self.font.render(
            f"State: {avoidance_state}",
            True, state_colors.get(avoidance_state, WHITE)
        )
        self.screen.blit(state_text, (SCREEN_WIDTH - 200, 170))

        # 순항 속도 표시
        cruise_text = self.font.render(
            f"Cruise: {int(self.cruise_speed)} px/s",
            True, CYAN
        )
        self.screen.blit(cruise_text, (SCREEN_WIDTH - 200, 230))

        # 전방 차량 정보 표시
        current_lane = self.road.get_current_lane(self.vehicle.x, self.vehicle.y)
        if current_lane:
            front_info = self.traffic_manager.get_front_vehicle(
                self.vehicle.x, self.vehicle.y, current_lane
            )
            if front_info:
                distance = front_info['distance']
                front_speed = int(front_info['vehicle'].speed)

                # ACC 활성화 표시
                acc_text = self.font.render(
                    f"ACC: {int(distance)}px @ {front_speed}",
                    True, ORANGE
                )
                self.screen.blit(acc_text, (SCREEN_WIDTH - 200, 260))
            else:
                # 전방 차량 없음
                acc_text = self.font.render(
                    "ACC: Clear",
                    True, GREEN
                )
                self.screen.blit(acc_text, (SCREEN_WIDTH - 200, 260))

        # NPC 차량 수 표시
        npc_count = len(self.traffic_manager.get_all_vehicles())
        npc_text = self.font.render(
            f"NPC Vehicles: {npc_count}",
            True, WHITE
        )
        self.screen.blit(npc_text, (SCREEN_WIDTH - 200, 200))

        # 긴급 정지 경고
        if avoidance_state == "EMERGENCY_STOP":
            warning_font = pygame.font.Font(None, 36)
            warning_text = warning_font.render("! EMERGENCY STOP !", True, (255, 0, 255))
            warning_rect = warning_text.get_rect(center=(SCREEN_WIDTH // 2, 60))
            # 깜빡이는 효과
            import time
            if int(time.time() * 2) % 2 == 0:
                self.screen.blit(warning_text, warning_rect)

        # 조작 안내
        if self.paused:
            pause_text = self.font.render("PAUSED", True, RED)
            self.screen.blit(pause_text, (SCREEN_WIDTH // 2 - 50, 20))

        help_text = [
            "SPACE: Pause",
            "UP/DOWN: Speed",
            "1/2/3: Lane",
            "D: Debug",
            "ESC: Quit"
        ]

        small_font = pygame.font.Font(None, 18)
        for i, text in enumerate(help_text):
            rendered = small_font.render(text, True, LIGHT_GRAY)
            self.screen.blit(rendered, (10, SCREEN_HEIGHT - 100 + i * 20))

    def run(self):
        """
        메인 루프 실행
        """
        while self.running:
            dt = self.clock.tick(FPS) / 1000.0  # 초 단위로 변환

            self.handle_events()
            self.update(dt)
            self.draw()

        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    sim = Simulation()
    sim.run()
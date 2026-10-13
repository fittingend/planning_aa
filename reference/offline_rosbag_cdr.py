"""ROS 실행 환경 없이 K-City 기록의 경로와 결정 문자열을 읽습니다."""

import sqlite3
import struct
from pathlib import Path


class Reader:
    def __init__(self, data):
        if data[:4] != b"\x00\x01\x00\x00":
            raise ValueError("지원하지 않는 CDR 인코딩입니다")
        self.data = data
        self.offset = 4

    def value(self, code, alignment):
        self.offset += (-(self.offset - 4)) % alignment
        size = struct.calcsize("<" + code)
        if self.offset + size > len(self.data):
            raise ValueError("CDR 메시지가 중간에서 끝났습니다")
        result = struct.unpack_from("<" + code, self.data, self.offset)
        self.offset += size
        return result[0] if len(result) == 1 else result

    def text(self):
        length = self.value("I", 4)
        if length < 1 or self.offset + length > len(self.data):
            raise ValueError("CDR 문자열 길이가 잘못되었습니다")
        result = self.data[self.offset:self.offset + length]
        self.offset += length
        if result[-1] != 0:
            raise ValueError("CDR 문자열 끝이 잘못되었습니다")
        return result[:-1].decode("utf-8")

    def header(self):
        seconds = self.value("i", 4)
        nanoseconds = self.value("I", 4)
        frame = self.text()
        return seconds * 1_000_000_000 + nanoseconds, frame

    def point(self):
        return tuple(self.value("d", 8) for _ in range(3))

    def path_point(self):
        pose = tuple(self.value("d", 8) for _ in range(7))
        motion = tuple(self.value("f", 4) for _ in range(3))
        final = bool(self.value("B", 1))
        return pose + motion + (final,)


def decode_path(data, with_lane_ids=False):
    reader = Reader(data)
    stamp, frame = reader.header()
    count = reader.value("I", 4)
    if count > 1_000_000:
        raise ValueError("경로 점 수가 비정상적입니다")
    points = []
    lanes = []
    for _ in range(count):
        points.append(reader.path_point())
        if with_lane_ids:
            lane_count = reader.value("I", 4)
            if lane_count > 100_000:
                raise ValueError("차로 ID 수가 비정상적입니다")
            lanes.append(tuple(reader.value("q", 8)
                               for _ in range(lane_count)))
    bounds = []
    for _ in range(2):
        bound_count = reader.value("I", 4)
        if bound_count > 1_000_000:
            raise ValueError("경계 점 수가 비정상적입니다")
        bounds.append([reader.point() for _ in range(bound_count)])
    if reader.offset != len(data):
        raise ValueError("경로 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": stamp, "frame": frame,
            "points": points, "lane_ids": lanes,
            "left_bound": bounds[0], "right_bound": bounds[1]}


def decode_pointcloud2_frame(data):
    """점군 메시지의 필드 배치와 원시 점 바이트를 읽습니다."""
    reader = Reader(data)
    stamp, frame = reader.header()
    height = reader.value("I", 4)
    width = reader.value("I", 4)
    field_count = reader.value("I", 4)
    if field_count > 1000:
        raise ValueError("점군 필드 수가 비정상적입니다")
    fields = []
    for _ in range(field_count):
        fields.append({"name": reader.text(),
                       "offset": reader.value("I", 4),
                       "datatype": reader.value("B", 1),
                       "count": reader.value("I", 4)})
    big_endian = bool(reader.value("B", 1))
    point_step = reader.value("I", 4)
    row_step = reader.value("I", 4)
    byte_count = reader.value("I", 4)
    if reader.offset + byte_count > len(data):
        raise ValueError("점군 바이트 배열이 중간에서 끝났습니다")
    points = data[reader.offset:reader.offset + byte_count]
    reader.offset += byte_count
    is_dense = bool(reader.value("B", 1))
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("점군 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": stamp, "frame": frame,
            "height": height, "width": width,
            "fields": fields, "big_endian": big_endian,
            "point_step": point_step, "row_step": row_step,
            "data": points, "is_dense": is_dense}


def decode_string(data):
    reader = Reader(data)
    value = reader.text()
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("문자열 메시지 끝에 해석되지 않은 내용이 있습니다")
    return value


def decode_odometry(data):
    reader = Reader(data)
    stamp, _ = reader.header()
    reader.text()
    pose = tuple(reader.value("d", 8) for _ in range(7))
    for _ in range(36):
        reader.value("d", 8)
    twist = tuple(reader.value("d", 8) for _ in range(6))
    return {"stamp_ns": stamp, "x": pose[0], "y": pose[1],
            "z": pose[2], "orientation_x": pose[3],
            "orientation_y": pose[4], "orientation_z": pose[5],
            "orientation_w": pose[6], "velocity_mps": twist[0]}


def decode_occupancy_grid(data):
    """점유 격자의 해상도·원점·셀 값을 원본 바이트 순서로 읽습니다."""
    reader = Reader(data)
    stamp, frame = reader.header()
    reader.value("i", 4)
    reader.value("I", 4)
    resolution = reader.value("f", 4)
    width = reader.value("I", 4)
    height = reader.value("I", 4)
    origin = tuple(reader.value("d", 8) for _ in range(7))
    count = reader.value("I", 4)
    if count != width * height or reader.offset + count > len(data):
        raise ValueError("점유 격자의 셀 수가 유효하지 않습니다")
    values = memoryview(data)[reader.offset:reader.offset + count].cast("b")
    reader.offset += count
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("점유 격자 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": stamp, "frame": frame, "resolution_m": resolution,
            "width": width, "height": height, "origin": origin,
            "values": values}


def decode_lanelet_route(data):
    """목표 자세와 차로 구간의 기본 차로·대체 차로 ID를 읽습니다."""
    reader = Reader(data)
    stamp, frame = reader.header()
    start = tuple(reader.value("d", 8) for _ in range(7))
    goal = tuple(reader.value("d", 8) for _ in range(7))
    segment_count = reader.value("I", 4)
    if segment_count > 100_000:
        raise ValueError("경로 차로 구간 수가 비정상적입니다")
    segments = []
    for _ in range(segment_count):
        preferred = (reader.value("q", 8), reader.text())
        primitive_count = reader.value("I", 4)
        if primitive_count > 100_000:
            raise ValueError("경로 차로 후보 수가 비정상적입니다")
        primitives = [(reader.value("q", 8), reader.text())
                      for _ in range(primitive_count)]
        segments.append({"preferred": preferred, "primitives": primitives})
    uuid = bytes(reader.value("B", 1) for _ in range(16)).hex()
    allow_modification = bool(reader.value("B", 1))
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("경로 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": stamp, "frame": frame, "start": start,
            "goal": goal, "segments": segments, "uuid": uuid,
            "allow_modification": allow_modification}


def decode_marker_array(data):
    """기록된 시각화 표식의 이름·자세·점 배열을 읽습니다."""
    reader = Reader(data)
    count = reader.value("I", 4)
    if count > 100_000:
        raise ValueError("시각화 표식 수가 비정상적입니다")
    markers = []
    for _ in range(count):
        stamp, frame = reader.header()
        namespace = reader.text()
        marker_id = reader.value("i", 4)
        marker_type = reader.value("i", 4)
        action = reader.value("i", 4)
        pose = tuple(reader.value("d", 8) for _ in range(7))
        scale = tuple(reader.value("d", 8) for _ in range(3))
        color = tuple(reader.value("f", 4) for _ in range(4))
        lifetime = (reader.value("i", 4), reader.value("I", 4))
        frame_locked = bool(reader.value("B", 1))
        point_count = reader.value("I", 4)
        if point_count > 1_000_000:
            raise ValueError("시각화 점 수가 비정상적입니다")
        points = [reader.point() for _ in range(point_count)]
        color_count = reader.value("I", 4)
        if color_count > 1_000_000:
            raise ValueError("시각화 색 수가 비정상적입니다")
        colors = [tuple(reader.value("f", 4) for _ in range(4))
                  for _ in range(color_count)]
        texture_resource = reader.text()
        texture_stamp, texture_frame = reader.header()
        texture_format = reader.text()
        texture_size = reader.value("I", 4)
        if texture_size > 100_000_000:
            raise ValueError("시각화 질감 크기가 비정상적입니다")
        reader.offset += texture_size
        uv_count = reader.value("I", 4)
        if uv_count > 1_000_000:
            raise ValueError("시각화 질감 좌표 수가 비정상적입니다")
        uv_coordinates = [(reader.value("f", 4), reader.value("f", 4))
                          for _ in range(uv_count)]
        content = reader.text()
        mesh = reader.text()
        mesh_filename = reader.text()
        mesh_size = reader.value("I", 4)
        if mesh_size > 100_000_000:
            raise ValueError("시각화 그물망 크기가 비정상적입니다")
        reader.offset += mesh_size
        embedded_materials = bool(reader.value("B", 1))
        markers.append({"stamp_ns": stamp, "frame": frame,
                        "namespace": namespace, "id": marker_id,
                        "type": marker_type, "action": action,
                        "pose": pose, "scale": scale, "color": color,
                        "lifetime": lifetime,
                        "frame_locked": frame_locked,
                        "points": points, "colors": colors,
                        "text": content, "mesh": mesh,
                        "mesh_filename": mesh_filename,
                        "texture_resource": texture_resource,
                        "texture_stamp_ns": texture_stamp,
                        "texture_frame": texture_frame,
                        "texture_format": texture_format,
                        "uv_coordinates": uv_coordinates,
                        "embedded_materials": embedded_materials})
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("시각화 표식 메시지 끝에 해석되지 않은 내용이 있습니다")
    return markers


def decode_predicted_objects(data):
    """원본 순항 계획 입력의 예측 객체와 경로를 CDR에서 읽습니다."""
    reader = Reader(data)
    stamp, frame = reader.header()
    count = reader.value("I", 4)
    if count > 100_000:
        raise ValueError("예측 객체 수가 비정상적입니다")
    objects = []
    for _ in range(count):
        uuid = bytes(reader.value("B", 1) for _ in range(16)).hex()
        existence = reader.value("f", 4)
        class_count = reader.value("I", 4)
        if class_count > 1000:
            raise ValueError("예측 객체 분류 수가 비정상적입니다")
        classes = [(reader.value("B", 1), reader.value("f", 4))
                   for _ in range(class_count)]
        pose = tuple(reader.value("d", 8) for _ in range(7))
        for _ in range(36):
            reader.value("d", 8)
        twist = tuple(reader.value("d", 8) for _ in range(6))
        twist_covariance = tuple(reader.value("d", 8) for _ in range(36))
        acceleration = tuple(reader.value("d", 8) for _ in range(6))
        for _ in range(36):
            reader.value("d", 8)
        path_count = reader.value("I", 4)
        if path_count > 1000:
            raise ValueError("객체 예측 경로 수가 비정상적입니다")
        paths = []
        for _ in range(path_count):
            pose_count = reader.value("I", 4)
            if pose_count > 100_000:
                raise ValueError("객체 예측 자세 수가 비정상적입니다")
            path = [tuple(reader.value("d", 8) for _ in range(7))
                    for _ in range(pose_count)]
            seconds = reader.value("i", 4)
            nanoseconds = reader.value("I", 4)
            confidence = reader.value("f", 4)
            paths.append({"poses": path,
                          "time_step_s": seconds + nanoseconds * 1e-9,
                          "confidence": confidence})
        shape_type = reader.value("B", 1)
        footprint_count = reader.value("I", 4)
        if footprint_count > 100_000:
            raise ValueError("객체 외형 점 수가 비정상적입니다")
        footprint = [tuple(reader.value("f", 4) for _ in range(3))
                     for _ in range(footprint_count)]
        dimensions = tuple(reader.value("d", 8) for _ in range(3))
        objects.append({"uuid": uuid, "existence": existence,
                        "classification": classes, "pose": pose,
                        "twist": twist, "twist_variance_xy":
                        (twist_covariance[0], twist_covariance[7]),
                        "acceleration": acceleration,
                        "predicted_paths": paths, "shape_type": shape_type,
                        "footprint": footprint, "dimensions": dimensions})
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("예측 객체 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": stamp, "frame": frame, "objects": objects}


def decode_traffic_light_groups(data):
    """기록된 신호 그룹 ID와 전구 색·모양·상태를 읽습니다."""
    reader = Reader(data)
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    count = reader.value("I", 4)
    if count > 100_000:
        raise ValueError("신호 그룹 수가 비정상적입니다")
    groups = []
    for _ in range(count):
        group_id = reader.value("q", 8)
        element_count = reader.value("I", 4)
        if element_count > 1000:
            raise ValueError("신호 전구 수가 비정상적입니다")
        elements = [(
            reader.value("B", 1), reader.value("B", 1),
            reader.value("B", 1), reader.value("f", 4))
            for _ in range(element_count)]
        groups.append({"id": group_id, "elements": elements})
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("신호 그룹 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": seconds * 1_000_000_000 + nanoseconds,
            "groups": groups}


def decode_cooperate_status_array(data):
    """기록된 RTC 협력 상태의 승인·안전·실행 상태를 읽습니다."""
    reader = Reader(data)
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    count = reader.value("I", 4)
    if count > 100_000:
        raise ValueError("RTC 상태 개수가 비정상적입니다")
    statuses = []
    for _ in range(count):
        status_seconds = reader.value("i", 4)
        status_nanoseconds = reader.value("I", 4)
        uuid = bytes(reader.value("B", 1) for _ in range(16)).hex()
        module = reader.value("B", 1)
        safe = bool(reader.value("B", 1))
        requested = bool(reader.value("B", 1))
        command_activated = reader.value("B", 1) == 1
        state = reader.value("B", 1)
        auto_mode = bool(reader.value("B", 1))
        start_distance = reader.value("f", 4)
        finish_distance = reader.value("f", 4)
        statuses.append({
            "stamp_ns": status_seconds * 1_000_000_000 +
            status_nanoseconds,
            "uuid": uuid, "module": module, "safe": safe,
            "requested": requested,
            "command_activated": command_activated,
            "state": state, "auto_mode": auto_mode,
            "start_distance": start_distance,
            "finish_distance": finish_distance,
        })
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("RTC 상태 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": seconds * 1_000_000_000 + nanoseconds,
            "statuses": statuses}


def decode_velocity_factors(data):
    """속도 요인의 정지 자세·거리·상태와 설명을 읽습니다."""
    reader = Reader(data)
    stamp, frame = reader.header()
    count = reader.value("I", 4)
    if count > 100_000:
        raise ValueError("속도 요인 수가 비정상적입니다")
    factors = []
    for _ in range(count):
        pose = tuple(reader.value("d", 8) for _ in range(7))
        distance = reader.value("f", 4)
        status = reader.value("H", 2)
        behavior = reader.text()
        sequence = reader.text()
        detail = reader.text()
        cooperation_count = reader.value("I", 4)
        if cooperation_count > 1:
            raise ValueError("속도 요인의 협력 상태 수가 비정상적입니다")
        for _ in range(cooperation_count):
            reader.value("16s", 1)
            reader.value("B", 1)
            reader.value("B", 1)
            reader.value("B", 1)
        factors.append({"pose": pose, "distance": distance,
                        "status": status, "behavior": behavior,
                        "sequence": sequence, "detail": detail})
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("속도 요인 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": stamp, "frame": frame, "factors": factors}


def decode_acceleration(data):
    reader = Reader(data)
    stamp, _ = reader.header()
    linear = tuple(reader.value("d", 8) for _ in range(3))
    return {"stamp_ns": stamp, "acceleration_mps2": linear[0]}


def decode_operation_mode(data):
    """기록된 제어 모드와 Autoware 제어 활성 여부를 읽습니다."""
    reader = Reader(data)
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    mode = reader.value("B", 1)
    enabled = bool(reader.value("B", 1))
    reader.value("B", 1)
    for _ in range(4):
        reader.value("B", 1)
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("제어 모드 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": seconds * 1_000_000_000 + nanoseconds,
            "mode": mode, "is_autoware_control_enabled": enabled}


def decode_velocity_limit(data):
    reader = Reader(data)
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    maximum = reader.value("f", 4)
    if maximum < 0:
        raise ValueError("최고 속도 제한이 잘못되었습니다")
    use_constraints = bool(reader.value("B", 1))
    constraint_offset = reader.offset
    # Older K-City recordings carry min_acceleration/max_jerk/min_jerk;
    # the checked-in message definition also carries max_acceleration.
    try:
        minimum_acceleration = reader.value("f", 4)
        maximum_jerk = reader.value("f", 4)
        minimum_jerk = reader.value("f", 4)
        sender = reader.text()
        if len(data) - reader.offset > 3 or any(data[reader.offset:]):
            raise ValueError("legacy velocity limit layout mismatch")
        maximum_acceleration = None
    except (ValueError, UnicodeDecodeError):
        reader.offset = constraint_offset
        maximum_acceleration = reader.value("f", 4)
        minimum_acceleration = reader.value("f", 4)
        maximum_jerk = reader.value("f", 4)
        minimum_jerk = reader.value("f", 4)
        sender = reader.text()
        if len(data) - reader.offset > 3 or any(data[reader.offset:]):
            raise ValueError("속도 제한 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": seconds * 1_000_000_000 + nanoseconds,
            "max_velocity_mps": maximum,
            "use_constraints": use_constraints,
            "max_acceleration_mps2": maximum_acceleration,
            "min_acceleration_mps2": minimum_acceleration,
            "max_jerk_mps3": maximum_jerk,
            "min_jerk_mps3": minimum_jerk,
            "sender": sender}


def decode_remaining_distance_time(data):
    reader = Reader(data)
    distance = reader.value("d", 8)
    time = reader.value("d", 8)
    if reader.offset != len(data):
        raise ValueError("남은 거리·시간 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"distance_m": distance, "time_s": time}


def decode_scenario(data):
    reader = Reader(data)
    current = reader.text()
    count = reader.value("I", 4)
    if count > 1000:
        raise ValueError("활성 시나리오 수가 비정상적입니다")
    activating = [reader.text() for _ in range(count)]
    if reader.offset == len(data):
        # 저장된 K-City 배포본의 Scenario에는 stamp가 없습니다.
        return {"stamp_ns": None, "current": current,
                "activating": activating}
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("시나리오 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": seconds * 1_000_000_000 + nanoseconds,
            "current": current, "activating": activating}


def decode_metric_array(data):
    reader = Reader(data)
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    count = reader.value("I", 4)
    if count > 100_000:
        raise ValueError("지표 수가 비정상적입니다")
    metrics = [(reader.text(), reader.text(), reader.text())
               for _ in range(count)]
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("지표 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": seconds * 1_000_000_000 + nanoseconds,
            "metrics": metrics}


def decode_diagnostic_array(data):
    """기록된 planning evaluator의 DiagnosticArray 지표를 읽습니다."""
    reader = Reader(data)
    stamp, frame = reader.header()
    count = reader.value("I", 4)
    if count > 100_000:
        raise ValueError("진단 상태 수가 비정상적입니다")
    statuses = []
    for _ in range(count):
        level = reader.value("B", 1)
        name = reader.text()
        message = reader.text()
        hardware_id = reader.text()
        value_count = reader.value("I", 4)
        if value_count > 100_000:
            raise ValueError("진단 지표 수가 비정상적입니다")
        values = {reader.text(): reader.text()
                  for _ in range(value_count)}
        statuses.append({"level": level, "name": name,
                         "message": message,
                         "hardware_id": hardware_id,
                         "values": values})
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("진단 배열 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": stamp, "frame": frame, "statuses": statuses}


def decode_float32_stamped(data):
    reader = Reader(data)
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    value = reader.value("f", 4)
    if reader.offset != len(data):
        raise ValueError("실수 상태 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": seconds * 1_000_000_000 + nanoseconds,
            "value": value}


def decode_planning_validator_status(data):
    reader = Reader(data)
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    flags = (
        "is_valid_size", "is_valid_finite_value", "is_valid_interval",
        "is_valid_relative_angle", "is_valid_curvature",
        "is_valid_lateral_acc", "is_valid_longitudinal_max_acc",
        "is_valid_longitudinal_min_acc", "is_valid_steering",
        "is_valid_steering_rate", "is_valid_velocity_deviation",
        "is_valid_distance_deviation",
        "is_valid_longitudinal_distance_deviation",
        "is_valid_forward_trajectory_length")
    result = {name: bool(reader.value("B", 1)) for name in flags}
    result["trajectory_size"] = reader.value("q", 8)
    values = (
        "max_interval_distance", "max_relative_angle", "max_curvature",
        "max_lateral_acc", "max_longitudinal_acc", "min_longitudinal_acc",
        "max_steering", "max_steering_rate", "velocity_deviation",
        "distance_deviation", "longitudinal_distance_deviation",
        "forward_trajectory_length_required",
        "forward_trajectory_length_measured")
    result.update({name: reader.value("d", 8) for name in values})
    result["invalid_count"] = reader.value("q", 8)
    if reader.offset != len(data):
        raise ValueError("validator 상태 메시지 끝에 해석되지 않은 내용이 있습니다")
    result["stamp_ns"] = seconds * 1_000_000_000 + nanoseconds
    return result


def decode_lanelet_map(data):
    reader = Reader(data)
    reader.header()
    for _ in range(3):
        reader.text()
    length = reader.value("I", 4)
    if reader.offset + length > len(data):
        raise ValueError("지도 이진 자료 길이가 잘못되었습니다")
    result = data[reader.offset:reader.offset + length]
    reader.offset += length
    if len(data) - reader.offset > 3 or any(data[reader.offset:]):
        raise ValueError("지도 메시지 끝에 해석되지 않은 내용이 있습니다")
    return result


def decode_trajectory(data):
    reader = Reader(data)
    stamp, frame = reader.header()
    count = reader.value("I", 4)
    if count > 1_000_000:
        raise ValueError("궤적 점 수가 비정상적입니다")
    points = []
    for _ in range(count):
        seconds = reader.value("i", 4)
        nanoseconds = reader.value("I", 4)
        pose = tuple(reader.value("d", 8) for _ in range(7))
        motion = tuple(reader.value("f", 4) for _ in range(6))
        points.append({
            "x": pose[0], "y": pose[1], "z": pose[2],
            "orientation_x": pose[3], "orientation_y": pose[4],
            "orientation_z": pose[5], "orientation_w": pose[6],
            "velocity": motion[0], "acceleration": motion[2],
            "heading_rate": motion[3],
            "front_wheel_angle": motion[4],
            "time_from_start": seconds + nanoseconds * 1e-9})
    if reader.offset != len(data):
        raise ValueError("궤적 메시지 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": stamp, "frame": frame, "points": points}


def decode_float32_multi_array_stamped(data):
    """계획 디버그 값의 시각과 실수 배열을 읽습니다."""
    reader = Reader(data)
    seconds = reader.value("i", 4)
    nanoseconds = reader.value("I", 4)
    dimension_count = reader.value("I", 4)
    if dimension_count > 1000:
        raise ValueError("디버그 배열 차원 수가 비정상적입니다")
    for _ in range(dimension_count):
        reader.text()
        reader.value("I", 4)
        reader.value("I", 4)
    reader.value("I", 4)
    count = reader.value("I", 4)
    if count > 100_000:
        raise ValueError("디버그 배열 길이가 비정상적입니다")
    values = [reader.value("f", 4) for _ in range(count)]
    if reader.offset != len(data):
        raise ValueError("디버그 배열 끝에 해석되지 않은 내용이 있습니다")
    return {"stamp_ns": seconds * 1_000_000_000 + nanoseconds,
            "data": values}


def messages(bag, topic):
    connection = sqlite3.connect(f"file:{Path(bag)}?mode=ro", uri=True)
    try:
        for timestamp, data in connection.execute(
                "SELECT messages.timestamp, messages.data FROM messages "
                "JOIN topics ON topics.id = messages.topic_id "
                "WHERE topics.name = ? ORDER BY messages.timestamp", (topic,)):
            yield timestamp, data
    finally:
        connection.close()

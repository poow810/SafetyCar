import cv2
from ultralytics import YOLO
import time
import asyncio
from main import send_coordinate
from udp import UdpSender


# 수평으로 쓰러졌을 때
def is_aligned_nose(nose_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    if nose_y is None or left_ankle_y is None or right_ankle_y is None or left_knee_y is None or right_knee_y is None:
        return False
    
    if nose_y < left_ankle_y or nose_y < right_ankle_y or nose_y < left_knee_y or nose_y < right_knee_y:
        return True

    valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y != 0.0 and y > 0]
    average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None

    valid_knees = [y for y in [left_knee_y, right_knee_y] if y != 0.0 and y > 0]
    average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

    if nose_y != 0.0 and average_ankle_y is not None and average_knee_y is not None:
        y_diff_ankle = abs(nose_y - average_ankle_y)
        y_diff_knee = abs(average_ankle_y - average_knee_y)

        if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
            return True
    return False


def is_aligned_ear(left_ear_y, right_ear_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    valid_ears = [y for y in [left_ear_y, right_ear_y] if y is not None and y > 0]
    average_ears_y = sum(valid_ears) / len(valid_ears) if valid_ears else None

    if average_ears_y is None or left_ankle_y is None or right_ankle_y is None or left_knee_y is None or right_knee_y is None:
        return False

    if average_ears_y < left_ankle_y or average_ears_y < right_ankle_y or average_ears_y < left_knee_y or average_ears_y < right_knee_y:
        return True

    valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y is not None and y > 0]
    average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None

    valid_knees = [y for y in [left_knee_y, right_knee_y] if y is not None and y > 0]
    average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

    if average_ears_y is not None and average_ankle_y is not None and average_knee_y is not None:
        y_diff_ankle = abs(average_ears_y - average_ankle_y)
        y_diff_knee = abs(average_ankle_y - average_knee_y)

        if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
            return True

    return False


def is_aligned_eye(left_eye_y, right_eye_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    valid_eyes = [y for y in [left_eye_y, right_eye_y] if y is not None and y > 0]
    average_eyes_y = sum(valid_eyes) / len(valid_eyes) if valid_eyes else None

    if average_eyes_y is None or left_ankle_y is None or right_ankle_y is None or left_knee_y is None or right_knee_y is None:
        return False

    if average_eyes_y < left_ankle_y or average_eyes_y < right_ankle_y or average_eyes_y < left_knee_y or average_eyes_y < right_knee_y:
        return True

    valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y is not None and y > 0]
    average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None

    valid_knees = [y for y in [left_knee_y, right_knee_y] if y is not None and y > 0]
    average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

    if average_eyes_y is not None and average_ankle_y is not None and average_knee_y is not None:
        y_diff_ankle = abs(average_eyes_y - average_ankle_y)
        y_diff_knee = abs(average_ankle_y - average_knee_y)

        if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
            return True
        
    return False


def is_falling_func(width, height, keypoint_data, is_falling):

    nose_y = keypoint_data[0][0][1].item() if keypoint_data[0][0][2].numel() == 1 and keypoint_data[0][0][2] > 0 else None
    left_eye_y = keypoint_data[0][1][1].item() if keypoint_data[0][1][2].numel() == 1 and keypoint_data[0][1][2] > 0 else None 
    right_eye_y = keypoint_data[0][2][1].item() if keypoint_data[0][2][2].numel() == 1 and keypoint_data[0][2][2] > 0 else None
    left_ear_y = keypoint_data[0][3][1].item() if keypoint_data[0][3][2].numel() == 1 and keypoint_data[0][3][2] > 0 else None
    right_ear_y = keypoint_data[0][4][1].item() if keypoint_data[0][4][2].numel() == 1 and keypoint_data[0][4][2] > 0 else None
    left_knee_y = keypoint_data[0][13][1].item() if keypoint_data[0][13][2].numel() == 1 and keypoint_data[0][13][2] > 0 else None
    right_knee_y = keypoint_data[0][14][1].item() if keypoint_data[0][14][2].numel() == 1 and keypoint_data[0][14][2] > 0 else None
    left_ankle_y = keypoint_data[0][15][1].item() if keypoint_data[0][15][2].numel() == 1 and keypoint_data[0][15][2] > 0 else None
    right_ankle_y = keypoint_data[0][16][1].item() if keypoint_data[0][16][2].numel() == 1 and keypoint_data[0][16][2] > 0 else None


    if left_ankle_y is None and left_knee_y is None and right_ankle_y is None and right_knee_y is None:
        return False
    else:
        aspect_ratio_threshold = 1.2

        if width > height * aspect_ratio_threshold:
            if nose_y is not None and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
                if is_aligned_nose(nose_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                    return True

            if not is_falling and (left_ear_y is not None or right_ear_y is not None) and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
                if is_aligned_ear(left_ear_y, right_ear_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                    return True
            
            if not is_falling and (left_eye_y is not None or right_eye_y is not None) and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
                if is_aligned_eye(left_eye_y, right_eye_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                    return True

    return False


async def pose_estimation(keypoints, results, tracking_data, annotated_frame):
    
    send_x, send_y = None, None
    start_x, start_y, end_x, end_y = None, None, None, None

    # 바운딩 박스 좌표 추출
    if results[0].boxes is not None and len(results[0].boxes) > 0:
        for i, box in enumerate(results[0].boxes):
            if box is None or box.id is None:
                continue

            tracking_id = box.id.item()
            x1, y1, x2, y2 = box.xyxy[0]
            width = box.xywh[0][2].item()
            height = box.xywh[0][3].item()

            if keypoints is not None and len(keypoints) > i:
                keypoint_data = keypoints[i].data

                if keypoint_data.size(1) == 17:
                    # 각 키포인트의 좌표 추출
                    is_falling = False
                    is_falling = is_falling_func(width, height, keypoint_data, is_falling)

                    current_time = time.time()

                    status_type = 2

                    if is_falling:
                        if tracking_id not in tracking_data:
                            tracking_data[tracking_id] = {'start_time': current_time, 'sent': False}
                        else:
                            elapsed_time = current_time - tracking_data[tracking_id]['start_time']
                            if elapsed_time >= 5:
                                status_type = 0
                                
                                # 객체 정중앙 좌표
                                # center_x = (x1 + x2) / 2
                                # center_y = (y1 + y2) / 2

                                # 객체 발 밑 좌표
                                send_x, send_y = (x1 + x2) / 2, y2
                                start_x, start_y, end_x, end_y = x1, y1, x2, y2
                                if not tracking_data[tracking_id]['sent']:
                                    await send_coordinate(float(send_x), float(send_y))
                                    tracking_data[tracking_id]['sent'] = True
                            else:
                                status_type = 1

                    else:
                        tracking_data.pop(tracking_id, None)
                        status_type = 2

                    status_list = [{'text': 'Falling', 'color': (0, 0, 255)},
                                   {'text': 'Falling (not confirmed)', 'color': (0, 255, 255)},
                                   {'text': 'standing', 'color': (255, 0, 0)}]
                    status = status_list[status_type]

                    # 쓰러진 상태일 때만 현재 상태를 이미지에 표시
                    cv2.putText(annotated_frame, status['text'], (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, status['color'], 2)
    lst = []
    if start_x is not None:
        lst.append((int(start_x), int(start_y), int(end_x), int(end_y)))
    return (annotated_frame, lst)
            

async def process_video(udp_sender):
    model = YOLO("model/yolov8s-pose.pt").to('cuda')

    # 동영상 파일 열기
    video_path = "falling.mp4"
    cap = cv2.VideoCapture(0)

    # 객체 상태 추적
    tracking_data = {}

    # 프레임 건너뛰기 설정
    frame_skip = 2  # 2프레임마다 처리

    while True:
        # 프레임 건너뛰기
        for _ in range(frame_skip):
            ret, frame = cap.read()
            if not ret:
                break

        if not ret:
            break

        # 이미지에서 포즈 추정 수행
        results = model.track(frame, persist=True)

        # 결과를 이미지에 표시
        annotated_frame = results[0].plot()

        # 스켈레톤 데이터 추출
        keypoints = results[0].keypoints

        annotated_frame, bounding_box = await pose_estimation(keypoints, results, tracking_data, annotated_frame)
        

        if bounding_box:
            for (x1, y1, x2, y2) in bounding_box:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # 이미지 출력
        cv2.imshow('Pose Estimation', annotated_frame)

        udp_sender.send_frame(annotated_frame)

        # 'q' 키를 눌러 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    SERVER_IP = "43.202.61.242"
    PORT = 5432
    camera_id = 0
    udp_sender = UdpSender(SERVER_IP, PORT, camera_id)
    
    try:
        asyncio.run(process_video(udp_sender))
    finally:
        udp_sender.close()
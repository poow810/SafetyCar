import cv2
from ultralytics import YOLO
import time
import asyncio
from main import send_coordinate

# 수평으로 쓰러졌을 때
def is_aligned_nose(nose_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    if nose_y < left_ankle_y or nose_y < right_ankle_y or nose_y < left_knee_y or nose_y < right_knee_y:
        return True

    else:
        valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y != 0.0 and y > 0]
        average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None
        
        valid_knees = [y for y in [left_knee_y, right_knee_y] if y != 0.0 and y > 0]
        average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

        if nose_y != 0.0 and average_ankle_y != 0.0 and average_knee_y != 0.0:
            y_diff_ankle = abs(nose_y - average_ankle_y)
            y_diff_knee = abs(average_ankle_y - average_knee_y)

            if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
                return True
    return False


def is_aligned_ear(left_ear_y, right_ear_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    valid_ears = [y for y in [left_ear_y, right_ear_y] if y != 0.0 and y > 0]
    average_ears_y = sum(valid_ears) / len(valid_ears) if valid_ears else None

    if average_ears_y < left_ankle_y or average_ears_y < right_ankle_y or average_ears_y < left_knee_y or average_ears_y < right_knee_y:
        return True

    else:
        valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y != 0.0 and y > 0]
        average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None
        
        valid_knees = [y for y in [left_knee_y, right_knee_y] if y != 0.0 and y > 0]
        average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

        if average_ears_y != 0.0 and average_ankle_y != 0.0 and average_knee_y != 0.0:
            y_diff_ankle = abs(average_ears_y - average_ankle_y)
            y_diff_knee = abs(average_ankle_y - average_knee_y)

            if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
                return True
    return False


def is_aligned_eye(left_eye_y, right_eye_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y, slope_threshold=40):
    valid_eyes = [y for y in [left_eye_y, right_eye_y] if y != 0.0 and y > 0]
    average_eyes_y = sum(valid_eyes) / len(valid_eyes) if valid_eyes else None

    if average_eyes_y < left_ankle_y or average_eyes_y < right_ankle_y or average_eyes_y < left_knee_y or average_eyes_y < right_knee_y:
        return True

    else:
        valid_ankles = [y for y in [left_ankle_y, right_ankle_y] if y != 0.0 and y > 0]
        average_ankle_y = sum(valid_ankles) / len(valid_ankles) if valid_ankles else None
        
        valid_knees = [y for y in [left_knee_y, right_knee_y] if y != 0.0 and y > 0]
        average_knee_y = sum(valid_knees) / len(valid_knees) if valid_knees else None

        if average_eyes_y != 0.0 and average_ankle_y != 0.0 and average_knee_y != 0.0:
            y_diff_ankle = abs(average_eyes_y - average_ankle_y)
            y_diff_knee = abs(average_ankle_y - average_knee_y)

            if y_diff_ankle < slope_threshold and y_diff_knee < slope_threshold:
                return True
    return False

async def process_video():
    model = YOLO("model/yolov8s-pose.pt").to('cuda')

    # 동영상 파일 열기
    video_path = "falling2.mp4"
    cap = cv2.VideoCapture(video_path)

    # 객체 상태 추적
    tracking_data = {}

    # 기울기 임계값 설정
    slope_threshold = 0.6

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

        # 바운딩 박스 좌표 추출
        if results[0].boxes is not None and len(results[0].boxes) > 0:
            for i, box in enumerate(results[0].boxes):
                if box is None or box.id is None:
                    continue

                tracking_id = box.id.item()
                x1, y1, x2, y2 = box.xyxy[0]
                width = box.xywh[0][2].item()
                height = box.xywh[0][3].item()

                aspect_ratio_threshold = 1.2

                if keypoints is not None and len(keypoints) > i:
                    keypoint_data = keypoints[i].data

                    if keypoint_data.size(1) == 17:
                        # 각 키포인트의 좌표 추출
                        nose_y = keypoint_data[0][0][1].item() if keypoint_data[0][0][2].numel() == 1 and keypoint_data[0][0][2] > 0 else None
                        left_eye_y = keypoint_data[0][1][1].item() if keypoint_data[0][1][2].numel() == 1 and keypoint_data[0][1][2] > 0 else None 
                        right_eye_y = keypoint_data[0][2][1].item() if keypoint_data[0][2][2].numel() == 1 and keypoint_data[0][2][2] > 0 else None
                        left_ear_y = keypoint_data[0][3][1].item() if keypoint_data[0][3][2].numel() == 1 and keypoint_data[0][3][2] > 0 else None
                        right_ear_y = keypoint_data[0][4][1].item() if keypoint_data[0][4][2].numel() == 1 and keypoint_data[0][4][2] > 0 else None
                        left_knee_y = keypoint_data[0][13][1].item() if keypoint_data[0][13][2].numel() == 1 and keypoint_data[0][13][2] > 0 else None
                        right_knee_y = keypoint_data[0][14][1].item() if keypoint_data[0][14][2].numel() == 1 and keypoint_data[0][14][2] > 0 else None
                        left_ankle_y = keypoint_data[0][15][1].item() if keypoint_data[0][15][2].numel() == 1 and keypoint_data[0][15][2] > 0 else None
                        right_ankle_y = keypoint_data[0][16][1].item() if keypoint_data[0][16][2].numel() == 1 and keypoint_data[0][16][2] > 0 else None


                        # 쓰러진 사람 판별
                        is_falling = False
                        status = "Standing"
                        color = (255, 0, 0)

                        if left_ankle_y is None and left_knee_y is None and right_ankle_y is None and right_knee_y is None:
                            status = "Standing"
                        else:
                            if width > height * aspect_ratio_threshold:
                                if nose_y is not None and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
                                    if is_aligned_nose(nose_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                                        is_falling = True

                                if not is_falling and (left_ear_y is not None or right_ear_y is not None) and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
                                    if is_aligned_ear(left_ear_y, right_ear_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                                        is_falling = True
                                
                                if not is_falling and (left_eye_y is not None or right_eye_y is not None) and (left_ankle_y is not None or right_ankle_y is not None) and (left_knee_y is not None or right_knee_y is not None):
                                    if is_aligned_eye(left_eye_y, right_eye_y, left_ankle_y, right_ankle_y, left_knee_y, right_knee_y):
                                        is_falling = True

                        current_time = time.time()
                        if is_falling:
                            if tracking_id not in tracking_data:
                                tracking_data[tracking_id] = {'start_time': current_time, 'sent': False}
                            else:
                                elapsed_time = current_time - tracking_data[tracking_id]['start_time']
                                if elapsed_time >= 5:
                                    status = "Falling"
                                    color = (0, 0, 255)
                                    center_x = (x1 + x2) / 2
                                    center_y = (y1 + y2) / 2
                                    if not tracking_data[tracking_id]['sent']:
                                        await send_coordinate(float(center_x), float(center_y))
                                        tracking_data[tracking_id]['sent'] = True
                                else:
                                    status = "Falling (not confirmed)"
                                    color = (0, 255, 255)
                        else:
                            if tracking_id in tracking_data:
                                del tracking_data[tracking_id]  # 상태 초기화
                            status = "Standing"
                            color = (255, 0, 0)

                        # 현재 상태를 이미지에 표시
                        cv2.putText(annotated_frame, status, (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    else:
                        print("포즈가 검출되지 않았습니다.")

        # 이미지 출력
        cv2.imshow('Pose Estimation', annotated_frame)

        # 'q' 키를 눌러 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(process_video())
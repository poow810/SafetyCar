from fastapi import FastAPI, UploadFile, File, Form
from fastapi.middleware.cors import CORSMiddleware
from socketHandler import socket_app, sio
import cv2
import numpy as np
import base64
import json

app = FastAPI(root_path="/pyapi")

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 실제 배포 시에는 허용할 도메인을 명시적으로 지정해야 합니다.
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount('/', socket_app)

# 전역 변수 및 상태 초기화
state = {
    'step': 1,
    'pts1_floor': [],
    'pts2_floor': [],
    'pts1_tile': [],
    'pts2_tile': [],
    'pts1_align': [],
    'pts2_align': [],
    'H1_total': None,
    'H2_total': None,
    'M1_adjust': np.eye(3, dtype=np.float32),
    'M2_adjust': np.eye(3, dtype=np.float32),
}

# 이미지 데이터를 저장하기 위한 변수
images = {
    'original_frame1': None,
    'original_frame2': None,
    'warped1_floor': None,
    'warped2_floor': None,
    'final_warped1': None,
    'final_warped2': None,
    'merged_image': None
}

# 유틸리티 함수들
def encode_image_to_base64(image):
    _, buffer = cv2.imencode('.jpg', image)
    img_str = base64.b64encode(buffer).decode('utf-8')
    return img_str

def rotate_image(image, angle):
    (h, w) = image.shape[:2]
    center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))
    M[0, 2] += (new_w / 2) - center[0]
    M[1, 2] += (new_h / 2) - center[1]
    rotated_image = cv2.warpAffine(image, M, (new_w, new_h))
    M_full = np.eye(3, dtype=np.float32)
    M_full[:2, :] = M
    return rotated_image, M_full

def flip_image(image, flip_code):
    flipped_image = cv2.flip(image, flip_code)
    h, w = image.shape[:2]
    if flip_code == 0:
        # 상하 반전
        M = np.array([[1, 0, 0],
                      [0, -1, h],
                      [0, 0, 1]], dtype=np.float32)
    elif flip_code == 1:
        # 좌우 반전
        M = np.array([[-1, 0, w],
                      [0, 1, 0],
                      [0, 0, 1]], dtype=np.float32)
    else:
        # 상하좌우 반전
        M = np.array([[-1, 0, w],
                      [0, -1, h],
                      [0, 0, 1]], dtype=np.float32)
    return flipped_image, M

def compute_homography_floor():
    # 클릭한 4개의 점을 좌표 배열로 변환
    pts1_floor = np.array(state['pts1_floor'], dtype=np.float32)
    pts2_floor = np.array(state['pts2_floor'], dtype=np.float32)

    # 원본 이미지의 크기로 목적지 좌표 설정
    h1, w1 = images['original_frame1'].shape[:2]
    h2, w2 = images['original_frame2'].shape[:2]
    dst_corners1 = np.array([
        [0, 0],
        [w1 - 1, 0],
        [w1 - 1, h1 - 1],
        [0, h1 - 1]
    ], dtype=np.float32)
    dst_corners2 = np.array([
        [0, 0],
        [w2 - 1, 0],
        [w2 - 1, h2 - 1],
        [0, h2 - 1]
    ], dtype=np.float32)

    # 호모그래피 계산
    H1_floor, status1 = cv2.findHomography(pts1_floor, dst_corners1)
    H2_floor, status2 = cv2.findHomography(pts2_floor, dst_corners2)

    if H1_floor is None or H2_floor is None:
        print("호모그래피 계산에 실패했습니다.")
        return

    # 이미지 변환 (원본 크기로)
    warped1_floor = cv2.warpPerspective(images['original_frame1'], H1_floor, (w1, h1), borderValue=(0, 0, 0))
    warped2_floor = cv2.warpPerspective(images['original_frame2'], H2_floor, (w2, h2), borderValue=(0, 0, 0))

    # 변환 행렬 저장
    state['H1_total'] = H1_floor.copy()
    state['H2_total'] = H2_floor.copy()
    images['warped1_floor'] = warped1_floor
    images['warped2_floor'] = warped2_floor

def measure_tile_size_and_update_scale():
    # 타일 크기 측정
    width1, height1 = get_tile_size(state['pts1_tile'])
    width2, height2 = get_tile_size(state['pts2_tile'])

    print(f"타일 크기 이미지1: 가로={width1}, 세로={height1}")
    print(f"타일 크기 이미지2: 가로={width2}, 세로={height2}")

    # 각 이미지에서 타일이 정사각형이 되도록 스케일 팩터 계산
    scale_x1 = 1.0
    scale_y1 = width1 / height1  # 이미지 1의 세로 방향 스케일 팩터

    scale_x2 = 1.0
    scale_y2 = width2 / height2  # 이미지 2의 세로 방향 스케일 팩터

    # 스케일링 후의 타일 크기 계산
    scaled_tile_size1 = width1 * scale_x1  # width1 * 1.0 = width1
    scaled_tile_size2 = width2 * scale_x2  # width2 * 1.0 = width2

    # 두 이미지의 타일 크기를 동일하게 맞추기 위한 추가 스케일 팩터 계산
    tile_scale_factor = scaled_tile_size1 / scaled_tile_size2

    # 이미지 2에 추가 스케일 팩터 적용
    scale_x2 *= tile_scale_factor
    scale_y2 *= tile_scale_factor

    print(f"이미지 1 스케일 팩터: scale_x={scale_x1}, scale_y={scale_y1}")
    print(f"이미지 2 스케일 팩터 (조정됨): scale_x={scale_x2}, scale_y={scale_y2}")

    # 스케일링 변환 행렬 생성
    scaling_matrix1 = np.array([
        [scale_x1, 0, 0],
        [0, scale_y1, 0],
        [0, 0, 1]
    ], dtype=np.float32)

    scaling_matrix2 = np.array([
        [scale_x2, 0, 0],
        [0, scale_y2, 0],
        [0, 0, 1]
    ], dtype=np.float32)

    # 전체 변환 행렬 업데이트
    state['H1_total'] = scaling_matrix1 @ state['H1_total']
    state['H2_total'] = scaling_matrix2 @ state['H2_total']

    # 이미지 스케일링 적용
    h1, w1 = images['warped1_floor'].shape[:2]
    h2, w2 = images['warped2_floor'].shape[:2]

    new_w1 = int(w1 * scale_x1)
    new_h1 = int(h1 * scale_y1)
    new_w2 = int(w2 * scale_x2)
    new_h2 = int(h2 * scale_y2)

    images['warped1_floor'] = cv2.resize(images['warped1_floor'], (new_w1, new_h1), interpolation=cv2.INTER_LINEAR)
    images['warped2_floor'] = cv2.resize(images['warped2_floor'], (new_w2, new_h2), interpolation=cv2.INTER_LINEAR)

def get_tile_size(pts_tile):
    # 바운딩 박스 크기 계산
    pts_tile = np.array(pts_tile)
    x_min, y_min = np.min(pts_tile, axis=0)
    x_max, y_max = np.max(pts_tile, axis=0)
    width = x_max - x_min
    height = y_max - y_min
    return width, height

def align_images():
    # 좌표 배열로 변환
    pts1_align = np.array(state['pts1_align'], dtype=np.float32)
    pts2_align = np.array(state['pts2_align'], dtype=np.float32)

    # 호모그래피 계산
    H_align, status = cv2.findHomography(pts2_align, pts1_align, cv2.RANSAC)

    if H_align is None:
        print("H_align 호모그래피 계산에 실패했습니다.")
        return

    # 변환 행렬 업데이트
    state['H2_total'] = H_align @ state['H2_total']

    # 코너 점 변환
    transformed_corners1 = get_transformed_corners(images['warped1_floor'], np.eye(3))
    transformed_corners2 = get_transformed_corners(images['warped2_floor'], H_align)

    # 모든 코너를 하나의 배열로 결합
    all_corners = np.vstack((transformed_corners1, transformed_corners2))

    # 바운딩 박스 계산
    [x_min, y_min] = np.int32(all_corners.min(axis=0).ravel() - 0.5)
    [x_max, y_max] = np.int32(all_corners.max(axis=0).ravel() + 0.5)

    # 평행 이동 변환 행렬 생성
    translation_matrix = np.array([
        [1, 0, -x_min],
        [0, 1, -y_min],
        [0, 0, 1]
    ], dtype=np.float32)

    # 전체 변환 행렬 업데이트
    state['H1_total'] = translation_matrix @ state['H1_total']
    state['H2_total'] = translation_matrix @ state['H2_total']

    # 최종 이미지 크기 계산
    merged_width = x_max - x_min
    merged_height = y_max - y_min

    # 원하는 최종 바닥 크기 설정 (가로=500, 세로=1300)
    floor_width = 500
    floor_height = 1300

    # 스케일 팩터 계산
    scale_x = floor_width / merged_width
    scale_y = floor_height / merged_height

    print(f"스케일 팩터: scale_x={scale_x:.4f}, scale_y={scale_y:.4f}")

    # 스케일링 변환 행렬 생성
    scaling_matrix = np.array([
        [scale_x, 0, 0],
        [0, scale_y, 0],
        [0, 0, 1]
    ], dtype=np.float32)

    # 전체 변환 행렬에 스케일링 적용
    state['H1_total'] = scaling_matrix @ state['H1_total']
    state['H2_total'] = scaling_matrix @ state['H2_total']

    # 출력 이미지 크기 설정
    output_size_final_adjusted = (floor_width, floor_height)

    # 최종 변환된 이미지 생성
    final_warped1 = cv2.warpPerspective(images['original_frame1'], state['H1_total'], output_size_final_adjusted, borderValue=(0, 0, 0))
    final_warped2 = cv2.warpPerspective(images['original_frame2'], state['H2_total'], output_size_final_adjusted, borderValue=(0, 0, 0))

    # 이미지 합성 (최대값 기준)
    merged_image = np.maximum(final_warped1, final_warped2)

    images['final_warped1'] = final_warped1
    images['final_warped2'] = final_warped2
    images['merged_image'] = merged_image

def get_transformed_corners(image, H):
    h, w = image.shape[:2]
    corners = np.array([
        [0, 0],
        [w, 0],
        [w, h],
        [0, h]
    ], dtype=np.float32).reshape(-1, 1, 2)
    transformed_corners = cv2.perspectiveTransform(corners, H)
    return transformed_corners.reshape(-1, 2)

def map_point_to_floor_coordinates(x, y, H_total):
    point = np.array([[[x, y]]], dtype=np.float32)
    transformed_point = cv2.perspectiveTransform(point, H_total)
    return transformed_point[0][0]

# 1. 클라이언트로부터 이미지와 바닥의 네 끝점 좌표를 받는 엔드포인트
@app.post("/upload_images/")
async def upload_images(
    image1: UploadFile = File(...),
    image2: UploadFile = File(...),
    pts1_floor: str = Form(...),
    pts2_floor: str = Form(...)
):
    # 이미지 읽기
    contents1 = await image1.read()
    contents2 = await image2.read()
    nparr1 = np.frombuffer(contents1, np.uint8)
    nparr2 = np.frombuffer(contents2, np.uint8)
    frame1 = cv2.imdecode(nparr1, cv2.IMREAD_COLOR)
    frame2 = cv2.imdecode(nparr2, cv2.IMREAD_COLOR)

    images['original_frame1'] = frame1.copy()
    images['original_frame2'] = frame2.copy()

    # 바닥의 네 끝점 좌표 파싱
    pts1_floor_list = json.loads(pts1_floor)
    pts2_floor_list = json.loads(pts2_floor)

    state['pts1_floor'] = pts1_floor_list
    state['pts2_floor'] = pts2_floor_list

    # 단계 진행
    compute_homography_floor()
    state['step'] = 2

    # 변환된 이미지를 클라이언트로 반환
    image1_str = encode_image_to_base64(images['warped1_floor'])
    image2_str = encode_image_to_base64(images['warped2_floor'])

    return {
        'step': state['step'],
        'image1': image1_str,
        'image2': image2_str
    }

# 2. 클라이언트로부터 회전/반전 명령을 받는 엔드포인트
@app.post("/adjust_images/")
async def adjust_images(
    key: str = Form(...),
    img_id: int = Form(...)
):
    current_image = 'warped1_floor' if img_id == 1 else 'warped2_floor'
    M_adjust = 'M1_adjust' if img_id == 1 else 'M2_adjust'

    if key == 'r':
        rotated_image, M_rot = rotate_image(images[current_image], -90)
        images[current_image] = rotated_image
        state[M_adjust] = M_rot @ state[M_adjust]
        print(f"이미지 {img_id}을 90도 시계 방향으로 회전.")
    elif key == 'e':
        rotated_image, M_rot = rotate_image(images[current_image], 90)
        images[current_image] = rotated_image
        state[M_adjust] = M_rot @ state[M_adjust]
        print(f"이미지 {img_id}을 90도 반시계 방향으로 회전.")
    elif key == 'h':
        flipped_image, M_flip = flip_image(images[current_image], 1)
        images[current_image] = flipped_image
        state[M_adjust] = M_flip @ state[M_adjust]
        print(f"이미지 {img_id}을 좌우 반전.")
    elif key == 'v':
        flipped_image, M_flip = flip_image(images[current_image], 0)
        images[current_image] = flipped_image
        state[M_adjust] = M_flip @ state[M_adjust]
        print(f"이미지 {img_id}을 상하 반전.")
    elif key == 'n':
        # 회전/반전 변환 행렬을 기존 변환 행렬에 곱하여 전체 변환 행렬을 업데이트
        if img_id == 1:
            state['H1_total'] = state[M_adjust] @ state['H1_total']
            state[M_adjust] = np.eye(3, dtype=np.float32)  # 변환 행렬 초기화
            images['warped1_floor'] = images[current_image]
        else:
            state['H2_total'] = state[M_adjust] @ state['H2_total']
            state[M_adjust] = np.eye(3, dtype=np.float32)  # 변환 행렬 초기화
            images['warped2_floor'] = images[current_image]
        state['step'] = 3  # 다음 단계로 진행
        print("회전/반전 단계 완료. 다음 단계로 진행.")

    # 변환된 이미지를 클라이언트로 반환
    image1_str = encode_image_to_base64(images['warped1_floor'])
    image2_str = encode_image_to_base64(images['warped2_floor'])

    return {
        'step': state['step'],
        'image1': image1_str,
        'image2': image2_str
    }

# 3. 클라이언트로부터 타일 모서리 좌표를 받는 엔드포인트
@app.post("/upload_tile_points/")
async def upload_tile_points(
    pts1_tile: str = Form(...),
    pts2_tile: str = Form(...)
):
    # 좌표 파싱
    pts1_tile_list = json.loads(pts1_tile)
    pts2_tile_list = json.loads(pts2_tile)

    state['pts1_tile'] = pts1_tile_list
    state['pts2_tile'] = pts2_tile_list

    # 단계 진행
    measure_tile_size_and_update_scale()
    state['step'] = 4

    # 변환된 이미지를 클라이언트로 반환
    image1_str = encode_image_to_base64(images['warped1_floor'])
    image2_str = encode_image_to_base64(images['warped2_floor'])

    return {
        'step': state['step'],
        'image1': image1_str,
        'image2': image2_str
    }

# 4. 클라이언트로부터 이미지 합성을 위한 대응점 좌표를 받는 엔드포인트
@app.post("/upload_align_points/")
async def upload_align_points(
    pts1_align: str = Form(...),
    pts2_align: str = Form(...)
):
    # 좌표 파싱
    pts1_align_list = json.loads(pts1_align)
    pts2_align_list = json.loads(pts2_align)

    state['pts1_align'] = pts1_align_list
    state['pts2_align'] = pts2_align_list

    # 단계 진행
    align_images()
    state['step'] = 5

    # 합성된 이미지를 반환
    image_str = encode_image_to_base64(images['merged_image'])

    return {
        'step': state['step'],
        'merged_image': image_str
    }

# 5. 클라이언트로부터 클릭 좌표를 받아서 바닥의 가상 좌표를 반환하는 엔드포인트
@app.post("/get_floor_coordinates/")
async def get_floor_coordinates(
    x: float = Form(...),
    y: float = Form(...),
    img_id: int = Form(...)
):
    H_total = state['H1_total'] if img_id == 1 else state['H2_total']
    x_floor, y_floor = map_point_to_floor_coordinates(x, y, H_total)

    # 합성된 이미지에서의 좌표를 최종 바닥 크기에 맞게 스케일링
    floor_width = 500
    floor_height = 1300

    # 좌표를 바닥 크기에 맞게 제한
    x_floor = max(0, min(floor_width, x_floor))
    y_floor = max(0, min(floor_height, y_floor))

    # numpy.float32 타입을 Python의 float 타입으로 변환
    x_floor = float(x_floor)
    y_floor = float(y_floor)
    await sio.emit('gridmake', data=[x_floor, y_floor], namespace='/socketio')
    
    return {
        'x_floor': x_floor,
        'y_floor': y_floor
    }

# 서버 실행
if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host='0.0.0.0', port=8000)

import cv2
import numpy as np
import os

# Parâmetros do tabuleiro de xadrez
chessboard_size = (10, 7)  # Número de cantos internos no tabuleiro (largura x altura)

# Preparar pontos do objeto 3D
objp = np.zeros((np.prod(chessboard_size), 3), dtype=np.float32)
objp[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2)

# Listas para armazenar pontos do objeto 3D e pontos da imagem 2D
object_points = []
image_points = []

# Diretório para salvar as imagens capturadas
save_dir = 'calibration_images'
os.makedirs(save_dir, exist_ok=True)

if not os.path.exists(save_dir):
    raise ValueError(f"Não foi possível criar o diretório {save_dir}")

# Capturar imagens do tabuleiro de xadrez usando a câmera interna
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise ValueError("Não foi possível acessar a câmera")

print("Pressione 's' para capturar a imagem e 'q' para sair.")
image_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Falha ao capturar a imagem da câmera.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        cv2.drawChessboardCorners(frame, chessboard_size, corners, ret)

    cv2.imshow('Calibration', frame)
    
    key = cv2.waitKey(1)
    if key == ord('s') and ret:
        image_count += 1
        image_filename = os.path.join(save_dir, f'calibration_image_{image_count}.jpg')
        success = cv2.imwrite(image_filename, frame)
        if success:
            print(f"Imagem capturada e salva em {image_filename}")
            object_points.append(objp)
            image_points.append(corners)
        else:
            print(f"Falha ao salvar a imagem em {image_filename}")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if len(object_points) < 10:
    raise ValueError("Não foram capturadas imagens suficientes para calibração. Capture pelo menos 10 imagens.")

# Calibrar a câmera
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None)

# Salvar os parâmetros em um arquivo .npz
np.savez('calibration_params.npz', K=K, dist=dist)

print("Calibração concluída e parâmetros salvos.")
print("Matriz de calibração K:\n", K)
print("Distorção:", dist.ravel())

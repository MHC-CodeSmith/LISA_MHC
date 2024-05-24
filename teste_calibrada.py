import cv2
import numpy as np

# Carregar os parâmetros de calibração a partir do arquivo salvo
with np.load('calibration_params.npz') as data:
    K = data['K']
    dist = data['dist']

# Iniciar captura de vídeo da câmera interna
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise ValueError("Não foi possível acessar a câmera")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Corrigir a distorção
    h, w = frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    undistorted_frame = cv2.undistort(frame, K, dist, None, new_camera_matrix)

    # Recortar a imagem corrigida (opcional)
    x, y, w, h = roi
    undistorted_frame = undistorted_frame[y:y+h, x:x+w]

    # Mostrar a imagem original e a imagem corrigida
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Undistorted Frame', undistorted_frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

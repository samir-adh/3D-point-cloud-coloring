import numpy as np
from calibration_points import points
from PIL import Image
import matplotlib.pyplot as plt

def calibrationMatrix(calibration_points) -> np.ndarray:
    # Nombre de points de calibration
    N: int = 11

    # Matrice de taille 3 x N contenant les coordonées des points de calibrage sur l'image
    A: np.ndarray = np.array([ [p[1][0],p[1][1],1] for p in calibration_points]).T

    # Matrice de taille 4 x N contenant les coordonées des points de calibrage dans le monde réel
    B: np.ndarray = np.array([ [p[0][0],p[0][1],p[0][2],1] for p in calibration_points]).T

    # Matrice représentant le système d'équations linéaires
    S: np.ndarray = np.block(
        [
        [ np.array(
                [
                    np.block([B[:, i].T, np.zeros_like(B[:, i].T), -A[0, i] * B[:, i].T]),
                    np.block([np.zeros_like(B[:, i].T), B[:, i].T, -A[1, i] * B[:, i].T]),
                ]
            )]
            for i in range(N)
        ]
    )

    # Decomposition SVD
    _,_,Vt = np.linalg.svd(S)
    V = Vt.T
    m = V[:,-1]

    M = np.array([
        m[4*i: 4*(i+1)] for i in range(3)
    ])

    return M

if __name__ == '__main__':
    plt.figure()
    estimations = []
    for k in range(len(points)):
        calibration_points = points[0:k] + points[k+1:]
        pw = np.array(list(points[k][0]) + [1]).T
        M = calibrationMatrix(calibration_points)
        iw = np.matmul(M,pw)
        iw = iw[:2]/iw[2]
        estimations.append(iw)
    img = np.array(Image.open('./calib.jpg'))
    for p in estimations:
        i = int(p[1])
        j = int(p[0])
        for k in range(-10,11):

            img[i+k,j] = [255,0,0]
            img[i,j+k] = [255,0,0]
            img[i+k,j+k] = [255,0,0]
    # plt.imsave('result.png', img)
    plt.imshow(img)
    plt.show()

    

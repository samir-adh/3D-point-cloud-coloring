import numpy as np
from calibration_points import points, validation

# Nombre de points de calibration
N: int = 11

# Matrice de taille 3 x N contenant les coordonées des points de calibrage sur l'image
A: np.ndarray = np.array([ [p[1][0],p[1][1],1] for p in points]).T

# Matrice de taille 4 x N contenant les coordonées des points de calibrage dans le monde réel
B: np.ndarray = np.array([ [p[0][0],p[0][1],p[0][2],1] for p in points]).T

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

if __name__ == '__main__':
    pw = np.array(list(validation[0]) + [1]).T
    print(pw)
    iw = np.matmul(M,pw)
    iw = iw[:2]/iw[2]
    print(iw)

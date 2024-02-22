import numpy as np

# Nombre de points de calibration
N: int = 11

# Matrice de taille 3 x N contenant les coordonées des points de calibrage sur l'image
A: np.ndarray = np.zeros((3, N))

# Matrice de taille 4 x N contenant les coordonées des points de calibrage dans le monde réel
B: np.ndarray = np.zeros((4, N))

# Matrice représentant le système d'équations linéaires
S: np.ndarray = np.array(
    [
        np.array(
            [
                [B[:, i].T, np.zeros_like(B[:, i]).T, -A[0, i] * B[:, i]],
                [np.zeros_like(B[:, i]).T, B[:, i].T, -A[1, i] * B[:, i]],
            ]
        )
        for i in range(N)
    ]
)

# Decomposition SVD
_,_,Vt = np.linalg.svd(S)
V = Vt.T
m = V[:,-1]


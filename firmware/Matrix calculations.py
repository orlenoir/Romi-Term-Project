# -*- coding: utf-8 -*-
"""
Created on Tue Feb 24 10:28:18 2026

@author: orlen
"""

import numpy as np
from scipy.linalg import expm

Ts = 0.020  # seconds

# ---- FILL THESE IN (must be 2-D lists) ----
A = np.array([
  [0,0,0,0],
  [0,0,0,0],
  [0,0,0,1],
  [0,0,0,0]
], dtype=float)
B = np.zeros((4,2))

C = np.eye(4)

l_s = 20.0
l_psi = 15.0
l_psidot = 10.0

L = np.diag([l_s, l_s, l_psi, l_psidot])

def as_2d(M, name):
    M = np.array(M, dtype=float)
    if M.ndim == 0:
        raise ValueError(f"{name} is a scalar; expected a 2-D matrix")
    if M.ndim == 1:
        # promote 1-D vector to column vector by default
        M = M.reshape((-1, 1))
    if M.ndim != 2:
        raise ValueError(f"{name} has ndim={M.ndim}; expected 2")
    return M

A = as_2d(A, "A")
B = as_2d(B, "B")
C = as_2d(C, "C")
L = as_2d(L, "L")

n = A.shape[0]
if A.shape[1] != n:
    raise ValueError(f"A must be square. Got {A.shape}")

# Check compatibility for observer
# A: (n,n)
# B: (n,m)
# C: (p,n)
# L: (n,p)
m = B.shape[1]
p = C.shape[0]

if B.shape[0] != n:
    raise ValueError(f"B rows must match n. B={B.shape}, n={n}")
if C.shape[1] != n:
    raise ValueError(f"C cols must match n. C={C.shape}, n={n}")
if L.shape != (n, p):
    raise ValueError(f"L must be (n,p). L={L.shape}, expected {(n,p)}")

Ao = A - L @ C
print("A", np.array(A).ndim, np.array(A).shape)
print("B", np.array(B).ndim, np.array(B).shape)
print("C", np.array(C).ndim, np.array(C).shape)
print("L", np.array(L).ndim, np.array(L).shape)
Btilde = np.hstack((B, L))   # (n, m+p)

mp = Btilde.shape[1]

# Block discretization
M = np.zeros((n + mp, n + mp))
M[:n, :n] = Ao
M[:n, n:] = Btilde

Md = expm(M * Ts)
Ad = Md[:n, :n]
Bd = Md[:n, n:]

np.set_printoptions(precision=8, suppress=True)
print("Shapes:")
print("A", A.shape, "B", B.shape, "C", C.shape, "L", L.shape)
print("Ad", Ad.shape, "Bd", Bd.shape)

print("\nPaste-ready Ad =")
print(Ad.tolist())
print("\nPaste-ready Bd =")
print(Bd.tolist())
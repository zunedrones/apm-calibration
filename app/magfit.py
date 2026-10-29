"""Ajuste de esfera para calibração "live" da bússola (como o Mission Planner faz no 3.2).

No ArduCopter 3.2 o campo enviado em RAW_IMU já tem os offsets somados:
    field = raw + COMPASS_OFS
Então recuperamos raw = field - COMPASS_OFS, ajustamos uma esfera |raw - c| = r e o
novo offset é -c (assim field_novo = raw - c fica centrado na origem).
"""
from __future__ import annotations

import numpy as np

N_BANDS, N_SECTORS = 4, 8  # 32 células de mesma área na esfera unitária


def fit_sphere(points: np.ndarray) -> tuple[np.ndarray, float, float]:
    """Mínimos quadrados lineares: |p|^2 = 2 p·c + (r^2 - |c|^2).

    points: (N, 3) float64, mGauss.  Retorna (centro (3,), raio, rms do resíduo radial).
    """
    p = np.asarray(points, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 10:
        raise ValueError("precisa de pelo menos 10 amostras (N, 3)")
    A = np.hstack([2.0 * p, np.ones((len(p), 1))])          # (N, 4)
    b = np.sum(p * p, axis=1)                                 # (N,)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = x[:3]
    r2 = x[3] + c @ c
    if r2 <= 0:
        raise ValueError("ajuste degenerado (amostras insuficientes ou colineares)")
    r = float(np.sqrt(r2))
    resid = np.linalg.norm(p - c, axis=1) - r
    return c, r, float(np.sqrt(np.mean(resid**2)))


def coverage(points: np.ndarray, center: np.ndarray) -> tuple[float, list[int]]:
    """Fração das 32 células de mesma área visitadas pelas direções (p - c)/|p - c|."""
    d = np.asarray(points, dtype=np.float64) - center
    n = np.linalg.norm(d, axis=1)
    d = d[n > 1e-9] / n[n > 1e-9, None]
    if len(d) == 0:
        return 0.0, [0] * (N_BANDS * N_SECTORS)
    # faixas uniformes em z têm a mesma área (teorema de Arquimedes)
    band = np.clip(((d[:, 2] + 1) / 2 * N_BANDS).astype(int), 0, N_BANDS - 1)
    sector = np.clip(((np.arctan2(d[:, 1], d[:, 0]) + np.pi) / (2 * np.pi) * N_SECTORS).astype(int), 0, N_SECTORS - 1)
    counts = np.bincount(band * N_SECTORS + sector, minlength=N_BANDS * N_SECTORS)
    return float(np.count_nonzero(counts)) / counts.size, counts.tolist()


def evaluate(points: np.ndarray) -> dict:
    """Ajuste + métricas de qualidade para a interface."""
    c, r, rms = fit_sphere(points)
    cov, cells = coverage(points, c)
    offsets = -c
    worst = float(np.max(np.abs(offsets)))
    if cov < 0.75:
        quality, msg = "ruim", "Cobertura insuficiente: gire o drone em todas as direções."
    elif rms / r > 0.08:
        quality, msg = "ruim", "Muito ruído/distorção: afaste de metais, fios de força e ímãs."
    elif worst > 400:
        quality, msg = "ruim", "Offsets muito grandes: a bússola está perto de interferência magnética."
    elif worst > 150 or rms / r > 0.04:
        quality, msg = "aceitável", "Funciona, mas o ideal é offsets < 150 e menos ruído."
    else:
        quality, msg = "boa", "Calibração boa."
    return {
        "offsets": [round(float(v), 1) for v in offsets],
        "radius": round(r, 1),
        "rms": round(rms, 2),
        "rms_pct": round(100 * rms / r, 2),
        "coverage": round(cov, 3),
        "cells": cells,
        "n": int(len(points)),
        "quality": quality,
        "message": msg,
    }

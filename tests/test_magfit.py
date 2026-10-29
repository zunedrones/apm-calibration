"""Verificação numérica do ajuste de esfera da bússola."""
import numpy as np
import pytest

from app import magfit


def esfera(n: int, centro, raio: float, ruido: float, seed: int = 0, hemisferio: bool = False) -> np.ndarray:
    rng = np.random.default_rng(seed)
    d = rng.normal(size=(n, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    if hemisferio:
        d[:, 2] = np.abs(d[:, 2])
    return np.asarray(centro) + raio * d + rng.normal(scale=ruido, size=(n, 3))   # (n, 3) float64


def test_recupera_centro_e_raio():
    c = np.array([120.0, -80.0, 45.0])
    pts = esfera(600, c, 250.0, 2.0)
    centro, raio, rms = magfit.fit_sphere(pts)
    assert np.allclose(centro, c, atol=1.0)
    assert raio == pytest.approx(250.0, abs=1.0)
    assert rms == pytest.approx(2.0 * np.sqrt(1), rel=0.3)  # ruído radial ~ sigma


def test_offset_e_menos_o_centro_e_centraliza_o_campo():
    """Modelo do 3.2: field = raw + COMPASS_OFS. Com COMPASS_OFS = -c o campo fica centrado."""
    c = np.array([-60.0, 200.0, -30.0])
    raw = esfera(500, c, 300.0, 1.0, seed=1)
    ofs_atual = np.array([10.0, -5.0, 3.0])
    field = raw + ofs_atual                      # o que chega em RAW_IMU
    res = magfit.evaluate(field - ofs_atual)     # o que CompassCal acumula
    novo = np.array(res["offsets"])
    assert np.allclose(novo, -c, atol=1.0)
    raios = np.linalg.norm(raw + novo, axis=1)    # campo corrigido: esfera centrada na origem
    assert raios.std() < 3 and raios.mean() == pytest.approx(300.0, abs=1.0)
    assert res["coverage"] == 1.0
    assert res["quality"] == "aceitável"         # |ofs| = 200 > 150


def test_cobertura_parcial_e_ruim():
    pts = esfera(400, [0, 0, 0], 250.0, 1.0, seed=2, hemisferio=True)
    res = magfit.evaluate(pts)
    assert res["coverage"] <= 0.6                # ~metade das 32 células
    assert res["quality"] == "ruim"


def test_boa():
    res = magfit.evaluate(esfera(600, [20, -15, 40], 250.0, 2.0, seed=3))
    assert res["quality"] == "boa"


def test_poucas_amostras():
    with pytest.raises(ValueError):
        magfit.fit_sphere(np.zeros((5, 3)))

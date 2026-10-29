"""Calibração de 6 posições do acelerômetro (ArduCopter 3.2 / APM 2.x) via MAVLink.

Uso (interativo):
  docker run --rm -it --device /dev/ttyACM0 -v "$PWD":/work -w /work zune/apm-calibration python scripts/calib_accel.py
"""
import math
import statistics as st
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
PORT, BAUD = "/dev/ttyACM0", 115200

# ordem do firmware 3.2: level, LEFT, RIGHT, nose DOWN, nose UP, BACK
POSICOES = [
    ("LEFT", "deitada sobre o lado ESQUERDO (lado esquerdo encostado na mesa)"),
    ("RIGHT", "deitada sobre o lado DIREITO (lado direito encostado na mesa)"),
    ("DOWN", "em pé com o NARIZ PARA BAIXO (seta da placa apontando para a mesa)"),
    ("UP", "em pé com o NARIZ PARA CIMA (seta da placa apontando para o teto)"),
    ("BACK", "de CABEÇA PARA BAIXO (face de cima encostada na mesa)"),
    ("level", "NIVELADA, na posição normal de voo, sobre uma superfície plana"),
]


def traduz(texto: str) -> str | None:
    if "press any key" not in texto:
        return None
    for chave, pt in POSICOES:
        if chave in texto:
            return pt
    return texto


def ler_params(m, nomes: list[str]) -> dict[str, float]:
    out = {}
    for n in nomes:
        m.mav.param_request_read_send(m.target_system, m.target_component, n.encode(), -1)
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        while p is not None and p.param_id != n:
            p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        out[n] = p.param_value if p else float("nan")
    return out


def norma_gravidade(m, segundos: float = 6.0) -> tuple[float, float]:
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   M.MAV_DATA_STREAM_RAW_SENSORS, 4, 1)
    amostras = []
    t0 = time.time()
    while time.time() - t0 < segundos:
        msg = m.recv_match(type="RAW_IMU", blocking=True, timeout=1)
        if msg:
            amostras.append(math.sqrt(msg.xacc**2 + msg.yacc**2 + msg.zacc**2))
    return st.mean(amostras), st.pstdev(amostras)


def main() -> int:
    print("Conectando... deixe a placa PARADA (ela recalibra o giroscópio ao ligar).")
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    hb = m.wait_heartbeat(timeout=15)
    if not hb:
        print("ERRO: sem heartbeat."); return 1
    if hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
        print("ERRO: a placa está ARMADA. Desarme antes."); return 1
    # a abertura da serial reseta o ATmega2560; espera terminar o boot
    t0 = time.time()
    while time.time() - t0 < 8:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if msg:
            print(f"  [placa] {msg.text}")
            if "GROUND START" in msg.text:
                break
    time.sleep(1)

    antes = ler_params(m, [f"INS_ACC{t}_{a}" for t in ("OFFS", "SCAL") for a in "XYZ"])
    print("Antes:", {k: round(v, 4) for k, v in antes.items()})

    print("\nIniciando calibração do acelerômetro (6 posições).")
    m.mav.command_long_send(m.target_system, m.target_component,
                            M.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 1, 0, 0)
    passo = 0
    resultado = None
    t_ultimo = time.time()
    while resultado is None:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if msg is None:
            if time.time() - t_ultimo > 60:
                print("ERRO: a placa parou de responder."); return 1
            continue
        t_ultimo = time.time()
        texto = msg.text
        pt = traduz(texto)
        if pt:
            passo += 1
            print(f"\n=== Posição {passo}/6: coloque a placa {pt}")
            print(f"    (firmware: {texto})")
            input("    Posicione, solte a placa, e aperte ENTER... ")
            m.mav.command_ack_send(0, 1)
            print("    medindo, não mexa...")
            t_ultimo = time.time()
        else:
            print(f"  [placa] {texto}")
            if "successful" in texto.lower():
                resultado = True
            elif "failed" in texto.lower():
                resultado = False

    if not resultado:
        print("\nCalibração FALHOU. Nada foi alterado. Rode de novo segurando cada posição bem parada.")
        return 1

    time.sleep(2)
    depois = ler_params(m, list(antes))
    print("\nDepois:", {k: round(v, 4) for k, v in depois.items()})
    input("\nVerificação: coloque a placa NIVELADA e aperte ENTER... ")
    media, desvio = norma_gravidade(m)
    erro = 100 * (media - 1000) / 1000
    print(f"|a| = {media:.1f} ± {desvio:.1f} mG  (erro {erro:+.1f}% em relação a 1 g; antes era -5,6%)")
    print("OK" if abs(erro) < 2 else "ATENÇÃO: ainda longe de 1 g, considere repetir.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

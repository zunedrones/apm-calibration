# ArduCopter APM 2.x — diagnóstico

> Registro do dia em que a placa foi diagnosticada. Os scripts citados estão hoje em `scripts/`
> e a imagem Docker se chama `zune/apm-calibration` (veja o README principal).

Placa: APM 2.x (ATmega2560, USB `2341:0010`), `/dev/ttyACM0` @ 115200.
Firmware: ArduCopter 3.2.x (`SYSID_SW_MREV=120`), frame QUAD.

## Rodar o diagnóstico (Docker, sem instalar nada no host)

```bash
docker build -t apm-tools .
docker run --rm --device /dev/ttyACM0 -v "$PWD":/work apm-tools python /work/probe.py   # heartbeat, sensores, params -> params.json
docker run --rm --device /dev/ttyACM0 -v "$PWD":/work apm-tools python /work/probe2.py  # ruído da IMU, saúde dos sensores
docker run --rm -it --device /dev/ttyACM0 apm-tools mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
```

## Backup

`backup/params-2026-10-29.{json,param}` — parâmetros originais (306), antes de qualquer mudança.
O `.param` pode ser carregado no APM Planner / Mission Planner para restaurar.

## Resultado em 2026-10-29

- OK: boot, MAVLink, MPU6000 (acc/gyro), barômetro MS5611, Vcc 5.10 V, I2C sem erros, saídas de motor.
- Acelerômetro nunca calibrado (`INS_ACCOFFS=0`, `INS_ACCSCAL=1`): |a| = 943.7 mG (−5.6%).
  Provável causa do `PreArm: Alt disparity`.
- Bússola desligada (`MAG_ENABLE=0`), configurada como externa (`COMPASS_EXTERNAL=1`, `COMPASS_ORIENT=8`).
- Sem GPS nem receptor RC conectados (canais em valores padrão, CH3=900).

## Calibração do acelerômetro — 2026-10-29

`calib_accel.py` (6 posições). 1ª tentativa falhou, 2ª ok:
offsets 0.217 / 0.169 / 0.505 m/s², escalas 0.9997 / 1.0040 / 1.0035.
|a| nivelada = 999.7 ± 4.5 mG (antes 943.7). `PreArm: Alt disparity` sumiu.
Backup: `backup/params-2026-10-29-pos-accelcal.{json,param}`.
PreArm restante: `Compass not healthy` (sem bússola, `MAG_ENABLE=0`, `ARMING_CHECK=1`).
`prearm.py [nome]` escuta PreArm por 45 s e, com nome, salva backup em `backup/`.

## Voo sem bússola — 2026-10-29

`set_params.py COMPASS_USE=0 ARMING_CHECK=-5`. No 3.2 o `ARMING_CHECK` é int8
(250 satura em 127); `-5` = 0xFB = todas as checagens menos bússola (0x04).
Depois disso: nenhum PreArm pendente. Backup: `backup/params-2026-10-29-sem-bussola.{json,param}`.
Só modos sem GPS: STABILIZE, ACRO, ALT_HOLD. Para voltar a usar bússola: `COMPASS_USE=1 MAG_ENABLE=1 ARMING_CHECK=1`.

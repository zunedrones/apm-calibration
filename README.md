<p align="center"><img src="app/static/img/zune-logo.png" width="120" alt="Zune Drones"></p>

# apm-calibration

Interface web para **diagnosticar, calibrar e deixar pronta para voo** uma controladora ArduCopter:

- **APM 2.x** (ArduPilot Mega 2.5/2.6) com **ArduCopter 3.2.1**, o último firmware que roda nessa placa;
- **Pixhawk** (FMUv2/v3 e outras placas ChibiOS) com **ArduCopter 4.x** (testado no 4.5.7).

A interface reconhece a família na conexão (AUTOPILOT_VERSION / MAVLink 2) e adapta cada etapa.
Tudo roda em Docker; a interface conversa com a placa por MAVLink pela USB.

Equipe Zune Drones · UFRN.

## Rodar

```bash
docker compose up -d --build
```

Abra <http://localhost:8080>, escolha a porta (`ttyACM0 · APM 2.x (Mega 2560)` ou `ttyACM0 · Pixhawk`) e clique em **Conectar**.

- Abrir a USB **reinicia a placa** (DTR do ATmega2560). Deixe-a parada por ~5 s enquanto o giroscópio calibra.
- Só um programa pode usar a serial: clique em **Desconectar** antes de abrir o APM Planner ou o Mission Planner.
- O container usa `privileged` + `/dev` montado, então a placa pode ser plugada depois de subir.

## Etapas da interface

| # | Etapa | O que faz |
|---|---|---|
| 1 | Diagnóstico | Firmware, Vcc, I2C, saúde dos sensores (SYS_STATUS), IMU e horizonte artificial ao vivo, PreArm, log da placa, recalibrar giro/baro |
| 2 | GPS ou sem GPS | Aplica o preset: **com GPS** (`MAG_ENABLE=1`, `COMPASS_USE=1`, `ARMING_CHECK=1`, autodeclinação) ou **sem GPS** (`MAG_ENABLE=0`, `COMPASS_USE=0`, `ARMING_CHECK=-5`) |
| 3 | Acelerômetro | Assistente das 6 posições, com desenho de cada posição e checagem de \|a\| ≈ 1000 mG; nivelamento (AHRS_TRIM) |
| 4 | Bússola | Detecção, orientação (`COMPASS_ORIENT`) com rumo ao vivo, calibração *live* por ajuste de esfera com cobertura das 32 regiões |
| 5 | GPS | Fix, satélites, HDOP |
| 6 | Rádio | Barras dos 8 canais, detecção de receptor, conferência de direção dos sticks, calibração mín/centro/máx |
| 7 | Modos de voo | 6 posições do CH5 com destaque da posição atual; avisa modos que exigem GPS |
| 8 | Failsafe | `FS_THR_ENABLE`/`FS_THR_VALUE` com recomendação por configuração e roteiro de teste |
| 9 | Bateria | Power Module da APM 2.5/2.6, calibração do multiplicador de tensão, failsafe de bateria |
| 10 | ESCs | Roteiro da calibração "todos de uma vez" do 3.2 |
| 11 | Motores | Teste de motor (MAV_CMD_DO_MOTOR_TEST) com diagrama do frame X/+, ordem A–D e sentido de giro |
| 12 | Checklist pré-voo | Junta as checagens automáticas e as manuais e diz se está pronto para voar |
| 13 | Parâmetros | Backup `.param`, restauração com diff, busca e edição |

Cada etapa tem um indicador na barra lateral (verde/amarelo/vermelho). Confirmações manuais (ESCs, motores,
teste de failsafe, itens do checklist) ficam salvas no navegador.

## O que muda entre as placas

| Etapa | APM 2.x · ArduCopter 3.2 | Pixhawk · ArduCopter 4.x |
|---|---|---|
| Acelerômetro | placa trava; posições por STATUSTEXT + COMMAND_ACK | placa pede cada posição com `ACCELCAL_VEHICLE_POS` e o GCS confirma |
| Bússola | ajuste de esfera feito aqui (RAW_IMU) | calibração na placa (`DO_START_MAG_CAL`, progresso por bússola, gravar com `DO_ACCEPT_MAG_CAL`); lista de prioridade `COMPASS_PRIOn_ID` decodificada |
| GPS ou sem GPS | `MAG_ENABLE`, `ARMING_CHECK=-5` | `COMPASS_ENABLE/USE`, fontes do EKF3 (`EK3_SRC1_*`), `ARMING_CHECK=1` |
| PreArm | escuta o reenvio de 30 s | bit `PREARM_CHECK` do SYS_STATUS + `RUN_PREARM_CHECKS` |
| ESCs | todos de uma vez pelo throttle do rádio | `ESC_CALIBRATION=3` (automática) + chave de segurança; DShot dispensa |
| Bateria | pinos 13/12, `FS_BATT_*` | pinos 2/3, `BATT_AMP_PERVLT`, `BATT_LOW_*`, `BATT_FS_LOW_ACT` (exige reboot após `BATT_MONITOR`) |
| Motores | saídas 1–4 | MAIN OUT 1–4, exige a chave de segurança liberada |

O envio é sempre em MAVLink 1 (as duas placas entendem); a recepção usa o dialeto MAVLink 2.

## Particularidades da APM 2.x / ArduCopter 3.2 (aprendidas na prática)

- **`ARMING_CHECK` é int8.** Valores > 127 saturam. "Tudo menos bússola" é `-5` (0xFB), não 250.
- **`SR0_*` são gravados pela placa.** Cada `REQUEST_DATA_STREAM` vira o novo valor de `SR0_*` na EEPROM.
  Se algum programa pedir 0, a placa fica "muda" (só heartbeat) e o APM Planner não mostra atitude.
  A interface sempre pede 4 Hz (o valor de fábrica) e ignora `SR*_` ao restaurar backups.
- **Bússola só HMC5883L.** Módulos GPS genéricos com QMC5883L não são reconhecidos pelo 3.2.
  `MAG_ENABLE` só é lido no boot; depois de mudar, reinicie a placa.
- **Calibração da bússola é feita no GCS.** O 3.2 envia em `RAW_IMU` o campo já somado aos offsets
  (`field = raw + COMPASS_OFS`); a interface ajusta uma esfera em `raw` e grava `COMPASS_OFS = -centro`
  (verificado em `tests/test_magfit.py`).
- **Mensagens podem se perder na calibração do acelerômetro.** A placa fica bloqueada e descarta STATUSTEXT
  quando a serial enche. A interface infere a próxima posição pela ordem fixa e, no fim, confirma o resultado
  relendo `INS_ACC*` (testado em `tests/test_accelcal.py`).
- **Sem receptor, o CH3 fica em ~900 µs** (valor do encoder PPM da APM 2.x), abaixo de `FS_THR_VALUE`.
- **USB = 115200 baud**, rádio de telemetria = 57600.

## Particularidades da Pixhawk / ArduCopter 4.x

- **Bússola "com falha" com uma bússola ligada:** a placa lembra bússolas de montagens antigas em
  `COMPASS_PRIOn_ID`. Se a da prioridade 1 sumiu, o campo fica em 0 e a calibração é recusada
  ("Compass #0 … removed"). A página Bússola mostra a lista decodificada e reordena com um clique.
- **Chave de segurança:** sem liberá-la (LED vermelho fixo) não há teste de motor nem calibração de ESC.
- **`ARMING_CHECK=0`** é sinalizado como alerta: a placa arma sem conferir nada e não diz por que falha no PreArm.
- **Reiniciar** faz a USB sumir e voltar; a interface espera a porta e reconecta.

## Scripts de linha de comando (`scripts/`)

Usados no diagnóstico inicial; rodam sem a interface (pare a interface ou desconecte antes):

```bash
docker run --rm -it --device /dev/ttyACM0 -v "$PWD":/work -w /work zune/apm-calibration python scripts/probe.py
```

| Script | Função |
|---|---|
| `probe.py` | Heartbeat, sensores, telemetria e todos os parâmetros (→ `params.json`) |
| `probe2.py` | Ruído da IMU e saúde dos sensores decodificada |
| `prearm.py [nome]` | Escuta PreArm por 45 s; com nome, salva backup em `backup/` |
| `set_params.py N=V …` | Grava e confere parâmetros |
| `calib_accel.py` | Calibração do acelerômetro no terminal (interativo) |
| `listen.py` | Escuta passiva: taxa de cada mensagem |

## Testes

```bash
docker run --rm -v "$PWD":/opt/apm zune/apm-calibration python -m pytest -q tests
```

## Estrutura

```
app/
  main.py      API HTTP + WebSocket (FastAPI)
  vehicle.py   conexão MAVLink, detecção da placa, parâmetros e máquinas de estado (acelerômetro, bússola, rádio)
  magfit.py    ajuste de esfera e métricas da bússola (APM 2.x)
  ardu32.py    constantes do ArduCopter 3.2 (modos, frames, presets, ordem dos motores)
  ardu4.py     constantes do ArduCopter 4.x (Pixhawk)
  static/      interface (HTML/CSS/JS sem build, funciona offline)
backup/        parâmetros da placa da equipe (veja backup/README.md)
docs/          registro do diagnóstico de 2026-10-29
scripts/       ferramentas CLI
tests/         testes numéricos e da máquina de estados
```

## Segurança

Tire as hélices em **qualquer** etapa com bateria conectada (ESCs, motores, failsafe). O teste de motores
recusa o comando sem a confirmação e limita o throttle a 30 % e a duração a 5 s.

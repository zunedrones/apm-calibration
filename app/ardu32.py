"""Constantes do ArduCopter 3.2.x (último firmware suportado pela APM 2.x)."""

# custom_mode do HEARTBEAT
FLIGHT_MODES: dict[int, str] = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 10: "OF_LOITER", 11: "DRIFT", 13: "SPORT",
    14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
}
GPS_MODES = {3, 4, 5, 6, 7, 10, 11, 16}

# faixas de PWM do canal de modo (CH5) -> FLTMODE1..6
FLTMODE_PWM = [(0, 1230), (1231, 1360), (1361, 1490), (1491, 1620), (1621, 1749), (1750, 2200)]

# FRAME: só X e + têm diagrama de motores
FRAMES = {0: "PLUS", 1: "X", 2: "V", 3: "H", 4: "V-TAIL", 10: "Y6B"}

# ordem do teste de motores (A, B, C, D = sentido horário a partir da frente)
# -> (nome, posição, sentido de giro) para cada letra
MOTOR_TEST_LAYOUT = {
    1: [  # X
        ("A", "frente-direita", "anti-horário", 1),
        ("B", "trás-direita", "horário", 4),
        ("C", "trás-esquerda", "anti-horário", 2),
        ("D", "frente-esquerda", "horário", 3),
    ],
    0: [  # PLUS
        ("A", "frente", "horário", 3),
        ("B", "direita", "anti-horário", 1),
        ("C", "trás", "horário", 4),
        ("D", "esquerda", "anti-horário", 2),
    ],
}

# Bits do ARMING_CHECK no 3.2. O parâmetro é int8 com sinal: valores >127 saturam,
# então "todos menos X" é escrito como número negativo (ex.: -5 = 0xFB = sem bússola).
ARMING_CHECK_ALL = 1
ARMING_CHECK_BITS = {
    0x02: "barômetro", 0x04: "bússola", 0x08: "GPS", 0x10: "INS (acel./giro)",
    0x20: "parâmetros", 0x40: "rádio (RC)", 0x80: "tensão da placa",
}
ARMING_CHECK_NO_COMPASS = -5

# Parâmetros que a placa recalcula sozinha ou que são somente leitura:
# não faz sentido restaurá-los de um backup.
PARAMS_SKIP_RESTORE = {
    "SYSID_SW_MREV", "SYSID_SW_TYPE", "INS_PRODUCT_ID", "GND_ABS_PRESS", "GND_TEMP",
    "INS_GYROFFS_X", "INS_GYROFFS_Y", "INS_GYROFFS_Z", "CMD_TOTAL", "CMD_INDEX",
    "FENCE_TOTAL", "RALLY_TOTAL",
}
# Taxas de telemetria: a estação de solo ajusta sozinha; um backup com 0 deixaria a placa muda.
PARAMS_SKIP_RESTORE_PREFIX = ("SR0_", "SR1_", "SR2_")

# Presets de configuração conforme a escolha GPS / sem GPS
PRESETS: dict[str, dict[str, float]] = {
    "gps": {
        "MAG_ENABLE": 1, "COMPASS_USE": 1, "COMPASS_AUTODEC": 1,
        "ARMING_CHECK": ARMING_CHECK_ALL, "AHRS_GPS_USE": 1, "GPS_TYPE": 1,
    },
    "no_gps": {
        "MAG_ENABLE": 0, "COMPASS_USE": 0, "ARMING_CHECK": ARMING_CHECK_NO_COMPASS,
    },
}
# MAG_ENABLE só é lido no boot
PRESET_NEEDS_REBOOT = {"MAG_ENABLE"}

# Sensor de corrente/tensão "Power Module" da APM 2.5/2.6
APM2_POWER_MODULE = {"BATT_VOLT_PIN": 13, "BATT_CURR_PIN": 12, "BATT_VOLT_MULT": 10.1, "BATT_AMP_PERVOLT": 17.0}

# Taxa de telemetria padrão. O 3.2 grava em SR0_* a taxa pedida via REQUEST_DATA_STREAM,
# então sempre pedimos o valor original de fábrica para não deixar a placa "muda".
DEFAULT_STREAM_HZ = 4

"""Constantes do ArduCopter 4.x (Pixhawk e outras placas 32 bits com ChibiOS)."""
import re

from .ardu32 import FLTMODE_PWM, MOTOR_TEST_LAYOUT  # noqa: F401  (iguais no 4.x)

KEY = "ac4"
LABEL = "Pixhawk · ArduCopter 4.x"

FLIGHT_MODES: dict[int, str] = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE",
    16: "POSHOLD", 17: "BRAKE", 18: "THROW", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
    22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 27: "AUTO_RTL", 28: "TURTLE",
}
GPS_MODES = {3, 4, 5, 6, 7, 11, 16, 17, 18, 21, 23, 24, 27}

# FRAME_TYPE (com FRAME_CLASS=1, quad). A numeração dos motores do quad é a mesma do 3.2.
FRAMES = {0: "PLUS", 1: "X", 2: "V", 3: "H", 4: "V-TAIL", 5: "A-TAIL", 10: "Y6B", 12: "DJI X", 13: "CW X", 18: "BF X"}
FRAME_PARAM = "FRAME_TYPE"

ARMING_CHECK_ALL = 1
ARMING_CHECK_BITS = {
    0x02: "barômetro", 0x04: "bússola", 0x08: "GPS", 0x10: "INS (acel./giro)", 0x20: "parâmetros",
    0x40: "rádio (RC)", 0x80: "tensão da placa", 0x100: "bateria", 0x400: "log", 0x800: "chave de segurança",
    0x1000: "config. do GPS", 0x2000: "sistema",
}

PARAMS_SKIP_RESTORE = {"FORMAT_VERSION", "SYSID_SW_TYPE", "INS_GYROFFS_X", "INS_GYROFFS_Y", "INS_GYROFFS_Z",
                       "INS_GYR2OFFS_X", "INS_GYR2OFFS_Y", "INS_GYR2OFFS_Z", "INS_GYR3OFFS_X", "INS_GYR3OFFS_Y", "INS_GYR3OFFS_Z"}
PARAMS_SKIP_RESTORE_PREFIX = ("SR0_", "SR1_", "SR2_", "SR3_", "STAT_")
# IDs de hardware: são da placa, não da configuração
PARAMS_SKIP_RESTORE_RE = re.compile(r"(_DEV_ID\d*|_DEVID|INS_ACC\d?_ID|INS_GYR\d?_ID|_PRIO\d_ID)$")

PRESETS: dict[str, dict[str, float]] = {
    "gps": {
        "COMPASS_ENABLE": 1, "COMPASS_USE": 1, "COMPASS_AUTODEC": 1, "GPS_TYPE": 1, "ARMING_CHECK": ARMING_CHECK_ALL,
        "EK3_SRC1_POSXY": 3, "EK3_SRC1_VELXY": 3, "EK3_SRC1_YAW": 1,
    },
    "no_gps": {
        # sem bússola nem GPS: o EKF voa só com IMU e barômetro (STABILIZE/ALT_HOLD)
        "COMPASS_USE": 0, "COMPASS_USE2": 0, "COMPASS_USE3": 0, "ARMING_CHECK": ARMING_CHECK_ALL,
        "EK3_SRC1_POSXY": 0, "EK3_SRC1_VELXY": 0, "EK3_SRC1_YAW": 0,
    },
}
PRESET_NEEDS_REBOOT = {"COMPASS_ENABLE", "GPS_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_YAW"}

# Power Module analógico na porta POWER da Pixhawk 1 (ADC 2 = tensão, 3 = corrente)
POWER_MODULE = {"BATT_VOLT_PIN": 2, "BATT_CURR_PIN": 3, "BATT_VOLT_MULT": 10.1, "BATT_AMP_PERVLT": 17.0}

BATTERY = {
    "amp": "BATT_AMP_PERVLT", "fs_enable": "BATT_FS_LOW_ACT", "fs_volt": "BATT_LOW_VOLT", "fs_mah": "BATT_LOW_MAH",
    "fs_options": {0: "desligado", 1: "LAND", 2: "RTL", 3: "SmartRTL ou RTL", 4: "SmartRTL ou LAND"},
    "reboot_note": "Na Pixhawk os parâmetros BATT_* só aparecem depois de ligar o BATT_MONITOR e reiniciar.",
}
FS_THR_OPTIONS = {0: "desligado", 1: "RTL (volta para casa)", 2: "continua missão em AUTO", 3: "sempre LAND",
                  4: "SmartRTL ou RTL", 5: "SmartRTL ou LAND"}
ESC_PARAMS = ["ESC_CALIBRATION", "MOT_PWM_TYPE", "MOT_PWM_MIN", "MOT_PWM_MAX", "MOT_SPIN_ARM", "MOT_SPIN_MIN", "RC3_MIN", "RC3_MAX"]

DEFAULT_STREAM_HZ = 4

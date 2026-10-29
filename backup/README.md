# Backups de parâmetros — APM 2.x da equipe (ArduCopter 3.2.1)

| Arquivo | Estado |
|---|---|
| `params-2026-10-29` | Como a placa chegou: acelerômetro nunca calibrado, `ARMING_CHECK=1`, `COMPASS_USE=1`, `MAG_ENABLE=0` |
| `params-2026-10-29-pos-accelcal` | Depois da calibração do acelerômetro (offsets 0.217/0.169/0.505, escalas 0.9997/1.0040/1.0035) |
| `params-2026-10-29-sem-bussola` | Depois de configurar para voar sem GPS: `COMPASS_USE=0`, `ARMING_CHECK=-5` |

`.param` = formato Mission Planner/APM Planner (`NOME,VALOR`); `.json` = o mesmo em JSON.

**Atenção:** em `params-2026-10-29-sem-bussola` os `SR0_*` ficaram em 0 (efeito dos scripts da
época). Restaurar esses valores deixa a placa sem telemetria. A interface web ignora `SR*_` ao restaurar;
os valores certos são `SR0_*=4` e `SR0_PARAMS=10`.

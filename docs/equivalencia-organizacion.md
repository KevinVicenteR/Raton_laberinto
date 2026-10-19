# Equivalencia tras organizar por funcionalidad

Comparación local realizada el 18 de octubre de 2026, antes y después de mover los fuentes.
Se conservan los cambios previos de velocidad adaptativa y parámetros entrenados.

## Simulador

Biblioteca: `RATON_NATIVE_LIB=$PWD/build/libraton_navigation.dylib`; Python: `.venv/bin/python`.
Controlador `mapped`, pasillo de 192 mm, semilla de ruido 1. Se usa `entrenamiento.evaluation.run_job`.
Los casos de competencia usan semilla de mapa 0; los aleatorios, la indicada en la tabla.
Los límites originales de evaluación son 120 s (competencia 4), 300 s (alterada),
1200 s (modo 3) y 600 s (modo 5). No se publicaron ni regeneraron resultados.

Los valores de las cuatro columnas coinciden **exactamente**, incluidos los flotantes sin redondear:

| Pista | Modo | reached (s) | contacts | failure | elapsed (s) |
|---|---:|---:|---:|---|---:|
| competition | 4 | 35.60499999999976 | 0 | — | 35.60499999999976 |
| competition | 3 | 476.51999999963914 | 0 | giro incompleto | 573.479999999551 |
| altered | 4 | 45.110000000004625 | 0 | — | 45.110000000004625 |
| random 100 | 3 | 366.78999999973894 | 0 | giro incompleto | 474.3349999996411 |
| random 101 | 3 | 381.06999999972595 | 0 | giro incompleto | 514.6899999996044 |
| random 102 | 3 | 289.6199999998091 | 0 | giro incompleto | 514.4099999996047 |
| random 106 | 5 | 54.23000000000929 | 0 | — | 54.23000000000929 |

Los fallos de giro tras algunas llegadas ya existían y se conservan.
Además, los 44 métodos de Sim tienen el mismo AST tras ignorar únicamente rutas de importación.

## Firmware

Compilación de referencia y final: `.venv/bin/pio run -e raton`. También se compila `prueba_hardware`.
La caché global de PlatformIO no permite escritura en esta sesión; se usó una copia local temporal
con `PLATFORMIO_CORE_DIR=/tmp/raton-refactor/platformio`, con los mismos paquetes y compilador.

| Recurso raton | Antes | Después | Diferencia |
|---|---:|---:|---:|
| Flash | 424540 | 424652 | +112 bytes (0,026 %) |
| RAM | 103856 | 103848 | −8 bytes |

Secciones obtenidas con `arm-none-eabi-size -A` de la misma cadena:

| Sección | Antes | Después | Diferencia |
|---|---:|---:|---:|
| .text | 422556 | 422668 | +112 |
| .ARM.exidx | 8 | 8 | +0 |
| .data | 1976 | 1976 | +0 |
| .bss | 101880 | 101872 | -8 |
| .nvictable | 256 | 256 | +0 |
| .crash_data_ram | 256 | 256 | +0 |
| .heap | 156752 | 156760 | +8 |
| .stack | 1024 | 1024 | +0 |

El pequeño cambio de tamaño procede de la separación de unidades y funciones y de su enlace.
Se capturaron además los símbolos con `arm-none-eabi-nm -S --size-sort`. Los ELF y los listados
completos de esta sesión están en `/tmp/raton-refactor/`; no son artefactos versionados.
No se ha comprobado físicamente el robot: no se cargó nada a la placa.

## Integridad y pruebas

- Las 158 declaraciones originales de configuración conservan nombre, expresión y valor.
- Los 105 archivos existentes de `web/public/sim` y `simulator/results` mantienen su SHA-256.
- CTest: núcleo y controles portables, incluyendo las pruebas previas repartidas por función.
- Pytest: 73 pruebas; Vitest: 20 pruebas.
- Formato Python y web, lint, tipos, build web y compilación de ambos firmwares.
- Entrada final: `scripts/run.sh pruebas`, con el entorno temporal de PlatformIO indicado arriba.

`git mv` no pudo crear `.git/index.lock` por los permisos de la sesión: los archivos se movieron
en el sistema de archivos; Git puede detectar los renombrados al preparar posteriormente el cambio.
No se hicieron commits, push ni cargas a placa.

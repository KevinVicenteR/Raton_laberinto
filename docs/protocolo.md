# Protocolo de telemetría y órdenes

Bluetooth LE con el servicio UART de Nordic (`6E400001-…`), nombre `Raton`. También funciona por el
Monitor Serie a 115200 baudios.

## Tramas

Cada línea viaja como `$<seq>,<datos>*<CRC8>` (CRC-8 polinomio 0x07 sobre `<seq>,<datos>`, en
hexadecimal). El receptor cuenta las pérdidas con `seq` y descarta las corruptas. Una línea sin `$`
también se acepta (apps BLE sencillas o el Monitor Serie).

Las órdenes de la web llevan el mismo formato; la placa contesta `ACK <seq>` o `NAK <seq>`, y un
reintento con el mismo `seq` no se ejecuta dos veces. La web espera el ACK 800 ms y reintenta 3 veces.

## Órdenes (placa)

| Orden | Efecto |
|---|---|
| `s`, `s<1-6>` | Arrancar (en el modo n) |
| `x` | Parar al momento (también suelta, sin salto de línea) |
| `m<1-6>` | Elegir modo |
| `G<x>,<y>` | Celda meta |
| `v<mm/s>` `k` `j` `d` `p<0/1>` | Velocidad, KP_WALL, PID directo y modo de dirección |
| `w<mm>` `z<mm>` `r<mm>` `g<MCPS>` | Umbral lateral, punto muerto frontal, alcance y señal mínima de los láseres |
| `n<a>,<b>,<c>,<d>` | Pesos del modo 5 |
| `c<mm>` | Tamaño de celda a mano |
| `K<mm>` | Calibrar el ángulo de los laterales en un pasillo de ancho conocido |
| `T` | Prueba de ruedas (ratón levantado) |
| `R` | Reiniciar la placa |
| `LOAD_MAP` | Parar y descargar el mapa guardado para elegir meta en la web |
| `ERASE_MAP` | Parar y borrar el mapa persistente; esperar `MAPA_BORRADO` |
| `?` | Ver valores |

## Mensajes (placa → web)

- Estado (10 Hz): `<ESTADO> I<izq> F<frente> D<der> v<vel> r<rumbo> p<pasillo> x<x> y<y> a<ángulo> t<ciclo ms> q<crudas L,F,R> c<confianzas>`.
- `OBS …` (2 Hz) y `MOTION …` (al acabar cada maniobra): diagnóstico detallado.
- Importantes (cola prioritaria): `MODE n`, `START n`, `INICIO_MAPA`, `CELL x= y= w= d=`,
  `DECIDE: …`, `GEOM pasillo= celda=`, `MISION …`, `MAPA_NUEVO`, `MAPA COMPLETO celdas=`, `RUTA_INICIO`,
  `ROCE …`, `PEGADO …`, `CRUCE …`, `BUCLE …`, `META`, `REINICIO`, `CALIB …`, `PRUEBA …`.


## Aprender una pista y reutilizarla

`m6` selecciona **Aprender pista**; `s6` empieza sin necesitar una meta. Mide el pasillo con los
laterales, deriva la celda sumando los 12 mm de pared y explora hasta agotar las fronteras. Regresa a
la salida y recupera el rumbo inicial antes de escribir. Por compatibilidad, `G-1,-1` + `s3` también
aprende. El modo 6 vuelve a medir aunque hubiera otra geometría cargada; el grosor de pared y el ángulo
calibrado de los sensores siguen siendo parámetros físicos, no se identifican de forma independiente.

La salida se representa como `(20,20)` en una rejilla de 40×40. La pista puede tener otra forma y
medida que la competencia, pero no tamaño ilimitado: debe caber sin tocar el borde exterior de la
rejilla y con celda entre 140 y 400 mm. Una frontera inalcanzable o tocar el borde invalida el aprendizaje.

- `MAPA_NUEVO`: empieza un aprendizaje; limpiar el dibujo anterior.
- `MAPA COMPLETO celdas=n`: terminó de explorar; aún falta regresar y guardar.
- `MAPA_GUARDADO`: regreso y orientación correctos; escritura TDBStore terminada.
- `MAPA_CARGADO`: mapa y geometría validados y cargados para el modo 3 con meta.
- `MAPA_INCOMPLETO`, `MAPA_LIMITE: …`: no guardar ni anunciar una pista completa.
- `MAPA_ERROR_FLASH`: falló escritura/borrado; no equivale a `MAPA_GUARDADO`.
- `MAPA_LISTO`: terminó de encolar la descarga solicitada con `LOAD_MAP`.
- `MAPA_NO_DISPONIBLE`: no hay un mapa persistente íntegro.
- `MAPA_BORRADO`: borrado persistente terminado y mapa en RAM invalidado.

Después de apagar/reiniciar, colocar el robot **en la salida original con el mismo rumbo** y enviar
`LOAD_MAP` para recuperar el dibujo, elegir meta y enviar `G<x>,<y>` + `s3`. Las coordenadas corresponden al mapa aprendido, no a la rejilla del reglamento.
Si no existe un registro válido, se conserva el comportamiento de exploración del modo 3.
`ERASE_MAP` lleva ACK al aceptar la orden; la confirmación de borrado es `MAPA_BORRADO`.
El borrado se ejecuta con los motores detenidos, también si la orden llegó durante una misión.

Persistencia: TDBStore de mbed sobre FlashIAPBlockDevice, dos bancos en los últimos 32 KiB
(`0xF8000–0xFFFFF`), excluidos del firmware por `firmware/nano33ble_map.ld`.
El registro único contiene `TRK\x01`, geometría y `RMAP` v1, con CRC32 interno y externo.
RMAP usa dimensiones, paredes recíprocas, visto y visitas; las distancias y callejones se recalculan.
No necesita red. Está verificada la compilación y el formato portable; el corte de energía en la placa
no se ha ensayado en esta tarea porque no se ha escrito nada al hardware.

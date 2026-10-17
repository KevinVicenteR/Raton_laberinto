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
| `s`, `s<1-5>` | Arrancar (en el modo n) |
| `x` | Parar al momento (también suelta, sin salto de línea) |
| `m<1-5>` | Elegir modo |
| `G<x>,<y>` | Celda meta |
| `v<mm/s>` `k` `j` `d` `p<0/1>` | Velocidad, KP_WALL, PID directo y modo de dirección |
| `w<mm>` `z<mm>` `r<mm>` `g<MCPS>` | Umbral lateral, punto muerto frontal, alcance y señal mínima de los láseres |
| `n<a>,<b>,<c>,<d>` | Pesos del modo 5 |
| `c<mm>` | Tamaño de celda a mano |
| `K<mm>` | Calibrar el ángulo de los laterales en un pasillo de ancho conocido |
| `T` | Prueba de ruedas (ratón levantado) |
| `R` | Reiniciar la placa |
| `?` | Ver valores |

## Mensajes (placa → web)

- Estado (10 Hz): `<ESTADO> I<izq> F<frente> D<der> v<vel> r<rumbo> p<pasillo> x<x> y<y> a<ángulo> t<ciclo ms> q<crudas L,F,R> c<confianzas>`.
- `OBS …` (2 Hz) y `MOTION …` (al acabar cada maniobra): diagnóstico detallado.
- Importantes (cola prioritaria): `MODE n`, `START n`, `INICIO_MAPA`, `CELL x= y= w= d=`,
  `DECIDE: …`, `GEOM pasillo= celda=`, `MISION …`, `MAPA_NUEVO`, `MAPA COMPLETO celdas=`, `RUTA_INICIO`,
  `ROCE …`, `PEGADO …`, `CRUCE …`, `BUCLE …`, `META`, `REINICIO`, `CALIB …`, `PRUEBA …`.


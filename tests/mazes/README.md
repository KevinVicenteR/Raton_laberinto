# Laberintos de prueba

Archivos ASCII fijos, incluidos en Git para probar `loadMaze` desde disco y resolverlos en una copia limpia. El origen (0,0) está abajo a la izquierda.

- `clasico_01.txt`: 16×16, generado con `raton_sim --generate 7 --size 16 --loops 10`. Meta central de cuatro celdas.
- `impar_05.txt`: 5×5, generado con `raton_sim --generate 123 --size 5 --loops 0`. Meta central de una celda y callejones.

Son casos sintéticos del núcleo lógico; no representan la pista física de competencia. Las pruebas también verifican que ambos archivos existan, además de cargar y resolver cada `.txt` del directorio. No se generan durante los tests.

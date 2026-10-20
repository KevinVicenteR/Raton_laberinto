#!/bin/bash
# Bateria de pruebas del simulador en paralelo (4 procesos) con resumen.
#   scripts/banco.sh [N] [opciones de raton_sim]
# N = casos aleatorios con errores de nivel 2 (por defecto 80; mas el laberinto del reglamento).
# Ejemplos:
#   scripts/banco.sh 80                                   sensores a 40 mm (lo de los parametros)
#   SIM_H=25 scripts/banco.sh 80 --set sensorHeightMm=25  sensores bajos
#   SIM_MOTOR=0.8 scripts/banco.sh 80                     motores un 20 % mas lentos (bateria baja)
#   scripts/banco.sh 80 --fuera 450                       objetos fuera del laberinto a 45 cm
# Variables: SIMBIN (binario), SIM_LISTA=1 (ademas imprime cada caso).
N=${1:-80}
shift
DIR=$(cd "$(dirname "$0")/.." && pwd)
B=${SIMBIN:-$DIR/build/raton_sim}
D=$(mktemp -d)
Q=$((N / 4))
for i in 0 1 2 3; do
  a=$((i * Q)); b=$(((i + 1) * Q)); [ $i -eq 0 ] && a=-1
  "$B" --bench $b --desde $a --nivel 2 "$@" > "$D/$i.txt" 2>&1 &
done
wait
cat "$D"/*.txt | grep -E "^(OK|ROCE|FALLO)" > "$D/all.txt"
[ -n "$SIM_LISTA" ] && cat "$D/all.txt"
awk '{n++; if($1!="FALLO") ok++; if($1=="OK") cl++;
  for(i=1;i<=NF;i++){
    if($i ~ /^choques=/){split($i,a,"=");split(a[2],b,"(");c+=b[1]; cr+=b[2]+0}
    if($i ~ /^paredes_mal=/){split($i,a,"=");w+=a[2]}
    if($i ~ /^atascos=/){split($i,a,"=");st+=a[2]}
    if($i ~ /^zigzag=/){split($i,a,"=");zz+=a[2]}
    if($i ~ /^descentrado=/){split($i,a,"=");split(a[2],b,"/");dr+=b[1]; if(b[2]+0>dm) dm=b[2]+0}
    if($i ~ /^error_pose=/){split($i,a,"=");split(a[2],b,"/");l0+=b[1];l1+=b[2];l2+=b[3]}
    if($i == "meta1=si"){t=$(i+1); sub("s","",t); m1+=t; nm1++}
    if($i == "explora="){t=$(i+1); sub("s","",t); ex+=t; nex++}
    if($i == "carrera=si"){t=$(i+1); sub("s","",t); ca+=t; nca++}
  }}
  END {printf "casos %d cumplen %d limpios %d choques %d (carrera %d) atascos %d paredes_mal %d error_pose(avance/lateral/rumbo) %.1f/%.1f/%.2f zigzag %.2f descentrado %.1f max %d\n",
       n, ok, cl, c, cr, st, w, l0/n, l1/n, l2/n, zz/n, dr/n, dm;
       printf "tiempos medios: meta explorando %.1f s (%d), exploracion completa %.1f s, carrera %.1f s (%d)\n",
       m1/(nm1?nm1:1), nm1, ex/(nex?nex:1), ca/(nca?nca:1), nca}' "$D/all.txt"
grep FALLO "$D/all.txt" | cut -c1-60
rm -rf "$D"

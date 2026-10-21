# modos

App selecciona las páginas de Telemetría y Simulación y el tema visual.

Entradas: navegación de la interfaz, URL y preferencia local de tema.
Salidas: página visible y tema aplicado al documento.
Parámetros: page en la URL y raton-theme en localStorage.
Los estilos globales de la aplicación viven en styles.css.
Los seis modos físicos se despachan en firmware/modos/.
Pruebas: `cd web && npm run typecheck && npm run build`; navegación manual en la web.

## Resolver mapa

La pestaña **Resolver mapa** (`?page=solver`) calcula en el navegador la ruta mínima por celdas del mapa de competencia. Permite animar el avance, dar un paso, cambiar meta o posición y abrir/cerrar paredes interiores. Cada cambio recalcula desde la posición actual; si no existe ruta, se detiene. No envía comandos Bluetooth ni simula motores o sensores. Los tres modos del firmware permanecen iguales. Los cambios del mapa son temporales y se restauran al salir de esta pestaña.

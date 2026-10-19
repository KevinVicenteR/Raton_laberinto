# modos

App selecciona las páginas de Telemetría y Simulación y el tema visual.

Entradas: navegación de la interfaz, URL y preferencia local de tema.
Salidas: página visible y tema aplicado al documento.
Parámetros: page en la URL y raton-theme en localStorage.
Los estilos globales de la aplicación viven en styles.css.
Los seis modos físicos se despachan en firmware/modos/.
Pruebas: `cd web && npm run typecheck && npm run build`; navegación manual en la web.

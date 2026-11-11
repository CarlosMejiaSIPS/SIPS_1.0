# SIPS 1.0 — Sistema Inteligente de Producción de Silaje

**Proyecto de Trabajo Fin de Máster — Robótica y Automatización de Procesos**

SIPS 1.0 es un sistema automatizado de producción de silaje diseñado y simulado mediante CoppeliaSim, integrado con un PLC virtual CODESYS a través de comunicación OPC UA y un puente desarrollado en Python.

El proyecto contempla operaciones de transporte, inspección de calidad, clasificación de recipientes, llenado, compactación y supervisión del proceso.

## Archivos del proyecto

- `SIPS_COPPELIA_TFM_CMT.ttt`: escena de simulación en CoppeliaSim, con los scripts Lua incorporados.
- `SIPS_CODESYS_TFM_CMT.project`: proyecto de control PLC y visualización HMI en CODESYS.
- `SIPS_PUENTE_COPPELIA_CODESYS_CMT.py`: comunicación entre CoppeliaSim y CODESYS mediante Python y OPC UA.

## Software utilizado

- CoppeliaSim Edu.
- CODESYS V3.5 SP22 Patch 4.
- Python 3.12.5.
- Biblioteca `asyncua` 2.1.0.
- Cliente remoto ZMQ de CoppeliaSim.

## Ejecución general

1. Abrir el proyecto CODESYS y configurar el PLC virtual.
2. Configurar la publicación de variables OPC UA y las credenciales de acceso.
3. Abrir la escena SIPS en CoppeliaSim e iniciar la simulación.
4. Ejecutar el puente Python, introduciendo la contraseña del servidor OPC UA cuando se solicite.
5. Comprobar la comunicación y utilizar las interfaces HMI del sistema.

**Nota sobre credenciales:** las contraseñas y claves privadas no se incluyen en este repositorio. Cada usuario debe configurar sus propias credenciales y certificados OPC UA para ejecutar la integración completa.

## Videos demostrativos

Los videos del funcionamiento del sistema se incorporarán progresivamente a este repositorio.

## Observaciones

El proyecto corresponde a un prototipo de simulación académica. La ejecución integrada requiere disponer de las versiones compatibles y configurar correctamente la comunicación entre las aplicaciones.

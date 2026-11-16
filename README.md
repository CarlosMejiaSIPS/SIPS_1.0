# SIPS 1.0 — Sistema Inteligente de Producción de Silaje

**Proyecto de Trabajo Fin de Máster — Máster Universitario en Robótica y Automatización de Procesos**

## Descripción del proyecto

SIPS 1.0 es un sistema automatizado de producción de silaje, desarrollado como prototipo de simulación industrial bajo principios de la Industria 4.0 y Smart Factory.

El proyecto integra una celda robótica simulada en CoppeliaSim, un PLC virtual programado en CODESYS y un puente de comunicación desarrollado en Python mediante el protocolo OPC UA.

El sistema contempla operaciones de transporte de recipientes, inspección y clasificación de calidad, llenado, compactación, control secuencial, supervisión de estados y gestión de mantenimiento.

La integración permite estudiar el comportamiento coordinado de los equipos, las señales de control y las interfaces de supervisión dentro de un entorno virtual.

## Archivos principales del proyecto

Este repositorio contiene los siguientes archivos:

| Archivo | Descripción |
|---|---|
| `SIPS_COPPELIA_TFM_CMT.ttt` | Escena completa de simulación en CoppeliaSim, con robots, bandas transportadoras, equipos, sensores, lógica Lua e interfaces de supervisión. |
| `SIPS_CODESYS_TFM_CMT.project` | Proyecto de programación del PLC virtual en CODESYS, con variables de control, lógica secuencial y visualización HMI. |
| `SIPS_PUENTE_COPPELIA_CODESYS_CMT.py` | Programa Python encargado de intercambiar señales y estados entre CoppeliaSim y CODESYS mediante OPC UA y la API remota de CoppeliaSim. |

Los tres archivos constituyen los componentes principales para reproducir la simulación y estudiar la integración del sistema.

## Software y tecnologías utilizadas

- CoppeliaSim Edu.
- CODESYS V3.5 SP22 Patch 4.
- PLC virtual CODESYS Control Win x64.
- Python 3.12.5.
- Biblioteca `asyncua` 2.1.0.
- API remota ZMQ de CoppeliaSim.
- Protocolo industrial OPC UA.
- Programación Lua y Structured Text (ST).

## Procedimiento general de ejecución

1. Abrir el proyecto de CODESYS y configurar el PLC virtual.
2. Verificar la configuración de comunicación OPC UA, sus permisos y credenciales de acceso.
3. Abrir la escena del proyecto en CoppeliaSim.
4. Iniciar la simulación y ejecutar el puente de comunicación Python con la configuración correspondiente.
5. Comprobar la comunicación entre las aplicaciones y supervisar el funcionamiento mediante las interfaces HMI.

**Importante:** la reproducción integrada del proyecto requiere disponer del software compatible y configurar adecuadamente las conexiones, credenciales y certificados de seguridad.

## Credenciales de acceso y seguridad

Por motivos de seguridad informática, **el usuario y la contraseña utilizados para establecer la comunicación del PLC virtual CODESYS y el puente Python mediante OPC UA no se publican en este repositorio**.

Estas credenciales son necesarias para configurar correctamente el acceso y la comunicación entre los componentes del sistema.

El repositorio es de acceso público; por tanto, cualquier persona puede consultar o descargar los archivos disponibles. Publicar contraseñas en este espacio supondría una exposición innecesaria de información de acceso.

**Las personas autorizadas que necesiten reproducir la integración completa pueden solicitar las credenciales y las indicaciones de configuración directamente al autor del proyecto, mediante comunicación privada.**

La entrega de esta información se realizará de manera individual y por un canal seguro, evitando su difusión pública.

## Videos demostrativos del funcionamiento

El repositorio incluye seis videos demostrativos que presentan los principales módulos y funciones desarrollados en SIPS 1.0.

| Video | Contenido |
|---|---|
| `1_Introduccion.mp4` | Presentación general del sistema y de su entorno de simulación. |
| `2_Secuencia.mp4` | Funcionamiento del control secuencial y coordinación de las operaciones automatizadas. |
| `3_Mantenimiento.mp4` | Supervisión y funcionalidades del módulo de mantenimiento. |
| `4_Flexibilidad.mp4` | Adaptación del sistema a las configuraciones de producción de 25 kg y 50 kg. |
| `5_Calidad.mp4` | Inspección y clasificación de recipientes conformes y no conformes (OK/NOK). |
| `6_Gestion.mp4` | Presentación de las funcionalidades de gestión y supervisión del proyecto. |

**Los seis videos están incorporados en el directorio principal de este repositorio**, junto con los archivos de simulación, programación y comunicación.

Para consultarlos, se debe seleccionar el archivo correspondiente. Si GitHub no permite su reproducción directamente, puede descargarse mediante la opción **View raw**.

## Alcance y observaciones

SIPS 1.0 constituye un prototipo académico desarrollado y validado en un entorno de simulación. Su propósito es demostrar la integración de tecnologías de robótica, automatización, comunicación industrial y supervisión para un proceso automatizado de producción de silaje.

El proyecto no corresponde a una instalación industrial física en operación. Los resultados obtenidos pertenecen al entorno de simulación y a las pruebas de integración realizadas.

Los archivos técnicos y los videos demostrativos se encuentran disponibles para consulta y evaluación académica.

---

**Autor:** Carlos Iván Mejía Torres  
**Proyecto:** Trabajo Fin de Máster — Robótica y Automatización de Procesos  
**Universidad:** Universidad Internacional de Valencia (VIU)

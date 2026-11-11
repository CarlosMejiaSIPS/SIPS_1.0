# NUEVA VERSION: 2026-10-23, 8 nuevos BOOL de supervision, ACC. CERRADO existente.
# SIPS: PUENTE UNICO 6 BANDAS + ESTADO GENERAL (basado en variante funcional).
# Cinco variables OPC UA nuevas en GVL_CTRL: definidas en instrucciones adjuntas.
# VARIANTE COM. PLC + BANDA_ENTRADA: 1 indicador de estado REAL de orden de velocidad.
# Lee la senal SIPS_Banda_Entrada_Movimiento publicada desde Lua.
# El script Lua solo publica la senal tras escribir la velocidad: no cambia control.
# Requiere GVL_CTRL.SIPS_Banda_Entrada_Activa : BOOL := FALSE; publicado OPC UA.
# Esta variable es SOLO supervision, no manda velocidad a la banda.
# VARIANTE COM. PLC: latido OPC UA Python -> CODESYS cada 0.5 s.
# No cambia mandos de la HMI, logica PLC, ni los contadores existentes.
# VARIANTE DE SUPERVISION DE STOP Y DETENIDO - PRUEBA SIPS
# Requiere dos BOOL publicados en GVL_CTRL:
#   SIPS_STOP_EnCurso : BOOL := FALSE;
#   SIPS_Detenido : BOOL := FALSE;
# Las luces MARCHA / PAUSA y la secuencia existente quedan sin cambios.
# IMPORTANTE: DETENIDO supervisa evacuacion y Banda_Salida;
# no certifica robot HOME ni parada segura de la instalacion.

# SIPS / TFM — Puente de señales reales, fase de supervisión
# CoppeliaSim -> Python -> variables de entrada GVL_CTRL en CODESYS.
# CODESYS -> Python -> CoppeliaSim: permiso generador + cinco mandos por HMI.
# Usa la pequeña condición PLC añadida al Lua del Generador_Recipientes.
# HMI_Start refleja la autorizacion de banda de entrada: no usa el pulso
# SIPS_Autorizacion_START que puede consumir el control de acceso de Bill.
# Variante extendida: generador + 4 pulsadores + emergencia mantenida desde PLC.
# Las protecciones, la evacuacion STOP y la secuencia siguen en el HMI original.

from pathlib import Path
import time  # Supervisión de LIMP. BANDA: piloto de 3 segundos
import queue
import threading
import traceback
import tkinter as tk
from tkinter import simpledialog, ttk

from asyncua.sync import Client as OPCClient
from asyncua import ua
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

OPC_URL = 'opc.tcp://127.0.0.1:4840'
APP_URI = 'urn:PERSONAL:SIPS:PythonBridge'
CERT_DIR = Path.home() / 'SIPS_OPCUA'
PREFIJO_NODO = 'ns=4;s=|var|CODESYS Control Win V3 x64.Application.GVL_CTRL.'
INTERVALO = 0.5
SENAL_MODO = 'SIPS_PLC_Control_Activo'
SENAL_PERMISO = 'SIPS_PLC_Permiso_Generar'
SENAL_HEARTBEAT = 'SIPS_PLC_Heartbeat'
SENAL_CMD_START = 'SIPS_PLC_CMD_START'
SENAL_CMD_STOP = 'SIPS_PLC_CMD_STOP'
SENAL_CMD_PAUSE = 'SIPS_PLC_CMD_PAUSE'
SENAL_CMD_REANUDAR = 'SIPS_PLC_CMD_REANUDAR'
SENAL_CMD_EMERGENCIA = 'SIPS_PLC_CMD_EMERGENCIA'
SENAL_CMD_LIBERAR_EMERGENCIA = 'SIPS_PLC_CMD_LIBERAR_EMERGENCIA'

ENTRADAS = (
    ('HMI_Start', 'SIPS_START_BANDA_ENTRADA'),
    ('HMI_Stop', 'SIPS_CMD_STOP'),
    ('HMI_Pause', 'SIPS_CMD_PAUSE'),
    ('HMI_Emergencia', 'SIPS_CMD_EMERGENCIA'),
    ('Acceso_Solicitado', 'SIPS_Solicitud_Acceso'),
)

# Cinco bandas adicionales. Banda_Entrada conserva su lectura ya validada.
# (Variable BOOL de GVL_CTRL, senal INT publicada por el Lua de la banda).
BANDAS_ADICIONALES = (
    ('SIPS_Banda_Carga_Activa', 'SIPS_Banda_Carga_Movimiento'),
    ('SIPS_Banda_Rechazo_1_Activa', 'SIPS_Banda_Rechazo_1_Movimiento'),
    ('SIPS_Banda_Rechazo_2_Activa', 'SIPS_Banda_Rechazo_2_Movimiento'),
    ('SIPS_Banda_Alimentacion_Ensilaje_Activa', 'SIPS_Banda_Alimentacion_Ensilaje_Movimiento'),
    ('SIPS_Banda_Salida_Activa', 'SIPS_Banda_Salida_Movimiento'),
)

# Ocho nuevos pilotos de la visualización. ACC. CERRADO usa la
# variable Puerta_Cerrada, que ya existía en ENTRADAS/PLC.
# En este grupo NO hay mandos, únicamente estados de supervisión.
SENALES_PILOTOS = (
    ('SIPS_Compactador_Movimiento', 'SIPS_Compactador_Movimiento'),
    ('SIPS_Sellado_OK', 'SIPS_Sellado_OK'),
    ('SIPS_Dispensador_Tapas_Listo', 'SIPS_Dispensador_Tapas_Listo'),
    ('SIPS_Parada_Mant_Preventivo', 'SIPS_Parada_Mant_Preventivo'),
    ('SIPS_Parada_Mant_Predictivo', 'SIPS_Parada_Mant_Predictivo'),
    ('SIPS_Parada_Meta_Cumplida', 'SIPS_Parada_Meta_Cumplida'),
    ('SIPS_Robot_HOME', 'SIPS_Robot_HOME'),  # Solo supervision UR10 HOME.
    ('SIPS_Robot_Entrada', 'SIPS_Robot_Entrada'),  # Robot recoge recipiente en banda de entrada.
    ('SIPS_Robot_Carga', 'SIPS_Robot_Carga'),  # Solo supervision UR10 en CARGA.
    ('SIPS_Robot_Rechazo_1', 'SIPS_Robot_Rechazo_1'),  # Robot en rechazo de entrada.
    ('SIPS_Robot_Alimentacion', 'SIPS_Robot_Alimentacion'),  # Zona de salida de proceso.
    ('SIPS_Robot_Salida', 'SIPS_Robot_Salida'),  # Destino producto OK.
    ('SIPS_Robot_Rechazo_2', 'SIPS_Robot_Rechazo_2'),  # Destino producto NOK.
    ('SIPS_Robot_Toma_Tapa', 'SIPS_Robot_Toma_Tapa'),  # Robot en estacion de toma tapa.
    # Modos efectivos desde la HMI Lua: SOLO PILOTOS, no ordenes al PLC.
    ('SIPS_Modo_Control_Manual', 'SIPS_Modo_Control_Manual'),
    ('SIPS_Modo_Control_Automatico', 'SIPS_Modo_Control_Automatico'),
)
# Indicadores derivados adicionales (puerta abierta, limpieza y robot):
#    Puerta_Abierta             <- Sensor_Puerta_Abierta
#    SIPS_Limpieza_Banda_Activa <- Limpieza_ResiduosRetirados (pulso 3 s)


def puerta_abierta_instantanea(sim):
    """Solo indicador. NO participa en bloqueo de puerta ni rearme."""
    sensor = sim.getObject('/Sensor_Puerta_Abierta')
    puerta = sim.getObject('/Puerta_Abierta')
    formas = sim.getObjectsInTree(puerta, sim.object_shape_type, 0)
    if not formas:
        raise RuntimeError('Puerta_Abierta no contiene ninguna forma.')
    for forma in formas:
        # El control de acceso oculta la puerta inactiva con capa 0.
        # checkProximitySensor detecta figuras explicitas aunque esten ocultas:
        # ignorarlas aqui evita que ambas luces aparezcan activas.
        if sim.getObjectInt32Param(forma, sim.objintparam_visibility_layer) == 0:
            continue
        resultado = sim.checkProximitySensor(sensor, forma)
        detectada = resultado[0] if isinstance(resultado, (list, tuple)) else resultado
        if detectada > 0:
            return True
    return False


def puerta_cerrada_instantanea(sim):
    """Sensor de puerta de la escena. La confirmación de 2 s aún reside en Lua."""
    sensor = sim.getObject('/Sensor_Puerta_Cerrada')
    puerta = sim.getObject('/Puerta_Cerrada')
    formas = sim.getObjectsInTree(puerta, sim.object_shape_type, 0)
    if not formas:
        raise RuntimeError('Puerta_Cerrada no contiene ninguna forma.')
    for forma in formas:
        # El control de acceso oculta la puerta inactiva con capa 0.
        # checkProximitySensor detecta figuras explicitas aunque esten ocultas:
        # ignorarlas aqui evita que ambas luces aparezcan activas.
        if sim.getObjectInt32Param(forma, sim.objintparam_visibility_layer) == 0:
            continue
        resultado = sim.checkProximitySensor(sensor, forma)
        detectada = resultado[0] if isinstance(resultado, (list, tuple)) else resultado
        if detectada > 0:
            return True
    return False


def banda_entrada_orden_movimiento(sim, banda_handle):
    """Lee la señal de supervisión publicada por el script de Banda_Entrada.

    Se actualiza DESPUES de fijar la velocidad definitiva, ya con
    STOP/PAUSE/EMERGENCIA/sensor aplicados. No envía ninguna orden a la banda.
    """
    lectura = sim.getInt32Signal('SIPS_Banda_Entrada_Movimiento')
    if lectura is None:
        raise RuntimeError(
            'Falta SIPS_Banda_Entrada_Movimiento en CoppeliaSim. '
            'Agrega las dos líneas de publicación indicadas al script Lua '
            'de Banda_Entrada y reinicia la simulación.'
        )
    return int(lectura) == 1


def trabajar(clave, detener, mensajes):
    opc = None
    sim = None
    nodos = {}
    escritura_ok = False
    control_activado = False
    try:
        sim = RemoteAPIClient().require('sim')
        if sim.getSimulationState() == 0:
            raise RuntimeError('Inicia la simulación de CoppeliaSim antes del puente.')
        mensajes.put(('estado', 'CoppeliaSim conectado. Conectando OPC UA cifrado…'))
        opc = OPCClient(OPC_URL)
        opc.aio_obj.application_uri = APP_URI
        opc.set_security_string(
            'Basic256Sha256,SignAndEncrypt,'
            + str(CERT_DIR / 'client_cert.der') + ','
            + str(CERT_DIR / 'client_key.pem')
        )
        opc.set_user('admin')
        opc.set_password(clave)
        clave = None
        opc.connect()
        for nombre, _ in ENTRADAS:
            nodos[nombre] = opc.get_node(PREFIJO_NODO + nombre)
        nodos['Puerta_Cerrada'] = opc.get_node(PREFIJO_NODO + 'Puerta_Cerrada')
        nodo_permiso = opc.get_node(PREFIJO_NODO + 'PLC_Permiso_Generar')
        nodo_cmd_start = opc.get_node(PREFIJO_NODO + 'PLC_Orden_Start')
        nodo_cmd_stop = opc.get_node(PREFIJO_NODO + 'PLC_Orden_Stop')
        nodo_cmd_pause = opc.get_node(PREFIJO_NODO + 'PLC_Orden_Pause')
        nodo_cmd_reanudar = opc.get_node(PREFIJO_NODO + 'PLC_Orden_Reanudar')
        nodo_cmd_emergencia = opc.get_node(PREFIJO_NODO + 'PLC_Orden_Emergencia')
        # Señal nueva publicada en GVL_CTRL: solo supervision del enlace OPC UA.
        nodo_latido = opc.get_node(PREFIJO_NODO + 'SIPS_Heartbeat_Python')
        nodo_latido.read_value()  # Detecta antes de arrancar si el nodo falta.
        # Referencia conservada por compatibilidad; la supervisión ahora
        # recibe la señal publicada por el propio script Lua de Banda_Entrada.
        banda_entrada_handle = sim.getObjectHandle('Banda_Entrada')
        nodo_banda_entrada = opc.get_node(
            PREFIJO_NODO + 'SIPS_Banda_Entrada_Activa'
        )
        nodo_banda_entrada.read_value()  # Detecta si olvido publicar el simbolo.

        # Solo supervisión: comprobar las cinco nuevas variables OPC UA.
        nodos_bandas = {
            nombre: opc.get_node(PREFIJO_NODO + nombre)
            for nombre, _ in BANDAS_ADICIONALES
        }
        for nombre, nodo in nodos_bandas.items():
            nodo.read_value()  # Falla con nombre concreto si falta el simbolo.

        # NUEVAS LUCES: comprobar que las variables de GVL_CTRL existen
        # y aparecen en la configuración de símbolos OPC UA de CODESYS.
        nombres_pilotos = ([nombre for nombre, _ in SENALES_PILOTOS] +
                          ['Puerta_Abierta', 'SIPS_Limpieza_Banda_Activa',
                           'SIPS_Robot_Trabajando'])
        nodos_pilotos = {
            nombre: opc.get_node(PREFIJO_NODO + nombre)
            for nombre in nombres_pilotos
        }
        for nombre, nodo in nodos_pilotos.items():
            try:
                nodo.read_value()
            except Exception as exc:
                raise RuntimeError(
                    'Variable OPC UA nueva no disponible en GVL_CTRL: '
                    + nombre + '. Compile, descargue y publique simbolos. '
                    + str(exc)
                ) from exc

        # Variables previas de supervision de STOP, sin cambiar mandos ni PLC_PRG.
        nodo_stop_en_curso = opc.get_node(PREFIJO_NODO + 'SIPS_STOP_EnCurso')
        nodo_detenido = opc.get_node(PREFIJO_NODO + 'SIPS_Detenido')
        # Comprobar que CODESYS ha publicado ambos simbolos ANTES de comenzar.
        nodo_stop_en_curso.read_value()
        nodo_detenido.read_value()
        # Limpia SOLAMENTE pulsos pendientes. Nunca rearma la emergencia
        # de CoppeliaSim ni cambia el enclavamiento de emergencia del PLC.
        for senal in (SENAL_CMD_START, SENAL_CMD_STOP, SENAL_CMD_PAUSE,
                      SENAL_CMD_REANUDAR, SENAL_CMD_EMERGENCIA,
                      SENAL_CMD_LIBERAR_EMERGENCIA):
            sim.setInt32Signal(senal, 0)
        for nodo in (nodo_cmd_start, nodo_cmd_stop, nodo_cmd_pause,
                     nodo_cmd_reanudar):
            nodo.write_value(False, varianttype=ua.VariantType.Boolean)
        mensajes.put(('estado', 'CODESYS y CoppeliaSim conectados. Control PLC listo.'))

        anterior = None
        emergencia_anterior = False  # Primer TRUE: solicita activacion una sola vez.
        ciclo_iniciado = False  # Conserva estado aunque START_BANDA deje de autorizar.
        latido = False  # Alterna FALSE/TRUE en cada vuelta del puente.
        residuos_anteriores = None  # Inicializar sin crear un falso evento de limpieza.
        limpieza_visible_hasta = 0.0
        avisos_faltantes = set()  # No saturar la consola si falta un Lua nuevo.
        while not detener.is_set():
            if sim.getSimulationState() == 0:
                raise RuntimeError('La simulación de CoppeliaSim se ha detenido.')
            valores = {
                nombre: (sim.getInt32Signal(senal) or 0) == 1
                for nombre, senal in ENTRADAS
            }
            valores['Puerta_Cerrada'] = puerta_cerrada_instantanea(sim)

            # NUEVOS PILOTOS: SOLO LECTURAS DE CoppeliaSim.
            nuevos = {}
            lectura_home = None
            for nombre, senal in SENALES_PILOTOS:
                lectura = sim.getInt32Signal(senal)
                if nombre == 'SIPS_Robot_HOME':
                    lectura_home = lectura
                if lectura is None:
                    # No se rompe el enlace original si aún falta instalar
                    # la señal Lua en una estación. La luz queda apagada.
                    if senal not in avisos_faltantes:
                        print('[SIPS SUPERVISION] Falta señal Lua ' + senal
                              + ': luz en FALSE hasta corregir el script.', flush=True)
                        avisos_faltantes.add(senal)
                    nuevos[nombre] = False
                else:
                    nuevos[nombre] = int(lectura) == 1
                    avisos_faltantes.discard(senal)

            # Luz amarilla segun criterio SIPS: ROBOT TRABAJANDO / FUERA DE HOME.
            # Usa la MISMA lectura HOME que llega a su piloto; no modifica Lua
            # ni trayectorias. Cuando HOME es TRUE, se apaga. Cuando HOME es
            # FALSE, permanece encendida aun durante breves paradas del ciclo.
            # Si falta la lectura HOME, apagar por defecto (no inventar estado).
            nuevos['SIPS_Robot_Trabajando'] = (
                lectura_home is not None and int(lectura_home) == 0
            )

            # Sensor de puerta ABIERTA, completamente independiente de la
            # confirmación de PUERTA CERRADA utilizada por el control de acceso.
            try:
                nuevos['Puerta_Abierta'] = puerta_abierta_instantanea(sim)
                avisos_faltantes.discard('Puerta_Abierta')
            except Exception as exc:
                nuevos['Puerta_Abierta'] = False
                if 'Puerta_Abierta' not in avisos_faltantes:
                    print('[SIPS SUPERVISION] Sin lectura de puerta abierta: '
                          + str(exc), flush=True)
                    avisos_faltantes.add('Puerta_Abierta')

            # La aspiradora borra Pasto_Modelo inmediatamente: se supervisa
            # un evento de limpieza, NO una supuesta velocidad de motor.
            # Dejar verde 3 segundos al aumentar el contador original.
            lectura_residuos = sim.getInt32Signal('Limpieza_ResiduosRetirados')
            if lectura_residuos is None:
                if 'Limpieza_ResiduosRetirados' not in avisos_faltantes:
                    print('[SIPS SUPERVISION] Falta contador de limpieza: '
                          'Limpieza_ResiduosRetirados', flush=True)
                    avisos_faltantes.add('Limpieza_ResiduosRetirados')
                residuos_anteriores = None
                limpieza_visible_hasta = 0.0
            else:
                avisos_faltantes.discard('Limpieza_ResiduosRetirados')
                residuos_actuales = int(lectura_residuos)
                if residuos_anteriores is not None and residuos_actuales > residuos_anteriores:
                    limpieza_visible_hasta = time.monotonic() + 3.0
                residuos_anteriores = residuos_actuales
            nuevos['SIPS_Limpieza_Banda_Activa'] = (
                time.monotonic() < limpieza_visible_hasta
            )

            # Mandar solo estados de supervisión a GVL_CTRL. Nunca escribir
            # START, STOP o las salidas del PLC desde estas luces.
            for nombre, activo in nuevos.items():
                nodos_pilotos[nombre].write_value(
                    bool(activo), varianttype=ua.VariantType.Boolean
                )

            # SUPERVISION DE BANDA_ENTRADA: SOLO LECTURA, sin cambiar Lua.
            banda_entrada_activa = banda_entrada_orden_movimiento(
                sim, banda_entrada_handle
            )
            nodo_banda_entrada.write_value(
                bool(banda_entrada_activa), varianttype=ua.VariantType.Boolean
            )

            # Las otras cinco bandas publican 1/0 desde su funcion de velocidad.
            # No se escribe ninguna senal de mando en CoppeliaSim.
            estados_bandas = {'SIPS_Banda_Entrada_Activa': banda_entrada_activa}
            for nombre, senal in BANDAS_ADICIONALES:
                lectura = sim.getInt32Signal(senal)
                if lectura is None:
                    raise RuntimeError(
                        'No existe la señal ' + senal + ' en CoppeliaSim. '
                        'Comprueba el Lua de esa banda y reinicia la simulacion.'
                    )
                en_movimiento = int(lectura) == 1
                nodos_bandas[nombre].write_value(
                    en_movimiento, varianttype=ua.VariantType.Boolean
                )
                estados_bandas[nombre] = en_movimiento

            # SUPERVISION DE STOP - SOLO LECTURA DE SENALES REALES DE COPPELIA.
            # Replica la regla de evacuacion de la HMI y exige banda salida apagada.
            # No modifica ningun contador ni ninguna actuacion en CoppeliaSim.
            def leer_contador(nombre):
                return int(sim.getInt32Signal(nombre) or 0)

            generados = leer_contador('SIPS_Recipientes_Generados')
            retirados = leer_contador('SIPS_Recipientes_Retirados_Entrada')
            entrada_ok = leer_contador('SIPS_Entrada_OK')
            salida_ok = leer_contador('SIPS_Salida_OK')
            salida_nok = leer_contador('SIPS_Salida_NOK')
            productos_ok = leer_contador('Contador_Productos_OK')
            banda_salida_activa = leer_contador('Banda_Salida_Activa') == 1

            stop = valores['HMI_Stop']
            pause = valores['HMI_Pause']
            emergencia = valores['HMI_Emergencia']
            start = valores['HMI_Start']

            if start and not stop and not emergencia:
                ciclo_iniciado = True

            evacuacion_confirmada = (
                retirados >= generados
                and salida_ok + salida_nok >= entrada_ok
                and productos_ok >= salida_ok
                and not banda_salida_activa
            )
            stop_finalizado = stop and evacuacion_confirmada
            if stop_finalizado:
                ciclo_iniciado = False

            # Prioridad: emergencia > pausa > STOP > funcionamiento normal.
            stop_en_curso = stop and not stop_finalizado and not emergencia
            detenido = (not emergencia and not pause and (
                stop_finalizado or (not stop and not ciclo_iniciado)
            ))

            # Son indicadores; no alteran la logica automatica del PLC.
            nodo_stop_en_curso.write_value(bool(stop_en_curso),
                                          varianttype=ua.VariantType.Boolean)
            nodo_detenido.write_value(bool(detenido),
                                      varianttype=ua.VariantType.Boolean)

            # Este programa es el único escritor de estas seis entradas
            # mientras está abierto. No modifica ninguna variable de salida PLC.
            for nombre, valor in valores.items():
                nodos[nombre].write_value(bool(valor), varianttype=ua.VariantType.Boolean)
                escritura_ok = True
            permiso = bool(nodo_permiso.read_value())

            # Primero publicamos el permiso y el latido, luego activamos el control.
            # La secuencia, STOP y sus condiciones siguen en el HMI Lua.
            # Python solo comunica permiso y peticiones de mando START/STOP.
            sim.setInt32Signal(SENAL_PERMISO, 1 if permiso else 0)
            sim.setFloatSignal(SENAL_HEARTBEAT, float(sim.getSimulationTime()))
            sim.setInt32Signal(SENAL_MODO, 1)
            control_activado = True

            # EMERGENCIA es una variable MANTENIDA del PLC.
            # TRUE -> activar una vez. TRUE sostenido -> no repetir.
            # TRUE a FALSE -> liberar una vez; NO arranca la planta.
            # Se usa pulsarEmergencia() de Lua, que actualiza la HMI real.
            emergencia_plc = bool(nodo_cmd_emergencia.read_value())
            cambio_emergencia = emergencia_plc != emergencia_anterior
            if cambio_emergencia:
                senal = (SENAL_CMD_EMERGENCIA if emergencia_plc
                         else SENAL_CMD_LIBERAR_EMERGENCIA)
                # Un cambio de emergencia invalida pulsadores pendientes.
                for cancelar in (SENAL_CMD_START, SENAL_CMD_STOP,
                                 SENAL_CMD_PAUSE, SENAL_CMD_REANUDAR):
                    sim.setInt32Signal(cancelar, 0)
                for nodo in (nodo_cmd_start, nodo_cmd_stop,
                             nodo_cmd_pause, nodo_cmd_reanudar):
                    nodo.write_value(False, varianttype=ua.VariantType.Boolean)
                sim.setInt32Signal(senal, 1)
                emergencia_anterior = emergencia_plc
                mensajes.put(('comando',
                              'EMERGENCIA ACTIVAR (TRUE)' if emergencia_plc
                              else 'EMERGENCIA LIBERAR (FALSE); requiere START'))

            elif emergencia_plc:
                # Bajo emergencia no se aceptan ordenes de marcha ni pausa.
                for nodo in (nodo_cmd_start, nodo_cmd_stop,
                             nodo_cmd_pause, nodo_cmd_reanudar):
                    if bool(nodo.read_value()):
                        nodo.write_value(False, varianttype=ua.VariantType.Boolean)

            else:
                # El resto conserva pulsos momentaneos; STOP tiene prioridad.
                comandos = (
                    (nodo_cmd_stop, SENAL_CMD_STOP, 'STOP'),
                    (nodo_cmd_pause, SENAL_CMD_PAUSE, 'PAUSE'),
                    (nodo_cmd_reanudar, SENAL_CMD_REANUDAR, 'REANUDAR'),
                    (nodo_cmd_start, SENAL_CMD_START, 'START'),
                )
                pendientes = [(nodo, senal, nombre) for nodo, senal, nombre
                              in comandos if bool(nodo.read_value())]
                if pendientes:
                    _, senal_elegida, nombre_elegido = pendientes[0]
                    if nombre_elegido == 'STOP':
                        for limpiar in (SENAL_CMD_START, SENAL_CMD_PAUSE,
                                        SENAL_CMD_REANUDAR):
                            sim.setInt32Signal(limpiar, 0)
                    sim.setInt32Signal(senal_elegida, 1)
                    for nodo, _, _ in pendientes:
                        nodo.write_value(False, varianttype=ua.VariantType.Boolean)
                    mensajes.put(('comando', nombre_elegido +
                                  ' transmitido a la HMI original de CoppeliaSim'))

            instante = (tuple(valores.items()), permiso, tuple(estados_bandas.items()),
                        tuple(nuevos.items()))
            if instante != anterior:
                mensajes.put(('valores', (valores, permiso, estados_bandas, nuevos)))
                anterior = instante
            # Solo publicamos el latido al completar una iteracion correcta.
            # Si Python deja de comunicarse, el TON de CODESYS vence a 3 s.
            latido = not latido
            nodo_latido.write_value(latido, varianttype=ua.VariantType.Boolean)
            detener.wait(INTERVALO)
    except Exception as exc:
        detalle = type(exc).__name__ + ': ' + str(exc)
        print('SIPS - ERROR REAL DEL PUENTE:', detalle, flush=True)
        traceback.print_exc()
        mensajes.put(('error', 'ERROR DEL PUENTE: ' + detalle))
    finally:
        # Revertir la escena al control Lua original al cerrar o fallar OPC UA.
        if sim is not None and control_activado:
            try:
                sim.setInt32Signal(SENAL_MODO, 0)
                sim.setInt32Signal(SENAL_PERMISO, 0)
                # Evitar mandos pendientes al cerrar el puente.
                for senal in (SENAL_CMD_START, SENAL_CMD_STOP,
                              SENAL_CMD_PAUSE, SENAL_CMD_REANUDAR,
                              SENAL_CMD_EMERGENCIA,
                              SENAL_CMD_LIBERAR_EMERGENCIA):
                    sim.setInt32Signal(senal, 0)
                mensajes.put(('estado', 'Control PLC desactivado: generador vuelve a Lua.'))
            except Exception as exc:
                mensajes.put(('estado', 'ATENCIÓN: revisar señal de modo PLC: ' + str(exc)))
        if opc is not None:
            if escritura_ok:
                # Al desconectar no dejar indicadores operativos congelados.
                try:
                    nodo_stop_en_curso.write_value(False, varianttype=ua.VariantType.Boolean)
                    nodo_detenido.write_value(False, varianttype=ua.VariantType.Boolean)
                    nodo_banda_entrada.write_value(False, varianttype=ua.VariantType.Boolean)
                    for nodo in nodos_bandas.values():
                        nodo.write_value(False, varianttype=ua.VariantType.Boolean)
                    # Apagar los pilotos supervisados si el puente se desconecta.
                    for nodo in nodos_pilotos.values():
                        nodo.write_value(False, varianttype=ua.VariantType.Boolean)
                except Exception:
                    pass
                # No dejar la autorización de generación activa después del puente.
                try:
                    nodos['HMI_Start'].write_value(False, varianttype=ua.VariantType.Boolean)
                    nodos['Puerta_Cerrada'].write_value(False, varianttype=ua.VariantType.Boolean)
                    mensajes.put(('estado', 'Puente detenido: START y puerta del PLC en FALSE.'))
                except Exception as exc:
                    mensajes.put(('estado', 'ATENCIÓN: comprueba HMI_Start y Puerta_Cerrada en PLC: ' + str(exc)))
            try:
                if control_activado:
                    for nodo in (nodo_cmd_start, nodo_cmd_stop,
                                 nodo_cmd_pause, nodo_cmd_reanudar):
                        nodo.write_value(False, varianttype=ua.VariantType.Boolean)
            except Exception:
                pass
            try:
                opc.disconnect()
            except Exception:
                pass
        mensajes.put(('terminado', None))


def iniciar():
    ventana = tk.Tk()
    ventana.title('SIPS — Puente unico: 6 bandas + supervision UR10')
    ventana.geometry('1000x650')
    ventana.minsize(940, 610)

    ttk.Label(ventana, text='SIPS — CoppeliaSim ↔ Python ↔ CODESYS',
              font=('Segoe UI', 13, 'bold')).pack(pady=(15, 6))
    ttk.Label(ventana, text='START, STOP, PAUSE y REANUDAR: pulsos. EMERGENCIA: TRUE activa, FALSE libera; luego START.',
              wraplength=900).pack(pady=(0, 12))

    frame = ttk.Frame(ventana)
    frame.pack(fill='both', expand=True, padx=24)
    nombres = ([n for n, _ in ENTRADAS] +
               ['Puerta_Cerrada', 'PLC_Permiso_Generar', 'SIPS_Banda_Entrada_Activa'] +
               [n for n, _ in BANDAS_ADICIONALES] +
               [n for n, _ in SENALES_PILOTOS] +
               ['Puerta_Abierta', 'SIPS_Limpieza_Banda_Activa',
                'SIPS_Robot_Trabajando'])
    etiquetas = {}
    # Dos columnas para mostrar el conjunto sin barra de desplazamiento.
    mitad = (len(nombres) + 1) // 2
    for i, nombre in enumerate(nombres):
        fila = i % mitad
        base = 0 if i < mitad else 2
        ttk.Label(frame, text=nombre, font=('Segoe UI', 8)).grid(
            row=fila, column=base, sticky='w', pady=5, padx=(0, 5))
        etiqueta = ttk.Label(frame, text='—', font=('Consolas', 9, 'bold'))
        etiqueta.grid(row=fila, column=base + 1, sticky='w', padx=(4, 12), pady=5)
        etiquetas[nombre] = etiqueta

    ultima_orden = tk.StringVar(value='Última orden desde CODESYS: ninguna')
    ttk.Label(ventana, textvariable=ultima_orden, wraplength=620).pack(padx=15, pady=3)
    texto_estado = tk.StringVar(value='Esperando contraseña del PLC…')
    ttk.Label(ventana, textvariable=texto_estado, wraplength=620, foreground='#aa2222').pack(padx=15, pady=10)
    detener = threading.Event()
    mensajes = queue.Queue()
    terminado = False
    error_real = None

    def solicitar_cierre():
        texto_estado.set('Deteniendo puente y liberando OPC UA…')
        detener.set()
        boton.configure(state='disabled')
        if terminado:
            ventana.destroy()

    boton = ttk.Button(ventana, text='DETENER Y CERRAR', command=solicitar_cierre)
    boton.pack(pady=(0, 15))
    ventana.protocol('WM_DELETE_WINDOW', solicitar_cierre)

    clave = simpledialog.askstring('SIPS — Conexión PLC',
                                  'Contraseña admin del PLC virtual:',
                                  show='*', parent=ventana)
    if not clave:
        ventana.destroy()
        return

    trabajador = threading.Thread(target=trabajar, args=(clave, detener, mensajes), daemon=True)
    trabajador.start()
    clave = None

    def actualizar():
        nonlocal terminado, error_real
        try:
            while True:
                tipo, payload = mensajes.get_nowait()
                if tipo == 'error':
                    error_real = payload
                    texto_estado.set(payload)
                elif tipo == 'estado':
                    if error_real is None:
                        texto_estado.set(payload)
                elif tipo == 'comando':
                    ultima_orden.set('Última orden desde CODESYS: ' + payload)
                elif tipo == 'valores':
                    valores, permiso, estados_bandas, nuevos = payload
                    for nombre, valor in valores.items():
                        etiquetas[nombre].configure(text='TRUE' if valor else 'FALSE')
                    etiquetas['PLC_Permiso_Generar'].configure(
                        text='TRUE' if permiso else 'FALSE')
                    for nombre, activo in estados_bandas.items():
                        etiquetas[nombre].configure(text='TRUE' if activo else 'FALSE')
                    for nombre, activo in nuevos.items():
                        etiquetas[nombre].configure(text='TRUE' if activo else 'FALSE')
                elif tipo == 'terminado':
                    terminado = True
                    boton.configure(text='CERRAR VENTANA', state='normal')
        except queue.Empty:
            pass
        if not ventana.winfo_exists():
            return
        if terminado and detener.is_set():
            ventana.destroy()
        else:
            ventana.after(200, actualizar)

    ventana.after(200, actualizar)
    ventana.mainloop()


if __name__ == '__main__':
    iniciar()

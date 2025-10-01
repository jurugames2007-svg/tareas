# =============================================================================
# IMPORTACIONES Y CONFIGURACIÓN INICIAL
# =============================================================================

import numpy as np
import cv2
import cv2.aruco as aruco
import tkinter as tk
from tkinter import ttk, Scale, messagebox, filedialog
from PIL import Image, ImageTk, ImageOps
import threading
import random
import time
import os
from aruco_generador import crear_aruco

# Importar funcionalidades del láser y robot
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

# Importar funcionalidades del robot
try:
    import comun
    import conect
except ImportError:
    comun = None
    conect = None

# =============================================================================
# CONFIGURACIÓN DE ARUCO
# =============================================================================

# Diccionario y parámetros del marcador ArUco
diccionario_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
parametros_aruco = aruco.DetectorParameters()

# =============================================================================
# DETECTOR ARUCO GLOBAL
# =============================================================================
camara = None 
# Detector ArUco global (compatible con OpenCV 4.7+)
detector_aruco = None

def crear_detector_aruco():
    """Crea el detector ArUco con el diccionario y parámetros actuales"""
    global detector_aruco
    try:
        # Intenta usar la nueva API (OpenCV >= 4.7)
        detector_aruco = aruco.ArucoDetector(diccionario_aruco, parametros_aruco)
        return True
    except AttributeError:
        # Fallback para versiones anteriores
        detector_aruco = None
        return False

# Inicializar detector
usar_nueva_api = crear_detector_aruco()

# =============================================================================
# CONFIGURACIÓN DE COLORES Y MEDICIÓN
# =============================================================================

# Rangos de color en HSV para detección
# Rojo
rojo_bajo1 = np.array([0, 100, 100], dtype=np.uint8)
rojo_alto1 = np.array([10, 255, 255], dtype=np.uint8)
rojo_bajo2 = np.array([160, 100, 100], dtype=np.uint8)
rojo_alto2 = np.array([179, 255, 255], dtype=np.uint8)

# Verde
verde_bajo = np.array([40, 100, 100], dtype=np.uint8)
verde_alto = np.array([80, 255, 255], dtype=np.uint8)

# Azul
azul_bajo = np.array([100, 100, 100], dtype=np.uint8)
azul_alto = np.array([140, 255, 255], dtype=np.uint8)

# Tamaño real del marcador ArUco en centímetros
tamano_aruco_cm = 3.0
proporcion_cm_por_pixel = None  # Inicializar la variable global

# =============================================================================
# FUNCIONES DEL LÁSER - COMUNICACIÓN SERIAL
# =============================================================================

WAKE_DELAY = 2.0  # s

def open_serial(port: str, baud: int = 115200, timeout: float = 1.0):
    """Abre conexión serial para el láser"""
    if serial is None:
        raise RuntimeError("pyserial no está instalado. Instala con: pip install pyserial")
    ser = serial.Serial(port, baudrate=baud, timeout=timeout)
    ser.write(b"\r\n\r\n")
    time.sleep(WAKE_DELAY)
    ser.reset_input_buffer()
    return ser

def _readline(ser):
    """Lee una línea del puerto serial"""
    raw = ser.readline()
    return raw.decode(errors="ignore").strip() if raw else ""

def send_cmd(ser, cmd: str) -> str:
    """Envía una línea y espera 'ok' / 'error' / 'ALARM'."""
    cmd = cmd.strip()
    if not cmd:
        return "ok"
    ser.write((cmd + "\n").encode())
    while True:
        line = _readline(ser)
        if not line:
            continue
        L = line.lower()
        if line == "ok" or L.startswith("error") or line.upper().startswith("ALARM"):
            return line

def mm_per_pixel(ppmm: float) -> float:
    """Convierte píxeles por mm a mm por píxel"""
    return 1.0 / float(ppmm)

def to_grayscale(img: Image.Image, invert: bool) -> Image.Image:
    """Convierte imagen a escala de grises"""
    g = ImageOps.grayscale(img)
    if invert:
        g = ImageOps.invert(g)
    return g

# =============================================================================
# FUNCIONES DEL LÁSER - PROCESAMIENTO DE IMÁGENES
# =============================================================================

def prepare_image(path: str, size_mm, ppmm: float, invert: bool, mode: str) -> Image.Image:
    """Prepara imagen para grabado láser"""
    w_mm, h_mm = size_mm
    target_px = (int(round(w_mm * ppmm)), int(round(h_mm * ppmm)))

    img = Image.open(path).convert("L")
    img = img.resize(target_px, Image.Resampling.LANCZOS)

    if mode == "threshold":
        if invert:
            img = ImageOps.invert(img)
        bw = img.point(lambda p: 0 if p < 128 else 255, "1")
        return bw

    if mode == "dither":
        g = to_grayscale(img.convert("RGB"), invert)
        return g.convert("1")

    return to_grayscale(img.convert("RGB"), invert)

def gamma_correct(v: float, gamma: float) -> float:
    """Aplica corrección gamma"""
    return pow(max(0.0, min(1.0, v)), gamma)

# =============================================================================
# FUNCIONES DEL LÁSER - GENERACIÓN DE GCODE
# =============================================================================

def raster_to_gcode(img: Image.Image, *, ppmm: float, origin, f_engrave: float, 
                   f_travel: float, s_max: int, mode: str, gamma_val: float):
    """Convierte imagen raster a G-code"""
    px_w, px_h = img.size
    step = mm_per_pixel(ppmm)
    ox, oy = origin

    OVERSCAN_MM = 0.6
    S_FIXED = s_max

    yield ";; --- BEGIN ---"
    yield "G21"
    yield "G90"
    yield "M5"
    yield f"F{f_travel:.4f}"

    for row in range(px_h):
        y_mm = oy + (px_h - 1 - row) * step

        if row % 2 == 0:
            x_range = range(0, px_w)
        else:
            x_range = range(px_w - 1, -1, -1)

        first_x = x_range.start if isinstance(x_range, range) else x_range[0]
        x0_mm = ox + first_x * step
        yield f"G0 X{(x0_mm - OVERSCAN_MM):.4f} Y{y_mm:.4f}"
        yield f"F{f_engrave:.4f}"

        def pixel_intensity(col: int) -> float:
            if img.mode == "1":
                return 1.0 if img.getpixel((col, row)) == 0 else 0.0
            else:
                return (255 - img.getpixel((col, row))) / 255.0

        seg_start = None
        seg_power = None

        for x in x_range:
            inten = pixel_intensity(x)

            if mode == "grayscale":
                p = gamma_correct(inten, gamma_val)
                s_val = int(round(p * s_max))
                if s_val > 0 and s_val < 50:
                    s_val = 50

                if s_val > 0:
                    if seg_start is None:
                        seg_start = x
                        seg_power = None
                    if seg_power != s_val:
                        yield f"M4 S{s_val}"
                        seg_power = s_val
                    x_mm = ox + x * step
                    yield f"G1 X{x_mm:.4f} Y{y_mm:.4f}"
                else:
                    if seg_start is not None:
                        x_mm = ox + x * step
                        yield f"G1 X{(x_mm + OVERSCAN_MM):.4f} Y{y_mm:.4f}"
                        yield "M5"
                        seg_start = None
                        seg_power = None
            else:
                # Modo binario para ArUcos y otros
                if inten > 0.5:
                    if seg_start is None:
                        seg_start = x
                else:
                    if seg_start is not None:
                        x_mm = ox + x * step
                        yield f"M4 S{S_FIXED}"
                        yield f"G1 X{x_mm:.4f} Y{y_mm:.4f}"
                        yield f"G1 X{(x_mm + OVERSCAN_MM):.4f} Y{y_mm:.4f}"
                        yield "M5"
                        seg_start = None

        last_x = x_range[-1] if hasattr(x_range, "__getitem__") else (
            x_range.stop - 1 if x_range.step > 0 else x_range.stop + 1
        )
        end_x_mm = ox + last_x * step
        if seg_start is not None:
            if mode == "grayscale":
                yield f"M4 S{seg_power if seg_power is not None else s_max}"
            else:
                yield f"M4 S{S_FIXED}"
            yield f"G1 X{end_x_mm:.4f} Y{y_mm:.4f}"
            yield f"G1 X{(end_x_mm + OVERSCAN_MM):.4f} Y{y_mm:.4f}"
            yield "M5"

        yield f"F{f_travel:.4f}"

    yield "M5"
    yield ";; --- END ---"

def stream_to_grbl(ser, gcode_text: str) -> int:
    """Envía G-code a GRBL"""
    send_cmd(ser, "$X")
    send_cmd(ser, "G21")
    send_cmd(ser, "G90")

    errors = 0
    for raw in gcode_text.splitlines():
        line = raw.strip()
        if not line or line.startswith(";"):
            continue
        resp = send_cmd(ser, line)
        if resp != "ok":
            errors += 1
            send_cmd(ser, "M5")
            break

    send_cmd(ser, "M5")
    return 0 if errors == 0 else 2

def move_to_offset_and_set_origin(ser, dx: float = 0.0, dy: float = 0.0, feed: int = 1000):
    """Mueve a offset y establece origen"""
    send_cmd(ser, "G90")
    send_cmd(ser, "G91")
    parts = []
    if abs(dx) > 0: parts.append(f"X{dx}")
    if abs(dy) > 0: parts.append(f"Y{dy}")
    if parts:
        send_cmd(ser, f"G1 {' '.join(parts)} F{int(feed)}")
    send_cmd(ser, "G90")
    send_cmd(ser, "G92 X0 Y0")

def move_back_to_machine_origin(ser):
    """Retorna al origen de máquina"""
    send_cmd(ser, "G92.1")
    send_cmd(ser, "G90")
    send_cmd(ser, "G53 G0 X0 Y0")

def generate_gcode_text(*, image_path: str, size_mm, ppmm: float, mode: str, 
                       invert: bool, gamma_val: float, origin_xy, f_engrave: float, 
                       f_travel: float, s_max: int) -> str:
    """Genera G-code completo para una imagen"""
    img = prepare_image(image_path, size_mm, ppmm, invert, mode)
    lines = list(
        raster_to_gcode(
            img,
            ppmm=ppmm,
            origin=origin_xy,
            f_engrave=f_engrave,
            f_travel=f_travel,
            s_max=s_max,
            mode=mode,
            gamma_val=gamma_val,
        )
    )
    return "\n".join(lines) + "\n"

# =============================================================================
# FUNCIONES DE MEDICIÓN Y DETECCIÓN DE OBJETOS
# =============================================================================

# Función para medir objetos detectados
def medir_objeto(contorno, imagen, proporcion_cm_por_pixel):
    if proporcion_cm_por_pixel is None:
        return None
    
    x, y, ancho, alto = cv2.boundingRect(contorno)
    ancho_cm = ancho * proporcion_cm_por_pixel
    alto_cm = alto * proporcion_cm_por_pixel
    area_cm2 = ancho_cm * alto_cm
    
    # Retornar información del objeto para poder ordenarlo
    return {
        'contorno': contorno,
        'bbox': (x, y, ancho, alto),
        'ancho_cm': ancho_cm,
        'alto_cm': alto_cm,
        'area_cm2': area_cm2,
        'centro': (x + ancho // 2, y + alto // 2)
    }

def dibujar_objeto_medido(objeto_info, imagen, color=(0, 255, 0), numero=None):
    """Dibuja un objeto medido en la imagen"""
    x, y, ancho, alto = objeto_info['bbox']
    ancho_cm = objeto_info['ancho_cm']
    alto_cm = objeto_info['alto_cm']
    area_cm2 = objeto_info['area_cm2']
    centro_x, centro_y = objeto_info['centro']
    
    # Dibujar rectángulo
    cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), color, 2)
    
    # Dibujar centro
    cv2.circle(imagen, (centro_x, centro_y), 3, (0, 0, 255), -1)
    
    # Mostrar número si se proporciona
    if numero is not None:
        cv2.putText(imagen, f"#{numero}", (x, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
    
    # Mostrar medidas
    texto_ancho = f"Ancho: {ancho_cm:.2f} cm"
    texto_alto = f"Alto: {alto_cm:.2f} cm"
    texto_area = f"Area: {area_cm2:.2f} cm²"
    
    cv2.putText(imagen, texto_ancho, (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(imagen, texto_alto, (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(imagen, texto_area, (x, y - 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

def dibujar_aruco_ordenado(esquina, id_aruco, imagen, color=(0, 255, 0), numero_orden=None):
    """Dibuja un ArUco con información de ordenamiento"""
    puntos = esquina[0]
    
    # Calcular centro del ArUco
    centro = np.mean(puntos, axis=0).astype(int)
    
    # Dibujar un círculo en el centro
    cv2.circle(imagen, tuple(centro), 8, color, -1)
    
    # Mostrar número de orden si se proporciona
    if numero_orden is not None:
        cv2.putText(imagen, f"#{numero_orden}", (centro[0] - 15, centro[1] - 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
        cv2.putText(imagen, f"#{numero_orden}", (centro[0] - 15, centro[1] - 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    
    # Mostrar ID del ArUco
    cv2.putText(imagen, f"ID:{id_aruco}", (centro[0] - 20, centro[1] + 35), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 3)
    cv2.putText(imagen, f"ID:{id_aruco}", (centro[0] - 20, centro[1] + 35), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

# =============================================================================
# CLASE VENTANA DEL LÁSER - INTERFAZ GRÁFICA
# =============================================================================

# ===================== Ventana del Láser =====================

class VentanaLaser:
    def __init__(self, parent, aruco_info=None):
        self.parent = parent
        self.ventana = tk.Toplevel(parent)
        self.ventana.title("Control del Láser")
        self.ventana.geometry("520x700")  # Aumentar altura
        self.ventana.resizable(True, True)  # Permitir redimensionar
        
        # Variables del láser
        self.ser_laser = None
        self.laser_port = tk.StringVar(value="COM3")
        self.laser_baud = tk.StringVar(value="115200")
        self.selected_image = tk.StringVar(value="No hay imagen seleccionada")
        self.image_path = None
        self.laser_inicializado = False  # Estado de inicialización
        self.aruco_info = aruco_info  # Información del ArUco generado
        
        # Si tenemos información del ArUco, configurar automáticamente la imagen correcta
        is_aruco_image = False
        if self.aruco_info and 'imagen_path' in self.aruco_info:
            # Para obtener fondo negro quemado + patrón blanco, usar imagen NORMAL sin inversión
            # Esto hará que el láser grabe las áreas BLANCAS (fondo) y deje las áreas NEGRAS (patrón)
            self.image_path = self.aruco_info['imagen_path_normal'] if 'imagen_path_normal' in self.aruco_info else self.aruco_info['imagen_path']
            nombre_archivo = os.path.basename(self.image_path)
            self.selected_image.set(f"ArUco para fondo negro: {nombre_archivo}")
            is_aruco_image = True  # Activar inversión para quemar el fondo
        
        # Parámetros de grabado (NO MODIFICAR VALORES)
        self.size_mm_x = tk.DoubleVar(value=20.0)
        self.size_mm_y = tk.DoubleVar(value=20.0)
        self.ppmm = tk.DoubleVar(value=5.0)
        self.f_engrave = tk.DoubleVar(value=1000.0)
        self.f_travel = tk.DoubleVar(value=1000.0)
        self.s_max = tk.IntVar(value=400)
        self.gamma_val = tk.DoubleVar(value=0.6)
        self.offset_x = tk.DoubleVar(value=0.0)
        self.offset_y = tk.DoubleVar(value=0.0)
        self.mode = tk.StringVar(value="Aruco")
        # Para imágenes ArUco, activar inversión automáticamente para grabar el fondo
        self.invert = tk.BooleanVar(value=is_aruco_image)
        
        self.crear_interfaz_laser()
        
    # =========================================================================
    # MÉTODOS DE CREACIÓN DE INTERFAZ - VENTANA LÁSER
    # =========================================================================
        
    def crear_interfaz_laser(self):
        # Frame principal
        main_frame = ttk.Frame(self.ventana, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Conexión del láser
        conn_frame = ttk.LabelFrame(main_frame, text="Conexión del Láser", padding="5")
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(conn_frame, text="Puerto:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        puerto_combo = ttk.Combobox(conn_frame, textvariable=self.laser_port, 
                                   values=[f"COM{i}" for i in range(1, 21)], width=10)
        puerto_combo.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(conn_frame, text="Baudrate:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.laser_baud,
                                 values=["9600", "19200", "38400", "57600", "115200"], width=10)
        baud_combo.grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Button(conn_frame, text="Conectar", command=self.conectar_laser).grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(conn_frame, text="Desconectar", command=self.desconectar_laser).grid(row=0, column=5, padx=5, pady=5)
        
        # Selección de imagen
        img_frame = ttk.LabelFrame(main_frame, text="Selección de Imagen", padding="5")
        img_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(img_frame, textvariable=self.selected_image).pack(side=tk.LEFT, padx=5)
        ttk.Button(img_frame, text="Usar ArUco Generado", command=self.usar_aruco_generado).pack(side=tk.RIGHT, padx=5)
        ttk.Button(img_frame, text="Seleccionar Imagen", command=self.seleccionar_imagen).pack(side=tk.RIGHT, padx=5)
        ttk.Button(img_frame, text="Inicializar Láser", command=self.inicializar_laser).pack(side=tk.RIGHT, padx=(5, 10))
        
        # Parámetros de grabado
        params_frame = ttk.LabelFrame(main_frame, text="Parámetros de Grabado", padding="5")
        params_frame.pack(fill=tk.X, pady=(0, 10))
        
    # Tamaño
        size_frame = ttk.LabelFrame(params_frame, text="Tamaño del Aruco", padding="5")
        size_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0,10), pady=2)
        ttk.Label(size_frame, text="Tamaño (mm)").grid(row=0, column=0, sticky=tk.W, pady=(0,5))
        ttk.Label(size_frame, text="X:").grid(row=1, column=0, sticky=tk.W)
        ttk.Entry(size_frame, textvariable=self.size_mm_x, width=8).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(size_frame, text="Y:").grid(row=2, column=0, sticky=tk.W)
        ttk.Entry(size_frame, textvariable=self.size_mm_y, width=8).grid(row=2, column=1, sticky=tk.W)
        
        # Resolución
        res_frame = ttk.Frame(params_frame)
        res_frame.pack(fill=tk.X, pady=2)
        ttk.Label(res_frame, text="Píxeles por mm:").pack(side=tk.LEFT)
        ttk.Entry(res_frame, textvariable=self.ppmm, width=8, state="readonly").pack(side=tk.LEFT, padx=(10, 0))
        
        # Velocidades
        vel_frame = ttk.Frame(params_frame)
        vel_frame.pack(fill=tk.X, pady=2)
        ttk.Label(vel_frame, text="Vel. grabado:").pack(side=tk.LEFT)
        ttk.Entry(vel_frame, textvariable=self.f_engrave, width=8, state="readonly").pack(side=tk.LEFT, padx=(5, 0))
        ttk.Label(vel_frame, text="Vel. viaje:").pack(side=tk.LEFT, padx=(15, 0))
        ttk.Entry(vel_frame, textvariable=self.f_travel, width=8, state="readonly").pack(side=tk.LEFT, padx=(5, 0))
        
        # Potencia y gamma
        power_frame = ttk.Frame(params_frame)
        power_frame.pack(fill=tk.X, pady=2)
        ttk.Label(power_frame, text="Potencia máx:").pack(side=tk.LEFT)
        ttk.Entry(power_frame, textvariable=self.s_max, width=8, state="readonly").pack(side=tk.LEFT, padx=(5, 0))
        ttk.Label(power_frame, text="Gamma:").pack(side=tk.LEFT, padx=(15, 0))
        ttk.Entry(power_frame, textvariable=self.gamma_val, width=8, state="readonly").pack(side=tk.LEFT, padx=(5, 0))
        
        # Offset
        offset_frame = ttk.Frame(params_frame)
        offset_frame.pack(fill=tk.X, pady=2)
        ttk.Label(offset_frame, text="Offset (mm):").pack(side=tk.LEFT)
        ttk.Label(offset_frame, text="X:").pack(side=tk.LEFT, padx=(10, 0))
        ttk.Entry(offset_frame, textvariable=self.offset_x, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Label(offset_frame, text="Y:").pack(side=tk.LEFT, padx=(10, 0))
        ttk.Entry(offset_frame, textvariable=self.offset_y, width=8).pack(side=tk.LEFT, padx=2)
        
        # Modo y opciones
        mode_frame = ttk.Frame(params_frame)
        mode_frame.pack(fill=tk.X, pady=2)
        ttk.Label(mode_frame, text="Modo:").pack(side=tk.LEFT)
        mode_combo = ttk.Combobox(mode_frame, textvariable=self.mode, state="readonly", width=12)
        mode_combo['values'] = ["Aruco", "threshold", "grayscale", "dither"]
        mode_combo.pack(side=tk.LEFT, padx=(5, 0))
        ttk.Checkbutton(mode_frame, text="Invertir", variable=self.invert).pack(side=tk.LEFT, padx=(15, 0))
        
        # Etiqueta informativa sobre inversión para ArUcos
        info_frame = ttk.Frame(params_frame)
        info_frame.pack(fill=tk.X, pady=2)
        info_label = ttk.Label(info_frame, 
                              text="✅ CONFIGURACIÓN OPTIMIZADA PARA ARUCO:\n" +
                                   "• Modo: 'Aruco' (procesamiento binario optimizado)\n" +
                                   "• Inversión: Activada para fondo negro + patrón blanco\n" +
                                   "• Potencia: Ajustada manualmente (recomendado 400-500)\n" +
                                   "• RESULTADO: Líneas definidas sin áreas grises", 
                              font=("Arial", 7), foreground="green", justify=tk.LEFT)
        info_label.pack(side=tk.LEFT)
        
        # Log del láser
        log_frame = ttk.LabelFrame(main_frame, text="Log del Láser", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        self.laser_log = tk.Text(log_frame, height=10, state=tk.DISABLED)  # Reducir altura
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.laser_log.yview)
        self.laser_log.configure(yscrollcommand=scrollbar.set)
        
        self.laser_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Botones de control
        control_frame = ttk.LabelFrame(main_frame, text="Controles", padding="10")
        control_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Primera fila de botones
        control_row1 = ttk.Frame(control_frame)
        control_row1.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Button(control_row1, text="Generar G-Code", command=self.generar_gcode, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_row1, text="Iniciar Grabado", command=self.iniciar_grabado, width=15).pack(side=tk.LEFT, padx=5)
        
        # Segunda fila de botones
        control_row2 = ttk.Frame(control_frame)
        control_row2.pack(fill=tk.X)
        
        ttk.Button(control_row2, text="Cerrar", command=self.cerrar_ventana, width=15).pack(side=tk.LEFT, padx=5)
        
    # =========================================================================
    # MÉTODOS DE FUNCIONALIDAD - VENTANA LÁSER
    # =========================================================================

    def log_laser(self, mensaje):
        """Añade mensaje al log del láser"""
        self.laser_log.config(state=tk.NORMAL)
        self.laser_log.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {mensaje}\n")
        self.laser_log.see(tk.END)
        self.laser_log.config(state=tk.DISABLED)
        
    def conectar_laser(self):
        """Conecta al láser"""
        try:
            if self.ser_laser and getattr(self.ser_laser, "is_open", False):
                self.log_laser("Ya conectado al láser")
                return
                
            port = self.laser_port.get()
            baud = int(self.laser_baud.get())
            self.ser_laser = open_serial(port, baud)
            self.log_laser(f"✅ Conectado al láser en {port} @ {baud}")
            
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo conectar al láser:\n{e}")
            self.log_laser(f"❌ Error de conexión: {e}")
            
    def desconectar_laser(self):
        """Desconecta del láser"""
        try:
            if self.ser_laser and getattr(self.ser_laser, "is_open", False):
                send_cmd(self.ser_laser, "M5")  # Apagar láser
                self.ser_laser.close()
                self.log_laser("🔌 Láser desconectado")
            self.laser_inicializado = False  # Resetear estado
        except Exception as e:
            self.log_laser(f"Error al desconectar: {e}")
            self.laser_inicializado = False
            
    def seleccionar_imagen(self):
        """Selecciona archivo de imagen"""
        archivo = filedialog.askopenfilename(
            title="Seleccionar imagen",
            filetypes=[
                ("Imágenes", "*.png *.jpg *.jpeg *.bmp *.gif"),
                ("Todos los archivos", "*.*")
            ]
        )
        if archivo:
            self.image_path = archivo
            nombre = os.path.basename(archivo)
            self.selected_image.set(f"Imagen: {nombre}")
            self.log_laser(f"Imagen seleccionada: {nombre}")

    def usar_aruco_generado(self):
        """Usa automáticamente la imagen del ArUco generado para fondo negro"""
        if self.aruco_info and 'imagen_path' in self.aruco_info:
            # Para fondo negro + patrón blanco: usar imagen NORMAL con INVERSIÓN
            # Imagen normal: fondo blanco + patrón negro
            # Con inversión: fondo negro + patrón blanco
            # Láser graba áreas negras → graba el fondo, deja el patrón
            self.image_path = self.aruco_info['imagen_path_normal'] if 'imagen_path_normal' in self.aruco_info else self.aruco_info['imagen_path']
            nombre_archivo = os.path.basename(self.image_path)
            self.selected_image.set(f"ArUco fondo negro: {nombre_archivo}")
            self.invert.set(True)  # ACTIVAR inversión para quemar el fondo
            self.log_laser(f"🎯 Usando imagen normal del ArUco: {nombre_archivo}")
            self.log_laser("✅ Inversión ACTIVADA - Resultado: fondo negro + patrón blanco")
        else:
            messagebox.showwarning("Advertencia", "No hay un ArUco generado disponible.\nGenera un ArUco primero desde la ventana principal.")
            
    def inicializar_laser(self):
        """Inicializa el láser después de seleccionar imagen"""
        if not self.image_path:
            if self.aruco_info and 'imagen_path' in self.aruco_info:
                # Para fondo negro + patrón blanco: usar imagen NORMAL con INVERSIÓN
                self.image_path = self.aruco_info['imagen_path_normal'] if 'imagen_path_normal' in self.aruco_info else self.aruco_info['imagen_path']
                nombre_archivo = os.path.basename(self.image_path)
                self.selected_image.set(f"ArUco fondo negro: {nombre_archivo}")
                self.invert.set(True)  # ACTIVAR inversión
                self.log_laser(f"🎯 Usando imagen normal del ArUco: {nombre_archivo}")
                self.log_laser("✅ Inversión ACTIVADA - Resultado: fondo negro quemado + patrón blanco")
            else:
                messagebox.showwarning("Advertencia", "Selecciona una imagen primero o genera un ArUco")
                return
            
        if not self.ser_laser or not getattr(self.ser_laser, "is_open", False):
            messagebox.showwarning("Advertencia", "Conecta al láser primero")
            return
            
        try:
            self.log_laser("🔧 Inicializando láser...")
            
            # Comandos de inicialización del láser
            send_cmd(self.ser_laser, "$X")  # Desbloquear alarmas
            send_cmd(self.ser_laser, "G21")  # Unidades en milímetros
            send_cmd(self.ser_laser, "G90")  # Coordenadas absolutas
            send_cmd(self.ser_laser, "M5")   # Láser apagado
#           send_cmd(self.ser_laser, "G0 X0 Y0")  Ir al origen
            
            # Mover a posición de offset si está configurado
            if abs(self.offset_x.get()) > 0 or abs(self.offset_y.get()) > 0:
                self.log_laser("📍 Moviendo a posición de trabajo...")
                move_to_offset_and_set_origin(
                    self.ser_laser, 
                    dx=self.offset_x.get(), 
                    dy=self.offset_y.get(), 
                    feed=1000
                )
                self.log_laser(f"✅ Posicionado en offset X:{self.offset_x.get()}, Y:{self.offset_y.get()}")
            else:
                self.log_laser("✅ Láser inicializado en origen (0,0)")
                
            # Marcar como inicializado
            self.laser_inicializado = True
            self.log_laser("🎯 Láser listo para grabar - Presiona 'Iniciar Grabado' cuando estés listo")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error inicializando láser:\n{e}")
            self.log_laser(f"❌ Error en inicialización: {e}")
            self.laser_inicializado = False
            
    def generar_gcode(self):
        """Genera G-code para la imagen seleccionada"""
        if not self.image_path:
            messagebox.showwarning("Advertencia", "Selecciona una imagen primero")
            return
            
        try:
            self.log_laser("Generando G-code...")
            size_mm = (self.size_mm_x.get(), self.size_mm_y.get())
            
            gcode = generate_gcode_text(
                image_path=self.image_path,
                size_mm=size_mm,
                ppmm=self.ppmm.get(),
                mode=self.mode.get(),
                invert=self.invert.get(),
                gamma_val=self.gamma_val.get(),
                origin_xy=(0.0, 0.0),
                f_engrave=self.f_engrave.get(),
                f_travel=self.f_travel.get(),
                s_max=self.s_max.get()
            )
            
            # Guardar G-code en archivo
            gcode_file = filedialog.asksaveasfilename(
                title="Guardar G-code",
                defaultextension=".gcode",
                filetypes=[("G-code", "*.gcode"), ("Texto", "*.txt")]
            )
            
            if gcode_file:
                with open(gcode_file, 'w') as f:
                    f.write(gcode)
                self.log_laser(f"✅ G-code guardado en: {os.path.basename(gcode_file)}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error generando G-code:\n{e}")
            self.log_laser(f"❌ Error generando G-code: {e}")
            
    def iniciar_grabado(self):
        """Inicia el proceso de grabado después de la inicialización"""
        if not self.image_path:
            messagebox.showwarning("Advertencia", "Selecciona una imagen primero")
            return
            
        if not self.ser_laser or not getattr(self.ser_laser, "is_open", False):
            messagebox.showwarning("Advertencia", "Conecta al láser primero")
            return
            
        if not self.laser_inicializado:
            messagebox.showwarning("Advertencia", "Inicializa el láser primero con el botón 'Inicializar Láser'")
            return
            
        # Verificar que el láser esté en posición correcta
        respuesta = messagebox.askyesno("Confirmar Grabado", 
                                       "¿El láser está en la posición correcta para comenzar?\n\n" +
                                       "⚠️ ADVERTENCIA: El grabado comenzará inmediatamente.\n\n" +
                                       "Presiona 'Sí' para iniciar el grabado.")
        if not respuesta:
            self.log_laser("⏸️ Grabado cancelado por el usuario")
            return
            
        try:
            self.log_laser("🔥 INICIANDO GRABADO...")
            
            # Generar G-code
            size_mm = (self.size_mm_x.get(), self.size_mm_y.get())
            gcode = generate_gcode_text(
                image_path=self.image_path,
                size_mm=size_mm,
                ppmm=self.ppmm.get(),
                mode=self.mode.get(),
                invert=self.invert.get(),
                gamma_val=self.gamma_val.get(),
                origin_xy=(0.0, 0.0),
                f_engrave=self.f_engrave.get(),
                f_travel=self.f_travel.get(),
                s_max=self.s_max.get()
            )
            
            # Enviar G-code al láser
            self.log_laser("📤 Enviando comandos de grabado al láser...")
            resultado = stream_to_grbl(self.ser_laser, gcode)
            
            if resultado == 0:
                self.log_laser("✅ GRABADO COMPLETADO EXITOSAMENTE")
                self.log_laser("🏠 El láser ha regresado a su posición inicial")
                # Resetear estado de inicialización para próximo grabado
                self.laser_inicializado = False
            else:
                self.log_laser("❌ ERROR DURANTE EL GRABADO")
                self.laser_inicializado = False
                
        except Exception as e:
            messagebox.showerror("Error", f"Error durante el grabado:\n{e}")
            self.log_laser(f"❌ Error durante el grabado: {e}")
            self.laser_inicializado = False
            

        
    def cerrar_ventana(self):
        """Cierra la ventana del láser"""
        self.desconectar_laser()
        self.ventana.destroy()

# =============================================================================
# CLASE VENTANA DEL ROBOT - INTERFAZ GRÁFICA PARA CONTROL DE ROBOT
# =============================================================================

class VentanaRobot:
    def __init__(self, parent):
        self.parent = parent
        self.ventana = tk.Toplevel(parent)
        self.ventana.title("Control del Robot Scorbot-ER V Plus")
        self.ventana.geometry("600x500")
        self.ventana.resizable(True, True)
        
        # Variables del robot
        self.ser_robot = None
        self.robot_port = tk.StringVar(value="COM1")
        self.robot_connected = False
        self.escuchando_automatico = False
        self.hilo_escucha = None
        
        self.crear_interfaz_robot()
        
    def crear_interfaz_robot(self):
        # Frame principal
        main_frame = ttk.Frame(self.ventana, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Conexión del robot
        conn_frame = ttk.LabelFrame(main_frame, text="Conexión del Robot", padding="5")
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(conn_frame, text="Puerto:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        puerto_combo = ttk.Combobox(conn_frame, textvariable=self.robot_port)
        puerto_combo['values'] = [port.device for port in serial.tools.list_ports.comports()] if serial else ["COM1", "COM2", "COM3", "COM4"]
        puerto_combo.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(conn_frame, text="Conectar", command=self.conectar_robot).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(conn_frame, text="Desconectar", command=self.desconectar_robot).grid(row=0, column=3, padx=5, pady=5)
        
        # Estado de conexión
        self.lbl_estado = ttk.Label(conn_frame, text="DESCONECTADO", background="red", foreground="white")
        self.lbl_estado.grid(row=0, column=4, padx=10, pady=5)
        
        # Comandos manuales (de comun.py)
        manual_frame = ttk.LabelFrame(main_frame, text="Comandos Manuales", padding="5")
        manual_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Crear botones en grid
        comandos = [
            ("HOME", self.cmd_home, "Ir a posición HOME"),
            ("READY", self.cmd_ready, "Preparar robot"),
            ("COFF", self.cmd_coff, "Apagar servos"),
            ("OPEN", self.cmd_open, "Abrir gripper"),
            ("CLOSE", self.cmd_close, "Cerrar gripper"),
            ("ABORT", self.cmd_abort, "Abortar movimiento"),
            ("← Base", self.cmd_left, "Base izquierda"),
            ("→ Base", self.cmd_right, "Base derecha"),
            ("RUN PCPLC", self.cmd_pcplc, "Ejecutar PCPLC"),
            ("RUN TTSIB", self.cmd_ttsib, "Ejecutar TTSIB"),
        ]
        
        for i, (texto, comando, tooltip) in enumerate(comandos):
            btn = ttk.Button(manual_frame, text=texto, command=comando)
            btn.grid(row=i//5, column=i%5, padx=3, pady=3, sticky="ew")
        
        # Comandos automáticos (de conect.py)
        auto_frame = ttk.LabelFrame(main_frame, text="Comandos Automáticos", padding="5")
        auto_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(auto_frame, text="Ejecutar Secuencia Automática", command=self.ejecutar_secuencia_automatica).pack(side=tk.LEFT, padx=5)
        ttk.Button(auto_frame, text="Escuchar CMG/CHG", command=self.toggle_escucha_automatica).pack(side=tk.LEFT, padx=5)
        
        self.lbl_escucha = ttk.Label(auto_frame, text="Escucha: OFF", foreground="red")
        self.lbl_escucha.pack(side=tk.LEFT, padx=10)
        
        # Log del robot
        log_frame = ttk.LabelFrame(main_frame, text="Log del Robot", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        self.robot_log = tk.Text(log_frame, height=12, state=tk.DISABLED)
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.robot_log.yview)
        self.robot_log.configure(yscrollcommand=scrollbar.set)
        
        self.robot_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Botón para cerrar
        ttk.Button(main_frame, text="Cerrar", command=self.cerrar_ventana).pack(pady=(10, 0))
    
    def log_robot(self, mensaje):
        """Añade mensaje al log del robot"""
        self.robot_log.config(state=tk.NORMAL)
        self.robot_log.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {mensaje}\n")
        self.robot_log.see(tk.END)
        self.robot_log.config(state=tk.DISABLED)
    
    def conectar_robot(self):
        """Conecta al robot usando configuración de comun.py"""
        try:
            if self.ser_robot and self.ser_robot.is_open:
                self.log_robot("Ya conectado al robot")
                return
                
            self.ser_robot = serial.Serial()
            self.ser_robot.baudrate = 9600
            self.ser_robot.bytesize = 8
            self.ser_robot.parity = "N"
            self.ser_robot.stopbits = serial.STOPBITS_ONE
            self.ser_robot.timeout = 1
            self.ser_robot.port = self.robot_port.get()
            
            self.ser_robot.open()
            self.robot_connected = True
            self.lbl_estado.config(text="CONECTADO", background="lime")
            self.log_robot(f"✅ Conectado al robot en {self.robot_port.get()}")
            
            # Configuración inicial
            self.send_command("JOINT", "Modo joint activado")
            self.send_command("READY", "Robot listo")
            self.send_command("SPEED 30", "Velocidad inicial 30%")
            
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo conectar al robot:\n{e}")
            self.log_robot(f"❌ Error de conexión: {e}")
            
    def desconectar_robot(self):
        """Desconecta del robot"""
        try:
            if self.escuchando_automatico:
                self.toggle_escucha_automatica()
                
            if self.ser_robot and self.ser_robot.is_open:
                self.ser_robot.close()
                self.robot_connected = False
                self.lbl_estado.config(text="DESCONECTADO", background="red")
                self.log_robot("🔌 Robot desconectado")
        except Exception as e:
            self.log_robot(f"Error al desconectar: {e}")
            
    def send_command(self, command, description=""):
        """Envía comando al robot usando lógica de comun.py"""
        if not self.ser_robot or not self.ser_robot.is_open:
            messagebox.showerror("Error", "Debe conectar el puerto primero")
            return False
            
        try:
            cmd = command.upper().strip() + '\r'
            self.ser_robot.reset_input_buffer()
            self.ser_robot.write(cmd.encode())
            time.sleep(0.5)
            
            response = self.ser_robot.read_all().decode('ascii', errors='ignore').strip()
            
            self.log_robot(f"🤖 {cmd.strip()}")
            if response:
                self.log_robot(f"📢 {response}")
            if description:
                self.log_robot(f"💬 {description}")
            self.log_robot("-" * 40)
            
            return True
        except Exception as e:
            messagebox.showerror("Error", f"Error al enviar comando: {e}")
            return False
    
    # Comandos individuales
    def cmd_home(self):
        self.send_command("HOME", "Mover a posición HOME")
    
    def cmd_ready(self):
        self.send_command("READY", "Preparar robot para movimiento")
    
    def cmd_coff(self):
        self.send_command("COFF", "Apagar servomotores")
    
    def cmd_open(self):
        self.send_command("OPEN", "Abrir gripper")
    
    def cmd_close(self):
        self.send_command("CLOSE", "Cerrar gripper")
    
    def cmd_abort(self):
        self.send_command("ABORT", "Movimiento abortado")
    
    def cmd_left(self):
        self.send_command("MJ 1 -10", "Base movida 10° a la izquierda")
    
    def cmd_right(self):
        self.send_command("MJ 1 10", "Base movida 10° a la derecha")
    
    def cmd_pcplc(self):
        self.send_command("RUN PCPLC", "Ejecutar programa PCPLC")
    
    def cmd_ttsib(self):
        self.send_command("RUN TTSIB", "Ejecutar programa TTSIB")
    
    def ejecutar_secuencia_automatica(self):
        """Ejecuta secuencia automática de conect.py"""
        if not self.ser_robot or not self.ser_robot.is_open:
            messagebox.showerror("Error", "Debe conectar el robot primero")
            return
            
        self.log_robot("🚀 Iniciando secuencia automática...")
        
        comandos = ["run gp", "run dp", "run initc", "run gpv", "run gf", "run initc"]
        
        for comando in comandos:
            try:
                cmd_upper = comando.upper() + '\r'
                self.ser_robot.write(cmd_upper.encode())
                self.log_robot(f"🔄 Enviando: {comando}")
                
                # Esperar respuesta
                time.sleep(2)
                response = self.ser_robot.read_all().decode('ascii', errors='ignore').strip()
                
                if response:
                    self.log_robot(f"📢 Respuesta: {response}")
                    
                # Verificar si completado
                if any(word in response.lower() for word in ["done", "ok", "complete"]):
                    self.log_robot(f"✅ {comando} completado")
                elif "error" in response.lower():
                    self.log_robot(f"❌ Error en {comando}: {response}")
                    break
                else:
                    self.log_robot(f"⚠️ Respuesta no esperada para {comando}")
                    
            except Exception as e:
                self.log_robot(f"❌ Error ejecutando {comando}: {e}")
                break
                
        self.log_robot("🏁 Secuencia automática finalizada")
    
    def toggle_escucha_automatica(self):
        """Activa/desactiva escucha automática para CMG/CHG"""
        if not self.escuchando_automatico:
            if not self.ser_robot or not self.ser_robot.is_open:
                messagebox.showerror("Error", "Debe conectar el robot primero")
                return
                
            self.escuchando_automatico = True
            self.lbl_escucha.config(text="Escucha: ON", foreground="green")
            self.hilo_escucha = threading.Thread(target=self.escuchar_cmg_chg)
            self.hilo_escucha.daemon = True
            self.hilo_escucha.start()
            self.log_robot("🔍 Escucha automática CMG/CHG activada")
        else:
            self.escuchando_automatico = False
            self.lbl_escucha.config(text="Escucha: OFF", foreground="red")
            self.log_robot("⏹️ Escucha automática desactivada")
    
    def escuchar_cmg_chg(self):
        """Escucha por mensajes CMG/CHG en hilo separado"""
        while self.escuchando_automatica and self.ser_robot and self.ser_robot.is_open:
            try:
                recibir = self.ser_robot.read_all()
                if recibir:
                    recibido = recibir.decode('utf-8', errors='ignore')
                    if "CMG" in recibido or "CHG" in recibido:
                        self.log_robot(f"🔍 Detectado: {recibido.strip()}")
                        self.log_robot("🚀 Ejecutando secuencia automática...")
                        self.ejecutar_secuencia_automatica()
                        break
                time.sleep(1)
            except Exception as e:
                self.log_robot(f"❌ Error en escucha: {e}")
                break
    
    def cerrar_ventana(self):
        """Cerrar ventana del robot"""
        self.desconectar_robot()
        self.ventana.destroy()

# =============================================================================
# CLASE VENTANA SCORBOT ORIGINAL - INTERFAZ DE COMUN.PY
# =============================================================================

class VentanaScorbotOriginal:
    def __init__(self, parent):
        self.parent = parent
        self.ventana = tk.Toplevel(parent)
        self.ventana.title("Controlador Scorbot-ER V Plus")
        self.ventana.geometry("700x550")
        self.ventana.resizable(False, False)
        
        # Crear objeto Serial
        self.SerialPort1 = serial.Serial()
        
        self.crear_interfaz_scorbot()
        
    def crear_interfaz_scorbot(self):
        # Configuración de estilo
        style = ttk.Style()
        style.configure('TButton', font=('Arial', 9), padding=5)
        style.configure('TLabel', font=('Arial', 9))

        # Panel de estado
        frame_estado = ttk.Frame(self.ventana, padding=10)
        frame_estado.pack(fill=tk.X)

        self.TextoEstado = tk.Text(frame_estado, height=1, width=15, state="disabled", 
                             bg="red", font=("Arial", 10, "bold"))
        self.TextoEstado.insert("1.0", "DESCONECTADO")
        self.TextoEstado.pack(side=tk.LEFT, padx=5)

        ttk.Button(frame_estado, text="Conectar", command=self.click_conectar).pack(side=tk.LEFT, padx=5)
        ttk.Button(frame_estado, text="Desconectar", command=self.click_desconectar).pack(side=tk.LEFT, padx=5)

        self.comboBox1 = ttk.Combobox(frame_estado, state="readonly", width=25)
        self.comboBox1.pack(side=tk.LEFT, padx=5)
        self.update_com_ports()

        # Panel de control principal
        main_frame = ttk.Frame(self.ventana, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Panel de comandos rápidos
        cmd_frame = ttk.LabelFrame(main_frame, text="Comandos Rápidos", padding=10)
        cmd_frame.grid(row=0, column=0, padx=5, pady=5, sticky=tk.N)

        buttons = [
            ("Home", self.click_home),
            ("Ready", self.click_ready),
            ("Run PCPLC", self.click_pcplc),
            ("Run TTSIB", self.click_ttsib),
            ("Abortar", self.click_a),
            ("Apagar (COFF)", self.click_coff),
            ("Mover a 0", self.click_move),
            ("Abrir Gripper", self.click_open),
            ("Cerrar Gripper", self.click_close),
            ("Izquierda (-10°)", self.click_left),
            ("Derecha (+10°)", self.click_right),
            ("Velocidad 50%", self.click_speed)
        ]

        for text, cmd in buttons:
            ttk.Button(cmd_frame, text=text, command=cmd).pack(fill=tk.X, pady=2)

        # Panel de comunicación
        comm_frame = ttk.LabelFrame(main_frame, text="Comunicación", padding=10)
        comm_frame.grid(row=0, column=1, padx=5, pady=5, sticky=tk.NSEW)

        # Área de envío
        ttk.Label(comm_frame, text="Comando personalizado:").pack(anchor=tk.W)
        self.TextEnviar = tk.Text(comm_frame, height=3, width=30, font=('Consolas', 10))
        self.TextEnviar.pack(fill=tk.X, pady=5)

        ttk.Button(comm_frame, text="Enviar Comando", command=self.click_enviar).pack(fill=tk.X, pady=5)

        # Área de recepción
        ttk.Label(comm_frame, text="Respuesta del robot:").pack(anchor=tk.W)
        self.TextRecibidos = tk.Text(comm_frame, height=15, width=50, font=('Consolas', 9))
        self.TextRecibidos.pack(fill=tk.BOTH, expand=True)

        # Botón Guardar
        ttk.Button(comm_frame, text="Guardar Log", command=self.click_guardar).pack(fill=tk.X, pady=5)
        
        # Botón Cerrar
        ttk.Button(self.ventana, text="Cerrar", command=self.cerrar_ventana).pack(pady=10)

    def send_scorbot_command(self, command, description=""):
        """Función mejorada para enviar comandos al Scorbot-ER V Plus"""
        if self.SerialPort1.is_open:
            try:
                cmd = command.upper().strip() + '\r'
                self.SerialPort1.reset_input_buffer()
                self.SerialPort1.write(cmd.encode())
                time.sleep(0.5)
                
                response = self.SerialPort1.read_all().decode('ascii', errors='ignore').strip()
                
                self.TextRecibidos.insert("1.0", f"> {cmd.strip()}\n")
                self.TextRecibidos.insert("2.0", f"< {response}\n")
                if description:
                    self.TextRecibidos.insert("3.0", f"{description}\n")
                self.TextRecibidos.insert("4.0", "-"*50 + "\n")
                
                return True
            except Exception as e:
                messagebox.showerror("Error", f"Error al enviar comando: {str(e)}")
                return False
        else:
            messagebox.showerror("Error", "Debe conectar el puerto primero")
            return False

    def click_pcplc(self):
        self.send_scorbot_command("RUN PCPLC", "Ejecutar programa PCPLC")

    def click_a(self):
        self.send_scorbot_command("ABORT", "Movimiento abortado")

    def click_ttsib(self):
        self.send_scorbot_command("RUN TTSIB", "Ejecutar programa TTSIB")

    def click_coff(self):
        self.send_scorbot_command("COFF", "Apagar servomotores")

    def click_move(self):
        self.send_scorbot_command("MOVE 0", "Mover a posición 0")

    def click_open(self):
        self.send_scorbot_command("OPEN", "Abrir gripper")

    def click_close(self):
        self.send_scorbot_command("CLOSE", "Cerrar gripper")

    def click_left(self):
        self.send_scorbot_command("MJ 1 -10", "Base movida 10° a la izquierda")

    def click_right(self):
        self.send_scorbot_command("MJ 1 10", "Base movida 10° a la derecha")

    def click_home(self):
        self.send_scorbot_command("HOME", "Mover a posición HOME")

    def click_ready(self):
        self.send_scorbot_command("READY", "Preparar robot para movimiento")

    def click_speed(self):
        self.send_scorbot_command("SPEED 50", "Velocidad establecida al 50%")

    def click_conectar(self):
        if not self.SerialPort1.is_open:
            self.SerialPort1.baudrate = 9600
            self.SerialPort1.bytesize = 8
            self.SerialPort1.parity = "N"
            self.SerialPort1.stopbits = serial.STOPBITS_ONE
            self.SerialPort1.timeout = 1
            self.SerialPort1.port = self.comboBox1.get()
            try:
                self.SerialPort1.open()
                self.TextoEstado.config(state="normal")
                self.TextoEstado.delete("1.0", tk.END)
                self.TextoEstado.insert("1.0", "CONECTADO")
                self.TextoEstado.configure(background="lime")
                
                self.send_scorbot_command("JOINT", "Modo joint activado")
                self.send_scorbot_command("READY", "Robot listo para comandos")
                self.send_scorbot_command("SPEED 30", "Velocidad inicial al 30%")
                
                messagebox.showinfo("Conectado", "Conexión establecida con Scorbot-ER V Plus")
                self.TextoEstado.config(state="disabled")
            except Exception as e:
                messagebox.showerror("Error de conexión", f"No se pudo conectar: {str(e)}")
        else:
            messagebox.showinfo("Información", "El puerto ya está conectado.")

    def click_desconectar(self):
        if self.SerialPort1.is_open:
            try:
                self.send_scorbot_command("COFF", "Apagar servomotores antes de desconectar")
                self.SerialPort1.close()
                self.TextoEstado.config(state="normal")
                self.TextoEstado.delete("1.0", tk.END)
                self.TextoEstado.insert("1.0", "DESCONECTADO")
                self.TextoEstado.configure(background="red")
                messagebox.showinfo("Desconectado", "Puerto serial desconectado")
                self.TextoEstado.config(state="disabled")
            except Exception as e:
                messagebox.showerror("Error", f"Error al desconectar: {str(e)}")
        else:
            messagebox.showinfo("Información", "El puerto ya está desconectado")

    def click_enviar(self):
        custom_cmd = self.TextEnviar.get("1.0", tk.END).strip()
        if custom_cmd:
            self.send_scorbot_command(custom_cmd, "Comando personalizado enviado")
        else:
            messagebox.showwarning("Advertencia", "Escriba un comando primero")

    def click_guardar(self):
        from tkinter.filedialog import asksaveasfilename
        filepath = asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
        )
        if not filepath:
            return
        with open(filepath, "w") as output_file:
            text = self.TextRecibidos.get("1.0", tk.END)
            output_file.write(text)

    def update_com_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()] if serial else ["COM1", "COM2", "COM3"]
        self.comboBox1['values'] = ports
        if ports:
            self.comboBox1.set(ports[0])
        else:
            self.comboBox1.set("No ports found")
            
    def cerrar_ventana(self):
        """Cerrar ventana del scorbot original"""
        self.click_desconectar()
        self.ventana.destroy()

# =============================================================================
# CLASE VENTANA COMUNICACION SERIAL ORIGINAL - INTERFAZ DE CONECT.PY
# =============================================================================

class VentanaComunicacionSerial:
    def __init__(self, parent):
        self.parent = parent
        self.ventana = tk.Toplevel(parent)
        self.ventana.title("Comunicacion Serial")
        self.ventana.geometry("600x400")
        
        # Crear objeto Serial
        self.SerialPort1 = serial.Serial()
        self.escuchando = False
        self.hilo_escucha = None
        
        self.crear_interfaz_comunicacion()
        
    def crear_interfaz_comunicacion(self):
        # Etiqueta de estado (DESCONECTADO)
        self.TextoEstado = tk.Text(self.ventana, height=1, width=15, state="disabled", bg="red", font=("Arial", 10, "bold"))
        self.TextoEstado.insert("1.0", "DESCONECTADO")
        self.TextoEstado.place(x=240, y=10)

        # Botones Conectar y Desconectar
        ttk.Button(self.ventana, text="Conectar", command=self.click_conectar).place(x=10, y=40, width=100)
        ttk.Button(self.ventana, text="Desconectar", command=self.click_desconectar).place(x=390, y=40, width=100)

        # ComboBox de puertos
        self.comboBox1 = ttk.Combobox(self.ventana, state="readonly")
        self.comboBox1.place(x=120, y=40, width=260)
        self.update_com_ports()

        # Etiqueta "Datos Enviados" y campo de envío
        tk.Label(self.ventana, text="Datos Enviados").place(x=200, y=80)
        self.TextEnviar = tk.Text(self.ventana, height=3, width=30)
        self.TextEnviar.place(x=150, y=100)

        # Etiqueta "Datos Recibidos" y campo de recepción
        tk.Label(self.ventana, text="Datos Recibidos").place(x=430, y=80)
        self.TextRecibidos = tk.Text(self.ventana, height=10, width=30)
        self.TextRecibidos.place(x=350, y=100)

        # Botón Enviar
        ttk.Button(self.ventana, text="Enviar", command=self.click_enviar).place(x=220, y=160, width=80)
        
        # Botón Cerrar
        ttk.Button(self.ventana, text="Cerrar", command=self.cerrar_ventana).place(x=270, y=350, width=80)

    def escuchar_automatica(self):
        """Función de escucha automática en hilo separado"""
        while self.escuchando and self.SerialPort1.is_open:
            try:
                recibir = self.SerialPort1.read_all()
                if recibir:
                    recibir_str = recibir.decode('utf-8', errors='ignore')
                    if "CMG" in recibir_str or "CHG" in recibir_str:
                        self.ventana.after(0, lambda: self.TextRecibidos.insert(tk.END, f"Detectado: {recibir_str}\n"))
                        self.enviar_comandos_automatically()
                        break
                time.sleep(1)
            except Exception as e:
                self.ventana.after(0, lambda: self.TextRecibidos.insert(tk.END, f"Error en escucha: {e}\n"))
                break

    def enviar_comandos_automatically(self):
        """Envía comandos automáticamente cuando detecta CMG/CHG"""
        if self.SerialPort1.is_open:
            comandos = [b"run gp", b"run dp", b"run initc", b"run gpv", b"run gf", b"run initc"]

            for comando in comandos:
                try:
                    self.SerialPort1.write(comando + b"\r")
                    self.ventana.after(0, lambda cmd=comando: self.TextRecibidos.insert(tk.END, f"Comando Enviado: {cmd.decode()}\n"))

                    respuesta = self.leer_respuesta()
                    
                    if "done" in respuesta.lower() or "ok" in respuesta.lower() or "complete" in respuesta.lower():
                        self.ventana.after(0, lambda cmd=comando: self.TextRecibidos.insert(tk.END, f"Comando {cmd.decode()} completado.\n"))
                    else:
                        if "error" in respuesta.lower():
                            self.ventana.after(0, lambda cmd=comando, resp=respuesta: self.TextRecibidos.insert(tk.END, f"Error al ejecutar {cmd.decode()}.\nRespuesta: {resp}\n"))
                            break
                        else:
                            self.ventana.after(0, lambda cmd=comando, resp=respuesta: self.TextRecibidos.insert(tk.END, f"Respuesta no esperada para {cmd.decode()}: {resp}\n"))

                    if comando == b"run dp":
                        time.sleep(15)
                    elif comando == b"run gpv":
                        respuesta = self.leer_respuesta()
                        if "done" in respuesta.lower() or "ok2" in respuesta.lower() or "complete" in respuesta.lower():
                            self.ventana.after(0, lambda: self.TextRecibidos.insert(tk.END, "Comando gpv completado.\n"))
                        else:
                            self.ventana.after(0, lambda resp=respuesta: self.TextRecibidos.insert(tk.END, f"Respuesta no esperada para gpv: {resp}\n"))
                    elif comando == b"run gf":
                        respuesta = self.leer_respuesta()
                        if "done" in respuesta.lower() or "ok3" in respuesta.lower() or "complete" in respuesta.lower():
                            self.ventana.after(0, lambda: self.TextRecibidos.insert(tk.END, "Comando gf enviado correctamente.\n"))
                        else:
                            self.ventana.after(0, lambda: self.TextRecibidos.insert(tk.END, "Error al ejecutar gf. El robot no está listo.\n"))
                            break
                    else:
                        time.sleep(1)
                        
                except Exception as e:
                    self.ventana.after(0, lambda cmd=comando, err=e: self.TextRecibidos.insert(tk.END, f"Error enviando {cmd.decode()}: {err}\n"))
                    break

            self.ventana.after(0, lambda: self.TextRecibidos.insert(tk.END, "\nComandos enviados correctamente.\n"))

    def leer_respuesta(self):
        """Lee la respuesta del puerto serial hasta encontrar un salto de línea o timeout."""
        response = b""
        start_time = time.time()

        while time.time() - start_time < 30:
            if self.SerialPort1.in_waiting > 0:
                response += self.SerialPort1.read(self.SerialPort1.in_waiting)
            if b"Done." in response or b"done" in response or b"ok" in response or b"complete" in response:
                break
            time.sleep(0.1)

        respuesta_str = response.decode('utf-8', errors='ignore')
        return respuesta_str

    def click_conectar(self):
        if not self.SerialPort1.is_open:
            self.SerialPort1.baudrate = 9600
            self.SerialPort1.bytesize = 8
            self.SerialPort1.parity = "N"
            self.SerialPort1.stopbits = serial.STOPBITS_ONE
            self.SerialPort1.port = self.comboBox1.get()
            try:
                self.SerialPort1.open()
                self.TextoEstado.config(state="normal")
                self.TextoEstado.delete("1.0", tk.END)
                self.TextoEstado.insert("1.0", "CONECTADO")
                self.TextoEstado.configure(background="lime")
                messagebox.showinfo(message="Puerto Conectado")
                self.TextoEstado.config(state="disabled")
                
                # Iniciar el proceso de escucha automática en un hilo separado
                self.escuchando = True
                self.hilo_escucha = threading.Thread(target=self.escuchar_automatica, daemon=True)
                self.hilo_escucha.start()
            except Exception as e:
                messagebox.showerror("Error de conexión", str(e))
        else:
            messagebox.showinfo("Información", "El puerto ya está conectado.")

    def click_desconectar(self):
        if self.SerialPort1.is_open:
            self.escuchando = False
            self.SerialPort1.close()
            self.TextoEstado.config(state="normal")
            self.TextoEstado.delete("1.0", tk.END)
            self.TextoEstado.insert("1.0", "DESCONECTADO")
            self.TextoEstado.configure(background="red")
            messagebox.showinfo(message="Puerto Desconectado")
            self.TextoEstado.config(state="disabled")
        else:
            messagebox.showinfo("Información", "El puerto ya está desconectado")

    def click_enviar(self):
        if self.SerialPort1.is_open:
            data_to_send = self.TextEnviar.get("1.0", tk.END).strip()
            if data_to_send:
                self.SerialPort1.write(data_to_send.encode() + b"\r")
                time.sleep(2)
                aux = self.SerialPort1.read_all()
                if b"Done." in aux:
                    time.sleep(42)
                    self.TextRecibidos.insert(tk.END, b"OK\n")
                messagebox.showinfo(message="Enviado Correctamente", title="Resultado")
            else:
                messagebox.showwarning("Advertencia", "El campo de envío está vacío.")
        else:
            messagebox.showerror("Error", "Debe conectar el puerto primero")

    def update_com_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()] if serial else ["COM1", "COM2", "COM3"]
        self.comboBox1['values'] = ports
        if ports:
            self.comboBox1.set(ports[0])
        else:
            self.comboBox1.set("No ports found")
            
    def cerrar_ventana(self):
        """Cerrar ventana de comunicación serial"""
        self.click_desconectar()
        self.ventana.destroy()

# =============================================================================
# CLASE APLICACIÓN PRINCIPAL - INTERFAZ GRÁFICA PRINCIPAL
# =============================================================================

class Aplicacion:
    def __init__(self, ventana):
        self.ventana = ventana
        self.ventana.title("Sistema de Medición con ArUco")
        
        # Variables de control
        self.capturando = False
        self.camara = None
        
        # Variables para ajustes de imagen
        self.brillo = 100
        self.contraste = 100
        self.zoom = 100
        
        # Variables para ordenamiento
        self.modo_ordenamiento = "normal"
        self.objetos_detectados = []
        
        # Ventana del láser
        self.ventana_laser = None
        
        # Ventana del robot (NO se abre automáticamente - solo cuando se presiona el botón)
        self.ventana_robot = None
        
        # Ventanas originales del robot (NO se abren automáticamente - solo cuando se presiona el botón)
        self.ventana_scorbot_original = None
        self.ventana_comunicacion_serial = None
        
        # Información del ArUco generado
        self.aruco_info = None

        # Crear la interfaz principal SIEMPRE visible
        self.crear_interfaz()

        # Deshabilitar controles al inicio (excepto "Crear Aruco")
        self.deshabilitar_controles()

        # Forzar creación de ArUco antes de habilitar controles
        self.forzar_aruco_al_inicio()

    def deshabilitar_controles(self):
        """Deshabilita todos los controles excepto 'Crear ArUco'"""
        for child in self.ventana.winfo_children():
            self._set_estado_recursivo(child, tk.DISABLED)
        # Habilita solo el botón 'Crear ArUco'
        if hasattr(self, 'btn_crear_aruco'):
            self.btn_crear_aruco.config(state=tk.NORMAL)

    def habilitar_controles(self):
        """Habilita todos los controles"""
        for child in self.ventana.winfo_children():
            self._set_estado_recursivo(child, tk.NORMAL)

    def _set_estado_recursivo(self, widget, estado):
        try:
            widget.config(state=estado)
        except Exception:
            pass
        for child in widget.winfo_children():
            self._set_estado_recursivo(child, estado)

    def forzar_aruco_al_inicio(self):
        """Obliga a crear un ArUco antes de usar la app"""
        from aruco_generador import crear_aruco
        while True:
            resultado = crear_aruco(self.ventana)
            if resultado is not None and resultado.get("id") is not None:
                self.aruco_info = resultado
                messagebox.showinfo("ArUco generado", f"Diccionario: {resultado['diccionario']}\nTamaño: {resultado['tamano']}\nID: {resultado['id']}")
                self.habilitar_controles()
                break
            else:
                if not messagebox.askretrycancel("ArUco requerido", "Debes crear y aceptar un ArUco para continuar.\n¿Intentar de nuevo?"):
                    self.ventana.destroy()
                    return

    def abrir_ventana_crear_aruco(self):
        """Permite modificar el ArUco en cualquier momento"""
        try:
            from aruco_generador import crear_aruco
            resultado = crear_aruco(self.ventana)
            if resultado is not None and resultado.get("id") is not None:
                self.aruco_info = resultado
                messagebox.showinfo("ArUco generado", f"Diccionario: {resultado['diccionario']}\nTamaño: {resultado['tamano']}\nID: {resultado['id']}")
            else:
                messagebox.showwarning("Advertencia", "No se modificó el ArUco actual.")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo abrir el generador de ArUco:\n{e}")

    # =========================================================================
    # MÉTODOS DE CREACIÓN DE INTERFAZ - APLICACIÓN PRINCIPAL
    # =========================================================================
        
    def crear_interfaz(self):
        # Frame principal
        frame_principal = ttk.Frame(self.ventana, padding="10")
        frame_principal.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Frame para seleccionar tamaño de ArUco
        frame_diccionario = ttk.LabelFrame(frame_principal, text="Tamaño de ArUco", padding="5")
        frame_diccionario.grid(row=5, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))

        self.var_diccionario = tk.StringVar(value="DICT_4X4_100")
        opciones_diccionario = [
            ("3x3", "DICT_ARUCO_ORIGINAL"),
            ("4x4", "DICT_4X4_100"),
            ("5x5", "DICT_5X5_100"),
            ("6x6", "DICT_6X6_100"),
            ("7x7", "DICT_7X7_100")
        ]
        for i, (texto, valor) in enumerate(opciones_diccionario):
            ttk.Radiobutton(frame_diccionario, text=texto, variable=self.var_diccionario, value=valor, command=self.cambiar_diccionario).grid(row=0, column=i, padx=5, pady=5)

    # Frame para botones de control
        frame_controles = ttk.LabelFrame(frame_principal, text="Controles", padding="5")
        frame_controles.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.N))

    # Botones apilados verticalmente a la izquierda
        btn_inicio = ttk.Button(frame_controles, text="Inicio", command=self.boton_inicio_no_funciona)
        btn_inicio.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        btn_ordenar_mayor = ttk.Button(frame_controles, text="Ordenar Mayor", command=self.boton_ordenar_mayor)
        btn_ordenar_mayor.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        btn_ordenar_menor = ttk.Button(frame_controles, text="Ordenar Menor", command=self.boton_ordenar_menor)
        btn_ordenar_menor.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        btn_laser = ttk.Button(frame_controles, text="Control Láser", command=self.abrir_ventana_laser)
        btn_laser.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        btn_robot = ttk.Button(frame_controles, text="Control Robot", command=self.abrir_ventana_robot)
        btn_robot.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        btn_robot_original = ttk.Button(frame_controles, text="Ventanas Originales Robot", command=self.abrir_ventanas_robot_originales)
        btn_robot_original.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        self.btn_crear_aruco = ttk.Button(frame_controles, text="Crear Aruco", command=self.abrir_ventana_crear_aruco)
        self.btn_crear_aruco.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

    # Label para mostrar el modo actual
        self.lbl_modo = ttk.Label(frame_controles, text="Modo: NORMAL", 
             font=("Arial", 10, "bold"), foreground="blue")
        self.lbl_modo.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

    # Frame para ajustes de imagen
        frame_ajustes = ttk.LabelFrame(frame_principal, text="Ajustes de Imagen", padding="5")
        frame_ajustes.grid(row=1, column=0, padx=5, pady=5, sticky=(tk.W, tk.N))

    # Controles apilados verticalmente a la izquierda
        lbl_brillo = ttk.Label(frame_ajustes, text="Brillo:")
        lbl_brillo.pack(side=tk.TOP, anchor="w", padx=5, pady=2)
        self.scale_brillo = Scale(frame_ajustes, from_=0, to=200, orient=tk.HORIZONTAL, command=self.ajustar_brillo)
        self.scale_brillo.set(self.brillo)
        self.scale_brillo.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        lbl_contraste = ttk.Label(frame_ajustes, text="Contraste:")
        lbl_contraste.pack(side=tk.TOP, anchor="w", padx=5, pady=2)
        self.scale_contraste = Scale(frame_ajustes, from_=0, to=200, orient=tk.HORIZONTAL, command=self.ajustar_contraste)
        self.scale_contraste.set(self.contraste)
        self.scale_contraste.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        lbl_zoom = ttk.Label(frame_ajustes, text="Zoom:")
        lbl_zoom.pack(side=tk.TOP, anchor="w", padx=5, pady=2)
        self.scale_zoom = Scale(frame_ajustes, from_=100, to=200, orient=tk.HORIZONTAL, command=self.ajustar_zoom)
        self.scale_zoom.set(self.zoom)
        self.scale_zoom.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        btn_reset = ttk.Button(frame_ajustes, text="Resetear Ajustes", command=self.resetear_ajustes)
        btn_reset.pack(side=tk.TOP, anchor="w", fill=tk.X, padx=5, pady=2)

        # Frame para video

        frame_video = ttk.LabelFrame(frame_principal, text="Vista de Cámara", padding="5")
        frame_video.grid(row=3, column=0, padx=5, pady=5, sticky="n")

    # Botón para iniciar la cámara
        self.btn_iniciar_camara = ttk.Button(frame_video, text="Iniciar cámara", command=self.iniciar_camara)
        self.btn_iniciar_camara.pack(side=tk.TOP, anchor="center", pady=5)

    # Label para mostrar video centrado
        self.lbl_video = ttk.Label(frame_video, text="Cámara no iniciada")
        self.lbl_video.pack(anchor="center", pady=5)

    # Frame para máscara
        frame_mascara = ttk.LabelFrame(frame_principal, text="Vista de Máscara", padding="5")
        frame_mascara.grid(row=4, column=0, padx=5, pady=5, sticky="n")

    # Label para mostrar máscara centrada
        self.lbl_mascara = ttk.Label(frame_mascara, text="Máscara no disponible")
        self.lbl_mascara.pack(anchor="center", pady=5)

        # Configurar expansión
        self.ventana.columnconfigure(0, weight=1)
        self.ventana.rowconfigure(0, weight=1)
        frame_principal.columnconfigure(0, weight=1)
        frame_principal.rowconfigure(3, weight=1)
        frame_principal.rowconfigure(4, weight=1)
        frame_video.columnconfigure(0, weight=1)
        frame_video.rowconfigure(0, weight=1)
        frame_mascara.columnconfigure(0, weight=1)
        frame_mascara.rowconfigure(0, weight=1)
        frame_ajustes.columnconfigure(1, weight=1)

    # No iniciar la cámara automáticamente
    
    # =========================================================================
    # MÉTODOS DE FUNCIONALIDAD - APLICACIÓN PRINCIPAL
    # =========================================================================
    
    def cambiar_diccionario(self):
        global diccionario_aruco, proporcion_cm_por_pixel, detector_aruco, usar_nueva_api
        seleccion = self.var_diccionario.get()
        diccionario_aruco = getattr(aruco, f'getPredefinedDictionary')(getattr(aruco, seleccion))
        proporcion_cm_por_pixel = None  # Reiniciar proporción para forzar nueva medición
        
        # Actualizar el detector ArUco
        usar_nueva_api = crear_detector_aruco()
        
        # Reiniciar la cámara para que detecte el nuevo ArUco y actualice la proporción
        if self.camara:
            self.capturando = False
            # Esperar un poco para que el hilo termine
            threading.Event().wait(0.1)
            self.camara.release()
            
        self.iniciar_camara()
        
    # =========================================================================
    # MÉTODOS DE CONTROL DE MODOS Y ORDENAMIENTO
    # =========================================================================
        
    def boton_inicio_no_funciona(self):
        self.modo_ordenamiento = "normal"
        self.objetos_detectados = []
        self.lbl_modo.config(text="Modo: NORMAL", foreground="blue")
        print("Modo normal activado - Sin ordenamiento")
        
    def boton_ordenar_mayor(self):
        self.modo_ordenamiento = "mayor"
        self.lbl_modo.config(text="Modo: ORDENAR MAYOR (ID)", foreground="red")
        print("Modo ordenamiento activado - Mayor a menor por ID")
        
    def boton_ordenar_menor(self):
        self.modo_ordenamiento = "menor"
        self.lbl_modo.config(text="Modo: ORDENAR MENOR (ID)", foreground="green")
        print("Modo ordenamiento activado - Menor a mayor por ID")
        
    def abrir_ventana_laser(self):
        """Abre la ventana de control del láser"""
        if self.ventana_laser is None or not self.ventana_laser.ventana.winfo_exists():
            try:
                self.ventana_laser = VentanaLaser(self.ventana, self.aruco_info)
                print("Ventana del láser abierta")
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo abrir la ventana del láser:\n{e}")
        else:
            # Si ya existe, la trae al frente
            self.ventana_laser.ventana.lift()
            self.ventana_laser.ventana.focus_force()

    def abrir_ventana_robot(self):
        """Abre la ventana de control del robot SOLO cuando se presiona el botón"""
        if self.ventana_robot is None or not self.ventana_robot.ventana.winfo_exists():
            try:
                # Solo crear la ventana cuando el usuario presiona el botón
                self.ventana_robot = VentanaRobot(self.ventana)
                print("Ventana del robot abierta por solicitud del usuario")
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo abrir la ventana del robot:\n{e}")
        else:
            # Si ya existe, la trae al frente
            self.ventana_robot.ventana.lift()
            self.ventana_robot.ventana.focus_force()

    def abrir_ventanas_robot_originales(self):
        """Abre las ventanas originales del robot SOLO cuando se presiona el botón"""
        try:
            # Solo crear las ventanas cuando el usuario presiona el botón
            # Abrir ventana de control Scorbot original
            if self.ventana_scorbot_original is None or not self.ventana_scorbot_original.ventana.winfo_exists():
                self.ventana_scorbot_original = VentanaScorbotOriginal(self.ventana)
                print("Ventana Scorbot original abierta por solicitud del usuario")
            else:
                self.ventana_scorbot_original.ventana.lift()
                self.ventana_scorbot_original.ventana.focus_force()
            
            # Abrir ventana de comunicación serial original
            if self.ventana_comunicacion_serial is None or not self.ventana_comunicacion_serial.ventana.winfo_exists():
                self.ventana_comunicacion_serial = VentanaComunicacionSerial(self.ventana)
                print("Ventana comunicación serial original abierta por solicitud del usuario")
            else:
                self.ventana_comunicacion_serial.ventana.lift()
                self.ventana_comunicacion_serial.ventana.focus_force()
                
        except Exception as e:
            messagebox.showerror("Error", f"No se pudieron abrir las ventanas originales del robot:\n{e}")
        
#    def boton_ordenar_azar(self):
#       self.modo_ordenamiento = "azar"
#        self.lbl_modo.config(text="Modo: ORDENAR AL AZAR", foreground="purple")
#        print("Modo ordenamiento activado - Al azar")

    # =========================================================================
    # MÉTODOS DE AJUSTES DE IMAGEN
    # =========================================================================
        
    def ajustar_brillo(self, valor):
        self.brillo = int(valor)
        
    def ajustar_contraste(self, valor):
        self.contraste = int(valor)
        
    def ajustar_zoom(self, valor):
        self.zoom = int(valor)
        
    def resetear_ajustes(self):
        self.brillo = 100
        self.contraste = 100
        self.zoom = 100
        self.scale_brillo.set(self.brillo)
        self.scale_contraste.set(self.contraste)
        self.scale_zoom.set(self.zoom)
        
    # =========================================================================
    # MÉTODOS DE PROCESAMIENTO DE IMAGEN Y CÁMARA
    # =========================================================================
        
    def aplicar_ajustes_imagen(self, imagen):
        # Aplicar brillo y contraste
        alpha = self.contraste / 100.0  # Factor de contraste
        beta = self.brillo - 100  # Factor de brillo
        
        imagen_ajustada = cv2.convertScaleAbs(imagen, alpha=alpha, beta=beta)
        
        # Aplicar zoom si es necesario
        if self.zoom > 100:
            factor_zoom = self.zoom / 100.0
            h, w = imagen_ajustada.shape[:2]
            new_h, new_w = int(h * factor_zoom), int(w * factor_zoom)
            
            # Redimensionar la imagen
            imagen_zoom = cv2.resize(imagen_ajustada, (new_w, new_h))
            # Recorte más arriba y más ancho
            ancho_recorte = int(w * 1.2) if int(w * 1.2) <= new_w else new_w
            start_x = max((new_w - ancho_recorte) // 2, 0)
            start_y = max((new_h - h) // 6, 0)  # Más arriba aún
            imagen_ajustada = imagen_zoom[start_y:start_y+h, start_x:start_x+ancho_recorte]
        
        return imagen_ajustada
        
    def iniciar_camara(self):
        # Probar varios índices de cámara
        camara_abierta = False
        for idx in range(3):
            self.camara = cv2.VideoCapture(idx)
            if self.camara.isOpened():
                camara_abierta = True
                break
            else:
                self.camara.release()
        if not camara_abierta:
            error_msg = "No se pudo abrir ninguna cámara (índices 0, 1, 2)"
            print(error_msg)
            self.lbl_video.config(text=error_msg)
            return

        # Configurar propiedades de la cámara
        self.camara.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
        self.camara.set(cv2.CAP_PROP_FRAME_HEIGHT, 2580)
        self.camara.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        print("Cámara abierta exitosamente")
        self.capturando = True
        self.hilo_camara = threading.Thread(target=self.actualizar_video)
        self.hilo_camara.daemon = True
        self.hilo_camara.start()
    
    def actualizar_video(self):
        global proporcion_cm_por_pixel
        
        # Crear ventana de cámara con tamaño fijo pequeño y no redimensionable
        cv2.namedWindow("Vista de Cámara", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vista de Cámara", 480, 360)  # Ventana pequeña de 480x360
        cv2.setWindowProperty("Vista de Cámara", cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_KEEPRATIO)
        
        while self.capturando:
            if not self.camara or not self.camara.isOpened():
                break

            exito, imagen = self.camara.read()
            if not exito or imagen is None:
                threading.Event().wait(0.01)
                continue

            # Procesamiento igual que antes
            imagen = self.aplicar_ajustes_imagen(imagen)
            hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
            mascara_rojo1 = cv2.inRange(hsv, rojo_bajo1, rojo_alto1)
            mascara_rojo2 = cv2.inRange(hsv, rojo_bajo2, rojo_alto2)
            mascara_rojo = cv2.bitwise_or(mascara_rojo1, mascara_rojo2)
            mascara_verde = cv2.inRange(hsv, verde_bajo, verde_alto)
            mascara_azul = cv2.inRange(hsv, azul_bajo, azul_alto)
            mascara_colores = cv2.bitwise_or(mascara_rojo, mascara_verde)
            mascara_colores = cv2.bitwise_or(mascara_colores, mascara_azul)
            kernel = np.ones((5, 5), np.uint8)
            mascara_colores = cv2.morphologyEx(mascara_colores, cv2.MORPH_CLOSE, kernel)
            mascara_colores = cv2.morphologyEx(mascara_colores, cv2.MORPH_OPEN, kernel)
            contornos, _ = cv2.findContours(mascara_colores, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Detección ArUco igual que antes
            if usar_nueva_api and detector_aruco is not None:
                esquinas, ids, rechazados = detector_aruco.detectMarkers(imagen)
            else:
                try:
                    esquinas, ids, rechazados = aruco.detectMarkers(imagen, diccionario_aruco, parameters=parametros_aruco)
                except AttributeError:
                    esquinas, ids, rechazados = [], None, []
                    cv2.putText(imagen, "Error: ArUco no compatible", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if ids is not None:
                # Procesar hasta 5 ArUcos detectados en simultáneo, sin filtrar por ID
                max_arucos = 5
                total_detectados = min(len(esquinas), max_arucos)
                arucos_info = []
                for i in range(total_detectados):
                    esquina = esquinas[i]
                    id_val = ids.flatten()[i]
                    pts = esquina[0]
                    x_min = int(np.min(pts[:, 0]))
                    x_max = int(np.max(pts[:, 0]))
                    y_min = int(np.min(pts[:, 1]))
                    y_max = int(np.max(pts[:, 1]))
                    ancho_pix = x_max - x_min
                    alto_pix = y_max - y_min
                    proporcion_cm_por_pixel = tamano_aruco_cm / ancho_pix if ancho_pix > 0 else 0
                    texto_ancho = f"Ancho: {ancho_pix * proporcion_cm_por_pixel:.2f} cm"
                    texto_alto = f"Alto: {alto_pix * proporcion_cm_por_pixel:.2f} cm"
                    cv2.putText(imagen, texto_ancho, (x_min, y_min - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(imagen, texto_alto, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    dx = pts[1][0] - pts[0][0]
                    dy = pts[1][1] - pts[0][1]
                    angulo_rad = np.arctan2(dy, dx)
                    angulo_deg = np.degrees(angulo_rad)
                    centro = np.mean(pts, axis=0).astype(int)
                    arucos_info.append({
                        'esquina': esquina,
                        'id': id_val,
                        'angulo': angulo_deg,
                        'centro': centro
                    })
                # Dibujar todos los ArUcos detectados en simultáneo
                imagen = aruco.drawDetectedMarkers(imagen, esquinas[:total_detectados], ids[:total_detectados])
                for orden, aruco_info in enumerate(arucos_info):
                    numero_orden = orden + 1 if self.modo_ordenamiento != "normal" else None
                    color = (0, 255, 0)
                    dibujar_aruco_ordenado(aruco_info['esquina'], aruco_info['id'], imagen, color, numero_orden)
                    centro = aruco_info['centro']
                    texto_angulo = f"Angulo: {aruco_info['angulo']:.1f}"
                    offset = 20
                    cv2.putText(imagen, texto_angulo, (centro[0], centro[1] + offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                if hasattr(self, 'lbl_modo'):
                    modo_texto = f"Modo: {self.modo_ordenamiento.upper()}"
                    if len(arucos_info) > 0:
                        modo_texto += f" - {len(arucos_info)} ArUcos"
                    self.ventana.after(0, lambda: self.lbl_modo.config(text=modo_texto))
            else:
                cv2.putText(imagen, "Coloque un Aruco por favor", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if proporcion_cm_por_pixel is not None:
                for contorno in contornos:
                    area = cv2.contourArea(contorno)
                    if area > 100:
                        x, y, ancho, alto = cv2.boundingRect(contorno)
                        centro_x, centro_y = x + ancho // 2, y + alto // 2
                        cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), (0, 255, 0), 2)
                        cv2.circle(imagen, (centro_x, centro_y), 3, (0, 0, 255), -1)

            # Redimensionar imagen para ventana pequeña
            imagen_pequena = cv2.resize(imagen, (480, 360))
            
            # Mostrar la imagen redimensionada en la ventana pequeña
            cv2.imshow("Vista de Cámara", imagen_pequena)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.capturando = False
                break

            # No actualizar la imagen en la interfaz principal, solo mostrar la ventana aparte

        self.camara.release()
        cv2.destroyWindow("Vista de Cámara")
    
    # =========================================================================
    # MÉTODOS AUXILIARES DE INTERFAZ
    # =========================================================================
    
    def actualizar_etiquetas(self, img_tk, mascara_tk):
        self.lbl_video.configure(image=img_tk)
        self.lbl_video.image = img_tk
        
        self.lbl_mascara.configure(image=mascara_tk)
        self.lbl_mascara.image = mascara_tk
    
    def __del__(self):
        self.capturando = False
        if self.camara and self.camara.isOpened():
            self.camara.release()
        # Cerrar ventana del láser si está abierta
        if self.ventana_laser and self.ventana_laser.ventana.winfo_exists():
            self.ventana_laser.cerrar_ventana()
        # Cerrar ventana del robot si está abierta
        if self.ventana_robot and self.ventana_robot.ventana.winfo_exists():
            self.ventana_robot.cerrar_ventana()
        # Cerrar ventanas originales del robot si están abiertas
        if self.ventana_scorbot_original and self.ventana_scorbot_original.ventana.winfo_exists():
            self.ventana_scorbot_original.cerrar_ventana()
        if self.ventana_comunicacion_serial and self.ventana_comunicacion_serial.ventana.winfo_exists():
            self.ventana_comunicacion_serial.cerrar_ventana()

# =============================================================================
# CLASE VENTANA CREAR ARUCO - INTERFAZ PARA GENERAR MARCADORES
# =============================================================================

#Laser


#Crear y guardar Arucos

# Nueva clase para crear y guardar ArUco con interfaz en español
class VentanaCrearAruco:
    def __init__(self, parent):
        self.parent = parent
        self.ventana = tk.Toplevel(parent)
        self.ventana.title("Crear y Guardar ArUco")
        self.ventana.geometry("400x300")
        self.ventana.resizable(False, False)

        # Variables
        self.var_diccionario = tk.StringVar(value="DICT_4X4_100")
        self.var_tamano = tk.IntVar(value=200)
        self.var_id = tk.IntVar(value=0)
        self.label_imagen = None

        self.crear_interfaz()

    # =========================================================================
    # MÉTODOS DE CREACIÓN DE INTERFAZ - VENTANA CREAR ARUCO
    # =========================================================================

    def crear_interfaz(self):
        frame = ttk.Frame(self.ventana, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)

        # Diccionario
        ttk.Label(frame, text="Diccionario:").grid(row=0, column=0, sticky=tk.W)
        dicc_combo = ttk.Combobox(frame, textvariable=self.var_diccionario,
            values=["DICT_ARUCO_ORIGINAL", "DICT_4X4_100", "DICT_5X5_100", "DICT_6X6_100", "DICT_7X7_100"], width=20)
        dicc_combo.grid(row=0, column=1, padx=5, pady=5)

        # Tamaño
        ttk.Label(frame, text="Tamaño (px):").grid(row=1, column=0, sticky=tk.W)
        ttk.Entry(frame, textvariable=self.var_tamano, width=10).grid(row=1, column=1, padx=5, pady=5)

        # ID
        ttk.Label(frame, text="ID del marcador:").grid(row=2, column=0, sticky=tk.W)
        ttk.Entry(frame, textvariable=self.var_id, width=10).grid(row=2, column=1, padx=5, pady=5)

        # Botón para generar y guardar
        ttk.Button(frame, text="Generar y Guardar", command=self.generar_guardar_aruco).grid(row=3, column=0, columnspan=2, pady=10)

        # Label para mostrar imagen generada
        self.label_imagen = ttk.Label(frame, text="")
        self.label_imagen.grid(row=4, column=0, columnspan=2, pady=10)

        # Botón para cerrar
        ttk.Button(frame, text="Cerrar", command=self.ventana.destroy).grid(row=5, column=0, columnspan=2, pady=5)

    # =========================================================================
    # MÉTODOS DE FUNCIONALIDAD - VENTANA CREAR ARUCO
    # =========================================================================

    def generar_guardar_aruco(self):
        # Obtener parámetros
        dicc_nombre = self.var_diccionario.get()
        tamano = self.var_tamano.get()
        id_aruco = self.var_id.get()
        try:
            diccionario = getattr(aruco, 'getPredefinedDictionary')(getattr(aruco, dicc_nombre))
            marcador = aruco.generateImageMarker(diccionario, id_aruco, tamano)
            # Guardar archivo
            archivo = filedialog.asksaveasfilename(
                title="Guardar marcador ArUco",
                defaultextension=".png",
                filetypes=[("Imagen PNG", "*.png")]
            )
            if archivo:
                cv2.imwrite(archivo, marcador)
                self.label_imagen.config(text=f"Marcador guardado: {os.path.basename(archivo)}")
                # Mostrar imagen generada
                img_pil = Image.fromarray(marcador)
                img_pil = img_pil.resize((100, 100))
                img_tk = ImageTk.PhotoImage(img_pil)
                self.label_imagen.config(image=img_tk)
                self.label_imagen.image = img_tk
            else:
                self.label_imagen.config(text="Guardado cancelado")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo generar el marcador:\n{e}")
            self.label_imagen.config(text="Error al generar el marcador")

# =============================================================================
# PUNTO DE ENTRADA PRINCIPAL
# =============================================================================

#Crear y guardar imagen de arucos


# Crear y ejecutar la aplicación
if __name__ == "__main__":
    root = tk.Tk()
    app = Aplicacion(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (root.quit(), root.destroy()))
    root.mainloop()
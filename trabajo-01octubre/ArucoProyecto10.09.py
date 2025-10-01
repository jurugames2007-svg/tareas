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

# Importar funcionalidades del láser
try:
    import serial
except ImportError:
    serial = None

# Diccionario y parámetros del marcador ArUco
diccionario_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
parametros_aruco = aruco.DetectorParameters()

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

# ===================== Funciones del Láser =====================

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

# ===================== Ventana del Láser =====================

class VentanaLaser:
    def __init__(self, parent):
        self.parent = parent
        self.ventana = tk.Toplevel(parent)
        self.ventana.title("Control del Láser")
        self.ventana.geometry("520x700")  # Aumentar altura
        self.ventana.resizable(True, True)  # Permitir redimensionar
        
        # Variables del láser
        self.ser_laser = None
        self.laser_port = tk.StringVar(value="COM1")
        self.laser_baud = tk.StringVar(value="115200")
        self.selected_image = tk.StringVar(value="No hay imagen seleccionada")
        self.image_path = None
        self.laser_inicializado = False  # Estado de inicialización
        
        # Parámetros de grabado
        self.size_mm_x = tk.DoubleVar(value=20.0)
        self.size_mm_y = tk.DoubleVar(value=20.0)
        self.ppmm = tk.DoubleVar(value=5.0)
        self.f_engrave = tk.DoubleVar(value=1000.0)
        self.f_travel = tk.DoubleVar(value=1000.0)
        self.s_max = tk.IntVar(value=600)
        self.gamma_val = tk.DoubleVar(value=0.6)
        self.offset_x = tk.DoubleVar(value=0.0)
        self.offset_y = tk.DoubleVar(value=0.0)
        self.mode = tk.StringVar(value="grayscale")
        self.invert = tk.BooleanVar(value=False)
        
        self.crear_interfaz_laser()
        
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
        ttk.Button(img_frame, text="Seleccionar Imagen", command=self.seleccionar_imagen).pack(side=tk.RIGHT, padx=5)
        ttk.Button(img_frame, text="Inicializar Láser", command=self.inicializar_laser).pack(side=tk.RIGHT, padx=(5, 10))
        
        # Parámetros de grabado
        params_frame = ttk.LabelFrame(main_frame, text="Parámetros de Grabado", padding="5")
        params_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Tamaño
        size_frame = ttk.Frame(params_frame)
        size_frame.pack(fill=tk.X, pady=2)
        ttk.Label(size_frame, text="Tamaño (mm):").pack(side=tk.LEFT)
        ttk.Label(size_frame, text="X:").pack(side=tk.LEFT, padx=(10, 0))
        ttk.Entry(size_frame, textvariable=self.size_mm_x, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Label(size_frame, text="Y:").pack(side=tk.LEFT, padx=(10, 0))
        ttk.Entry(size_frame, textvariable=self.size_mm_y, width=8).pack(side=tk.LEFT, padx=2)
        
        # Resolución
        res_frame = ttk.Frame(params_frame)
        res_frame.pack(fill=tk.X, pady=2)
        ttk.Label(res_frame, text="Píxeles por mm:").pack(side=tk.LEFT)
        ttk.Entry(res_frame, textvariable=self.ppmm, width=8).pack(side=tk.LEFT, padx=(10, 0))
        
        # Velocidades
        vel_frame = ttk.Frame(params_frame)
        vel_frame.pack(fill=tk.X, pady=2)
        ttk.Label(vel_frame, text="Vel. grabado:").pack(side=tk.LEFT)
        ttk.Entry(vel_frame, textvariable=self.f_engrave, width=8).pack(side=tk.LEFT, padx=(5, 0))
        ttk.Label(vel_frame, text="Vel. viaje:").pack(side=tk.LEFT, padx=(15, 0))
        ttk.Entry(vel_frame, textvariable=self.f_travel, width=8).pack(side=tk.LEFT, padx=(5, 0))
        
        # Potencia y gamma
        power_frame = ttk.Frame(params_frame)
        power_frame.pack(fill=tk.X, pady=2)
        ttk.Label(power_frame, text="Potencia máx:").pack(side=tk.LEFT)
        ttk.Entry(power_frame, textvariable=self.s_max, width=8).pack(side=tk.LEFT, padx=(5, 0))
        ttk.Label(power_frame, text="Gamma:").pack(side=tk.LEFT, padx=(15, 0))
        ttk.Entry(power_frame, textvariable=self.gamma_val, width=8).pack(side=tk.LEFT, padx=(5, 0))
        
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
        mode_combo = ttk.Combobox(mode_frame, textvariable=self.mode,
                                 values=["grayscale", "threshold", "dither"], width=12)
        mode_combo.pack(side=tk.LEFT, padx=(5, 0))
        ttk.Checkbutton(mode_frame, text="Invertir", variable=self.invert).pack(side=tk.LEFT, padx=(15, 0))
        
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
            
    def inicializar_laser(self):
        """Inicializa el láser después de seleccionar imagen"""
        if not self.image_path:
            messagebox.showwarning("Advertencia", "Selecciona una imagen primero")
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
            self.log_laser("� INICIANDO GRABADO...")
            
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

class Aplicacion:
    def abrir_ventana_crear_aruco(self):
     VentanaCrearAruco(self.ventana)

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
        self.modo_ordenamiento = "normal"  # "mayor", "menor", "azar", "normal"
        self.objetos_detectados = []  # Lista para almacenar objetos detectados
        
        # Ventana del láser
        self.ventana_laser = None
        
        # Crear interfaz
        self.crear_interfaz()
        
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
        frame_controles.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))

        # Botones de control
        btn_inicio = ttk.Button(frame_controles, text="Inicio", command=self.boton_inicio_no_funciona)
        btn_inicio.grid(row=0, column=0, padx=5, pady=5)

        btn_ordenar_mayor = ttk.Button(frame_controles, text="Ordenar Mayor", command=self.boton_ordenar_mayor)
        btn_ordenar_mayor.grid(row=0, column=1, padx=5, pady=5)

        btn_ordenar_menor = ttk.Button(frame_controles, text="Ordenar Menor", command=self.boton_ordenar_menor)
        btn_ordenar_menor.grid(row=0, column=2, padx=5, pady=5)
        btn_crear_aruco = ttk.Button(frame_controles, text="Crear Aruco", command=self.abrir_ventana_crear_aruco)   
        btn_crear_aruco.grid(row=0, column=4, padx=5, pady=5)
        # Botón para abrir ventana del láser
        btn_laser = ttk.Button(frame_controles, text="Control Láser", command=self.abrir_ventana_laser)
        btn_laser.grid(row=0, column=3, padx=5, pady=5)

        btn_crear_aruco = ttk.Button(frame_controles, text="Crear Aruco", command=self.abrir_ventana_crear_aruco)
        btn_crear_aruco.grid(row=0, column=4, padx=5, pady=5)

        # Label para mostrar el modo actual
        self.lbl_modo = ttk.Label(frame_controles, text="Modo: NORMAL", 
                                 font=("Arial", 10, "bold"), foreground="blue")
        self.lbl_modo.grid(row=1, column=0, columnspan=5, padx=5, pady=5)

        # Frame para ajustes de imagen
        frame_ajustes = ttk.LabelFrame(frame_principal, text="Ajustes de Imagen", padding="5")
        frame_ajustes.grid(row=1, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))

        # Control de brillo
        lbl_brillo = ttk.Label(frame_ajustes, text="Brillo:")
        lbl_brillo.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        self.scale_brillo = Scale(frame_ajustes, from_=0, to=200, orient=tk.HORIZONTAL, 
                                 command=self.ajustar_brillo)
        self.scale_brillo.set(self.brillo)
        self.scale_brillo.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))

        # Control de contraste
        lbl_contraste = ttk.Label(frame_ajustes, text="Contraste:")
        lbl_contraste.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)

        self.scale_contraste = Scale(frame_ajustes, from_=0, to=200, orient=tk.HORIZONTAL, 
                                   command=self.ajustar_contraste)
        self.scale_contraste.set(self.contraste)
        self.scale_contraste.grid(row=1, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))

        # Control de zoom
        lbl_zoom = ttk.Label(frame_ajustes, text="Zoom:")
        lbl_zoom.grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)

        self.scale_zoom = Scale(frame_ajustes, from_=100, to=200, orient=tk.HORIZONTAL, 
                              command=self.ajustar_zoom)
        self.scale_zoom.set(self.zoom)
        self.scale_zoom.grid(row=2, column=1, padx=5, pady=5, sticky=(tk.W, tk.E))

        # Botón para resetear ajustes
        btn_reset = ttk.Button(frame_ajustes, text="Resetear Ajustes", command=self.resetear_ajustes)
        btn_reset.grid(row=2, column=2, padx=5, pady=5)

        # Frame para video
        frame_video = ttk.LabelFrame(frame_principal, text="Vista de Cámara", padding="5")
        frame_video.grid(row=3, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Label para mostrar video
        self.lbl_video = ttk.Label(frame_video, text="Cámara no iniciada")
        self.lbl_video.grid(row=0, column=0)

        # Frame para máscara
        frame_mascara = ttk.LabelFrame(frame_principal, text="Vista de Máscara", padding="5")
        frame_mascara.grid(row=4, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Label para mostrar máscara
        self.lbl_mascara = ttk.Label(frame_mascara, text="Máscara no disponible")
        self.lbl_mascara.grid(row=0, column=0)

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

        # Iniciar cámara
        self.iniciar_camara()
    
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
                self.ventana_laser = VentanaLaser(self.ventana)
                print("Ventana del láser abierta")
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo abrir la ventana del láser:\n{e}")
        else:
            # Si ya existe, la trae al frente
            self.ventana_laser.ventana.lift()
            self.ventana_laser.ventana.focus_force()
        
#    def boton_ordenar_azar(self):
#       self.modo_ordenamiento = "azar"
#        self.lbl_modo.config(text="Modo: ORDENAR AL AZAR", foreground="purple")
#        print("Modo ordenamiento activado - Al azar")
        
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
            
            # Recortar el centro de la imagen
            start_x = (new_w - w) // 2
            start_y = (new_h - h) // 2
            imagen_ajustada = imagen_zoom[start_y:start_y+h, start_x:start_x+w]
        
        return imagen_ajustada
        
    def iniciar_camara(self):
        try:
            self.camara = cv2.VideoCapture(0)
            
            if not self.camara.isOpened():
                raise Exception("No se pudo abrir la cámara")
                
            # Configurar propiedades de la cámara
            self.camara.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camara.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # Configurar buffer para reducir latencia
            self.camara.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            print("Cámara abierta exitosamente")
            
            self.capturando = True
            self.hilo_camara = threading.Thread(target=self.actualizar_video)
            self.hilo_camara.daemon = True
            self.hilo_camara.start()
            
        except Exception as e:
            error_msg = f"Error al iniciar cámara: {e}"
            print(error_msg)
            self.lbl_video.config(text=error_msg)
    
    def actualizar_video(self):
        global proporcion_cm_por_pixel
        
        while self.capturando:
            if not self.camara or not self.camara.isOpened():
                break
                
            exito, imagen = self.camara.read()
            
            if not exito:
                # Si falla la captura, esperar un poco y continuar
                threading.Event().wait(0.01)  # Esperar 10ms
                continue
            
            if imagen is None:
                continue
                
            # Aplicar ajustes de imagen
            imagen = self.aplicar_ajustes_imagen(imagen)
            
            # Convertir a HSV para detección de color
            hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
            
            # Crear máscaras para cada color
            mascara_rojo1 = cv2.inRange(hsv, rojo_bajo1, rojo_alto1)
            mascara_rojo2 = cv2.inRange(hsv, rojo_bajo2, rojo_alto2)
            mascara_rojo = cv2.bitwise_or(mascara_rojo1, mascara_rojo2)
            mascara_verde = cv2.inRange(hsv, verde_bajo, verde_alto)
            mascara_azul = cv2.inRange(hsv, azul_bajo, azul_alto)
            mascara_colores = cv2.bitwise_or(mascara_rojo, mascara_verde)
            mascara_colores = cv2.bitwise_or(mascara_colores, mascara_azul)
            
            # Mejorar la máscara con operaciones morfológicas
            kernel = np.ones((5, 5), np.uint8)
            mascara_colores = cv2.morphologyEx(mascara_colores, cv2.MORPH_CLOSE, kernel)
            mascara_colores = cv2.morphologyEx(mascara_colores, cv2.MORPH_OPEN, kernel)
            
            # Encontrar contornos en la máscara
            contornos, _ = cv2.findContours(mascara_colores, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Detectar marcadores ArUco usando la API apropiada
            if usar_nueva_api and detector_aruco is not None:
                # Nueva API (OpenCV >= 4.7)
                esquinas, ids, rechazados = detector_aruco.detectMarkers(imagen)
            else:
                # API antigua (OpenCV < 4.7)
                try:
                    esquinas, ids, rechazados = aruco.detectMarkers(imagen, diccionario_aruco, parameters=parametros_aruco)
                except AttributeError:
                    # Si no funciona, mostrar error y continuar sin detección
                    esquinas, ids, rechazados = [], None, []
                    cv2.putText(imagen, "Error: ArUco no compatible", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            if ids is not None:
                # Dibujar los marcadores detectados
                imagen = aruco.drawDetectedMarkers(imagen, esquinas, ids)
                
                # Calcular ancho y alto del ArUco en píxeles usando las esquinas detectadas del primer ArUco
                puntos = esquinas[0][0]
                x_min = int(np.min(puntos[:, 0]))
                x_max = int(np.max(puntos[:, 0]))
                y_min = int(np.min(puntos[:, 1]))
                y_max = int(np.max(puntos[:, 1]))
                ancho_pix = x_max - x_min
                alto_pix = y_max - y_min

                # Calcular proporción cm/píxel usando el tamaño real del ArUco
                proporcion_cm_por_pixel = tamano_aruco_cm / ancho_pix  # Asumiendo marcador cuadrado

                # Mostrar ancho y alto del ArUco en pantalla
                texto_ancho = f"Ancho Aruco: {ancho_pix * proporcion_cm_por_pixel:.2f} cm"
                texto_alto = f"Alto Aruco: {alto_pix * proporcion_cm_por_pixel:.2f} cm"
                cv2.putText(imagen, texto_ancho, (x_min, y_min - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(imagen, texto_alto, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
                # Preparar lista de ArUcos para ordenamiento
                arucos_info = []
                for i, (esquina, id_val) in enumerate(zip(esquinas, ids.flatten())):
                    pts = esquina[0]
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
                
                # Ordenar ArUcos según el modo seleccionado
                if self.modo_ordenamiento == "mayor":
                    # Ordenar por ID de mayor a menor
                    arucos_info.sort(key=lambda aruco: aruco['id'], reverse=True)
                elif self.modo_ordenamiento == "menor":
                    # Ordenar por ID de menor a mayor
                    arucos_info.sort(key=lambda aruco: aruco['id'])
                elif self.modo_ordenamiento == "azar":
                    # Orden aleatorio
                    random.shuffle(arucos_info)
                
                # Dibujar ArUcos con información de ordenamiento
                for orden, aruco_info in enumerate(arucos_info):
                    numero_orden = orden + 1 if self.modo_ordenamiento != "normal" else None
                    
                    # Asignar colores según el modo
                    if self.modo_ordenamiento == "mayor":
                        # Del rojo (mayor ID) al verde (menor ID)
                        factor = orden / max(1, len(arucos_info) - 1)
                        color = (int(255 * (1 - factor)), int(255 * factor), 0)
                    elif self.modo_ordenamiento == "menor":
                        # Del verde (menor ID) al rojo (mayor ID)
                        factor = orden / max(1, len(arucos_info) - 1)
                        color = (int(255 * factor), int(255 * (1 - factor)), 0)
                    elif self.modo_ordenamiento == "azar":
                        # Colores aleatorios
                        color = (random.randint(100, 255), random.randint(100, 255), random.randint(100, 255))
                    else:
                        color = (0, 255, 0)  # Verde para modo normal
                    
                    # Dibujar ArUco ordenado
                    dibujar_aruco_ordenado(aruco_info['esquina'], aruco_info['id'], imagen, color, numero_orden)
                    
                    # Mostrar ángulo como antes
                    centro = aruco_info['centro']
                    texto_angulo = f"Angulo: {aruco_info['angulo']:.1f}"
                    offset = 20
                    cv2.putText(imagen, texto_angulo, (centro[0], centro[1] + offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                
                # Actualizar el label del modo con información de ArUcos
                if hasattr(self, 'lbl_modo'):
                    modo_texto = f"Modo: {self.modo_ordenamiento.upper()}"
                    if len(arucos_info) > 0:
                        modo_texto += f" - {len(arucos_info)} ArUcos"
                    self.ventana.after(0, lambda: self.lbl_modo.config(text=modo_texto))
                
            else:
                cv2.putText(imagen, "Coloque un Aruco por favor", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Medir objetos de colores si tenemos proporción (solo detectar, sin medir)
            if proporcion_cm_por_pixel is not None:
                for contorno in contornos:
                    area = cv2.contourArea(contorno)
                    if area > 100:  # Filtrar contornos pequeños
                        # Solo dibujar contorno sin mediciones
                        x, y, ancho, alto = cv2.boundingRect(contorno)
                        centro_x, centro_y = x + ancho // 2, y + alto // 2
                        
                        # Dibujar solo el rectángulo y centro
                        cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), (0, 255, 0), 2)
                        cv2.circle(imagen, (centro_x, centro_y), 3, (0, 0, 255), -1)
            
            # Convertir imágenes para mostrar en Tkinter
            img_rgb = cv2.cvtColor(imagen, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(img_rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            
            mascara_rgb = cv2.cvtColor(mascara_colores, cv2.COLOR_GRAY2RGB)
            mascara_pil = Image.fromarray(mascara_rgb)
            mascara_tk = ImageTk.PhotoImage(image=mascara_pil)
            
            # Actualizar labels en el hilo principal
            self.ventana.after(0, self.actualizar_etiquetas, img_tk, mascara_tk)
            
        self.camara.release()
    
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

#Crear y guardar imagen de arucos



# Crear y ejecutar la aplicación
if __name__ == "__main__":
    root = tk.Tk()
    app = Aplicacion(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (root.quit(), root.destroy()))
    root.mainloop()
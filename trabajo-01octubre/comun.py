import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.filedialog import asksaveasfilename
import serial
import time
import serial.tools.list_ports

# Crear objeto Serial
SerialPort1 = serial.Serial()

# Variables de la interfaz gráfica (se inicializan solo cuando se ejecuta directamente)
TextoEstado = None
TextEnviar = None
TextRecibidos = None
comboBox1 = None

# ==== FUNCIONES ACTUALIZADAS ====
def send_scorbot_command(command, description=""):
    """Función mejorada para enviar comandos al Scorbot-ER V Plus"""
    if SerialPort1.is_open:
        try:
            # Asegurar formato correcto del comando
            cmd = command.upper().strip() + '\r'  # Comandos en mayúsculas y con retorno de carro
            
            # Limpiar buffer de entrada
            SerialPort1.reset_input_buffer()
            
            # Enviar comando
            SerialPort1.write(cmd.encode())
            
            # Esperar respuesta (ajustar tiempo según necesidad)
            time.sleep(0.5)
            
            # Leer respuesta
            response = SerialPort1.read_all().decode('ascii', errors='ignore').strip()
            
            # Mostrar en la interfaz (solo si está disponible)
            if TextRecibidos is not None:
                TextRecibidos.insert("1.0", f"> {cmd.strip()}\n")
                TextRecibidos.insert("2.0", f"< {response}\n")
                if description:
                    TextRecibidos.insert("3.0", f"{description}\n")
                TextRecibidos.insert("4.0", "-"*50 + "\n")
            
            return True
        except Exception as e:
            messagebox.showerror("Error", f"Error al enviar comando: {str(e)}")
            return False
    else:
        messagebox.showerror("Error", "Debe conectar el puerto primero")
        return False

def click_pcplc():
    send_scorbot_command("RUN PCPLC", "Ejecutar programa PCPLC")

def click_a():
    send_scorbot_command("ABORT", "Movimiento abortado")

def click_ttsib():
    send_scorbot_command("RUN TTSIB", "Ejecutar programa TTSIB")

def click_coff():
    send_scorbot_command("COFF", "Apagar servomotores")

def click_move():
    send_scorbot_command("MOVE 0", "Mover a posición 0")

def click_open():
    send_scorbot_command("OPEN", "Abrir gripper")

def click_close():
    send_scorbot_command("CLOSE", "Cerrar gripper")

def click_left():
    send_scorbot_command("MJ 1 -10", "Base movida 10° a la izquierda (antihorario)")

def click_right():
    send_scorbot_command("MJ 1 10", "Base movida 10° a la derecha (horario)")

def click_home():
    send_scorbot_command("HOME", "Mover a posición HOME")

def click_ready():
    send_scorbot_command("READY", "Preparar robot para movimiento")

def click_speed():
    send_scorbot_command("SPEED 50", "Velocidad establecida al 50%")

def click_conectar():
    if not SerialPort1.is_open:
        SerialPort1.baudrate = 9600
        SerialPort1.bytesize = 8
        SerialPort1.parity = "N"
        SerialPort1.stopbits = serial.STOPBITS_ONE
        SerialPort1.timeout = 1  # Timeout de lectura
        
        if comboBox1 is not None:
            SerialPort1.port = comboBox1.get()
        else:
            SerialPort1.port = "COM1"  # Puerto por defecto cuando no hay interfaz
            
        try:
            SerialPort1.open()
            
            if TextoEstado is not None:
                TextoEstado.config(state="normal")
                TextoEstado.delete("1.0", tk.END)
                TextoEstado.insert("1.0", "CONECTADO")
                TextoEstado.configure(background="lime")
            
            # Configuración inicial recomendada para Scorbot-ER V Plus
            send_scorbot_command("JOINT", "Modo joint activado")
            send_scorbot_command("READY", "Robot listo para comandos")
            send_scorbot_command("SPEED 30", "Velocidad inicial al 30%")
            
            messagebox.showinfo("Conectado", "Conexión establecida con Scorbot-ER V Plus")
            
            if TextoEstado is not None:
                TextoEstado.config(state="disabled")
        except Exception as e:
            messagebox.showerror("Error de conexión", f"No se pudo conectar: {str(e)}")
    else:
        messagebox.showinfo("Información", "El puerto ya está conectado.")

def click_desconectar():
    if SerialPort1.is_open:
        try:
            send_scorbot_command("COFF", "Apagando servomotores antes de desconectar")
            SerialPort1.close()
            
            if TextoEstado is not None:
                TextoEstado.config(state="normal")
                TextoEstado.delete("1.0", tk.END)
                TextoEstado.insert("1.0", "DESCONECTADO")
                TextoEstado.configure(background="red")
            
            messagebox.showinfo("Desconectado", "Conexión cerrada correctamente")
            
            if TextoEstado is not None:
                TextoEstado.config(state="disabled")
        except Exception as e:
            messagebox.showerror("Error", f"Error al desconectar: {str(e)}")
    else:
        messagebox.showinfo("Información", "El puerto ya está desconectado")

def click_enviar():
    if TextEnviar is not None:
        custom_cmd = TextEnviar.get("1.0", tk.END).strip()
    else:
        custom_cmd = ""
        
    if custom_cmd:
        send_scorbot_command(custom_cmd, "Comando personalizado enviado")
    else:
        messagebox.showwarning("Advertencia", "Escriba un comando primero")

def click_guardar():
    if TextRecibidos is None:
        messagebox.showwarning("Advertencia", "No hay interfaz disponible para guardar")
        return
        
    filepath = asksaveasfilename(
        defaultextension=".txt",
        filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
    )
    if not filepath:
        return
    with open(filepath, "w") as output_file:
        text = TextRecibidos.get("1.0", tk.END)
        output_file.write(text)

def update_com_ports():
    if comboBox1 is None:
        return
        
    ports = [port.device for port in serial.tools.list_ports.comports()]
    comboBox1['values'] = ports
    if ports:
        comboBox1.set(ports[0])
    else:
        comboBox1.set("No ports found")

# ==== INTERFAZ GRÁFICA ====
# Solo ejecutar la interfaz si este archivo se ejecuta directamente
if __name__ == "__main__":
    root = tk.Tk()
    root.title("Controlador Scorbot-ER V Plus")
    root.geometry("700x550")
    root.resizable(False, False)

    # Configuración de estilo
    style = ttk.Style()
    style.configure('TButton', font=('Arial', 9), padding=5)
    style.configure('TLabel', font=('Arial', 9))

    # Panel de estado
    frame_estado = ttk.Frame(root, padding=10)
    frame_estado.pack(fill=tk.X)

    TextoEstado = tk.Text(frame_estado, height=1, width=15, state="disabled", 
                         bg="red", font=("Arial", 10, "bold"))
    TextoEstado.insert("1.0", "DESCONECTADO")
    TextoEstado.pack(side=tk.LEFT, padx=5)

    ttk.Button(frame_estado, text="Conectar", command=click_conectar).pack(side=tk.LEFT, padx=5)
    ttk.Button(frame_estado, text="Desconectar", command=click_desconectar).pack(side=tk.LEFT, padx=5)

    comboBox1 = ttk.Combobox(frame_estado, state="readonly", width=25)
    comboBox1.pack(side=tk.LEFT, padx=5)
    update_com_ports()

    # Panel de control principal
    main_frame = ttk.Frame(root, padding=10)
    main_frame.pack(fill=tk.BOTH, expand=True)

    # Panel de comandos rápidos
    cmd_frame = ttk.LabelFrame(main_frame, text="Comandos Rápidos", padding=10)
    cmd_frame.grid(row=0, column=0, padx=5, pady=5, sticky=tk.N)

    buttons = [
        ("Home", click_home),
        ("Ready", click_ready),
        ("Run PCPLC", click_pcplc),
        ("Run TTSIB", click_ttsib),
        ("Abortar", click_a),
        ("Apagar (COFF)", click_coff),
        ("Mover a 0", click_move),
        ("Abrir Gripper", click_open),
        ("Cerrar Gripper", click_close),
        ("Izquierda (-10°)", click_left),
        ("Derecha (+10°)", click_right),
        ("Velocidad 50%", click_speed)
    ]

    for text, cmd in buttons:
        ttk.Button(cmd_frame, text=text, command=cmd).pack(fill=tk.X, pady=2)

    # Panel de comunicación
    comm_frame = ttk.LabelFrame(main_frame, text="Comunicación", padding=10)
    comm_frame.grid(row=0, column=1, padx=5, pady=5, sticky=tk.NSEW)

    # Área de envío
    ttk.Label(comm_frame, text="Comando personalizado:").pack(anchor=tk.W)
    TextEnviar = tk.Text(comm_frame, height=3, width=30, font=('Consolas', 10))
    TextEnviar.pack(fill=tk.X, pady=5)

    ttk.Button(comm_frame, text="Enviar Comando", command=click_enviar).pack(fill=tk.X, pady=5)

    # Área de recepción
    ttk.Label(comm_frame, text="Respuesta del robot:").pack(anchor=tk.W)
    TextRecibidos = tk.Text(comm_frame, height=15, width=50, font=('Consolas', 9))
    TextRecibidos.pack(fill=tk.BOTH, expand=True)

    # Botón Guardar
    ttk.Button(comm_frame, text="Guardar Log", command=click_guardar).pack(fill=tk.X, pady=5)

    root.mainloop()
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.filedialog import asksaveasfilename
import serial
import time
import serial.tools.list_ports
from threading import Thread

# Crear objeto Serial
SerialPort1 = serial.Serial()

# ==== FUNCIONES ==== 
def escuchar_automatica():
    while True:
        if SerialPort1.is_open:
            recibir = SerialPort1.read_all()  # Leer todos los datos del puerto
            if recibir:
                # Verificar si el mensaje contiene "CMG" o "CHG"
                recibir_str = recibir.decode('utf-8')
                if "CMG" in recibir_str or "CHG" in recibir_str:
                    # Si se detecta "CMG" o "CHG", enviar los comandos en orden
                    enviar_comandos_automatically()
                    break  # Salir después de enviar los comandos (para evitar múltiples envíos)
            time.sleep(1)  # Pausa de 1 segundo antes de intentar leer más datos

def enviar_comandos_automatically():
    if SerialPort1.is_open:
        # Lista de comandos
        comandos = [b"run gp", b"run dp", b"run initc", b"run gpv", b"run gf", b"run initc"]

        for comando in comandos:
            # Enviar el comando
            SerialPort1.write(comando + b"\r")
            TextRecibidos.insert(tk.END, f"Comando Enviado: {comando.decode()}\n")

            # Esperar que el robot termine el comando
            respuesta = leer_respuesta()
            print(f"Respuesta recibida: {respuesta}")  # Depuración para ver lo que se recibe

            # Verificar si la respuesta indica que el comando ha terminado
            if "done" in respuesta.lower() or "ok" in respuesta.lower() or "complete" in respuesta.lower():
                TextRecibidos.insert(tk.END, f"Comando {comando.decode()} completado.\n")
            else:
                # Solo tratar como error si la respuesta no es esperada o claramente fallida
                if "error" in respuesta.lower():
                    TextRecibidos.insert(tk.END, f"Error al ejecutar {comando.decode()}.\nRespuesta: {respuesta}\n")
                    print(f"Error al ejecutar {comando.decode()}. Respuesta: {respuesta}")
                    break  # Salir si hubo un error crítico
                else:
                    TextRecibidos.insert(tk.END, f"Respuesta no esperada para {comando.decode()}: {respuesta}\n")
                    print(f"Respuesta no esperada: {respuesta}")
                    # No interrumpir el flujo, pero loguear el error para futuras revisiones

            if comando == b"run dp":
                # Ejecutar initc después de dp
                time.sleep(15)  # Esperar 15 segundos entre dp y gpv

            #elif comando == b"run initc":
                # Verificar si "initc" terminó correctamente
             #   respuesta = leer_respuesta()
              #  time.sleep(1)
                

            elif comando == b"run gpv":
                # Esperar a que el comando gpv termine antes de continuar con gf
                respuesta = leer_respuesta()
                if "done" in respuesta.lower()  or "ok2" in respuesta.lower() or "complete" in respuesta.lower():
                    TextRecibidos.insert(tk.END, "Comando gpv completado.\n")
                else:
                    TextRecibidos.insert(tk.END, f"Respuesta no esperada para gpv: {respuesta}\n")
                    print(f"Respuesta no esperada para gpv: {respuesta}")

            elif comando == b"run gf":
                # Esperar a que el comando gf termine correctamente
                respuesta = leer_respuesta()
                if "done" in respuesta.lower() or "ok3" in respuesta.lower() or "complete" in respuesta.lower():
                    TextRecibidos.insert(tk.END, "Comando gf enviado correctamente.\n")
                else:
                    TextRecibidos.insert(tk.END, "Error al ejecutar gf. El robot no está listo.\n")
                    break  # Salir si el robot no está listo para ejecutar gf

            else:
                time.sleep(1)  # Para los demás comandos, 1 segundo de espera

        # Enviar initc después de gf, asegurando que los pasos previos fueron completados correctamente
        TextRecibidos.insert(tk.END, "\nComandos enviados correctamente.\n")  # Confirmación final


def leer_respuesta():
    """Lee la respuesta del puerto serial hasta encontrar un salto de línea o timeout."""
    response = b""
    start_time = time.time()

    while time.time() - start_time < 30:  # Espera de hasta 30 segundos
        if SerialPort1.in_waiting > 0:
            response += SerialPort1.read(SerialPort1.in_waiting)
        if b"Done." in response or b"done" in response or b"ok" in response or b"complete" in response:
            break
        time.sleep(0.1)  # Pausa para no sobrecargar el CPU

    respuesta_str = response.decode('utf-8', errors='ignore')
    print(f"Respuesta completa recibida: {respuesta_str}")  # Esto ayudará a depurar el flujo
    return respuesta_str


def click_conectar():
    if not SerialPort1.is_open:
        SerialPort1.baudrate = 9600
        SerialPort1.bytesize = 8
        SerialPort1.parity = "N"
        SerialPort1.stopbits = serial.STOPBITS_ONE
        
        if comboBox1 is not None:
            SerialPort1.port = comboBox1.get()
        else:
            SerialPort1.port = "COM1"  # Puerto por defecto cuando no hay interfaz
            
        try:
            SerialPort1.open()
            safe_widget_config(TextoEstado, state="normal")
            if TextoEstado is not None:
                TextoEstado.delete("1.0", tk.END)
                TextoEstado.insert("1.0", "CONECTADO")
                TextoEstado.configure(background="lime")
            messagebox.showinfo(message="Puerto Conectado")
            safe_widget_config(TextoEstado, state="disabled")
            # Iniciar el proceso de escucha automática en un hilo separado
            Thread(target=escuchar_automatica, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Error de conexión", str(e))
    else:
        messagebox.showinfo("Información", "El puerto ya está conectado.")

def click_desconectar():
    if SerialPort1.is_open:
        SerialPort1.close()
        safe_widget_config(TextoEstado, state="normal")
        if TextoEstado is not None:
            TextoEstado.delete("1.0", tk.END)
            TextoEstado.insert("1.0", "DESCONECTADO")
            TextoEstado.configure(background="red")
        messagebox.showinfo(message="Puerto Desconectado")
        safe_widget_config(TextoEstado, state="disabled")
    else:
        messagebox.showinfo("Información", "El puerto ya está desconectado.")

def click_enviar():
    if SerialPort1.is_open:
        if TextEnviar is not None:
            data_to_send = TextEnviar.get("1.0", tk.END).strip()
        else:
            data_to_send = ""
            
        if data_to_send:
            SerialPort1.write(data_to_send.encode() + b"\r")
            time.sleep(2)
            aux = SerialPort1.read_all()
            if b"Done." in aux:
                time.sleep(42)
                safe_text_insert(TextRecibidos, tk.END, b"OK\n")
            messagebox.showinfo(message="Enviado Correctamente", title="Resultado")
        else:
            messagebox.showwarning("Advertencia", "El campo de envío está vacío.")
    else:
        messagebox.showerror("Error", "Debe conectar el puerto primero")

def update_com_ports():
    if comboBox1 is None:
        return
        
    ports = [port.device for port in serial.tools.list_ports.comports()]
    comboBox1['values'] = ports
    if ports:
        comboBox1.set(ports[0])
    else:
        comboBox1.set("No ports found")

# Variables de la interfaz gráfica (se inicializan solo cuando se ejecuta directamente)
TextoEstado = None
TextEnviar = None
TextRecibidos = None
comboBox1 = None

def safe_text_insert(widget, position, text):
    """Inserta texto en un widget de forma segura"""
    if widget is not None:
        widget.insert(position, text)

def safe_widget_config(widget, **kwargs):
    """Configura un widget de forma segura"""
    if widget is not None:
        widget.config(**kwargs)

# ==== INTERFAZ ====
# Solo ejecutar la interfaz si este archivo se ejecuta directamente
if __name__ == "__main__":
    root = tk.Tk()
    root.title("Comunicacion Serial")
    root.geometry("600x400")  #

    # Etiqueta de estado (DESCONECTADO)
    TextoEstado = tk.Text(root, height=1, width=15, state="disabled", bg="red", font=("Arial", 10, "bold"))
    TextoEstado.insert("1.0", "DESCONECTADO")
    TextoEstado.place(x=240, y=10)  # Reposicionado

    # Botones Conectar y Desconectar
    ttk.Button(root, text="Conectar", command=click_conectar).place(x=10, y=40, width=100)
    ttk.Button(root, text="Desconectar", command=click_desconectar).place(x=390, y=40, width=100)

    # ComboBox de puertos
    comboBox1 = ttk.Combobox(root, state="readonly")
    comboBox1.place(x=120, y=40, width=260)  # Reposicionado y ajustado el ancho
    update_com_ports()  # Llenar el combobox al inicio

    # Etiqueta "Datos Enviados" y campo de envío
    tk.Label(root, text="Datos Enviados").place(x=200, y=80)
    TextEnviar = tk.Text(root, height=3, width=30)
    TextEnviar.place(x=150, y=100)

    # Etiqueta "Datos Recibidos" y campo de recepción
    tk.Label(root, text="Datos Recibidos").place(x=430, y=80)
    TextRecibidos = tk.Text(root, height=10, width=30)
    TextRecibidos.place(x=350, y=100)

    # Botón Enviar
    ttk.Button(root, text="Enviar", command=click_enviar).place(x=220, y=160, width=80)

    root.mainloop()

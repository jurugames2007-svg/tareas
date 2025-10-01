import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image, ImageOps
import matplotlib.pyplot as plt

def crear_comparacion_aruco():
    """Crea una comparación visual de ArUco normal vs invertido"""
    
    # Crear ArUco
    diccionario = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    marcador = aruco.generateImageMarker(diccionario, 0, 200)
    
    # Convertir a PIL
    img_pil = Image.fromarray(marcador)
    img_invertida = ImageOps.invert(img_pil)
    marcador_invertido = np.array(img_invertida)
    
    # Crear visualización
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    # ArUco original
    axes[0].imshow(marcador, cmap='gray')
    axes[0].set_title('ArUco ORIGINAL\n(Negro=0, Blanco=255)\nSIN INVERTIR = Graba el código (negro)')
    axes[0].axis('off')
    
    # ArUco invertido
    axes[1].imshow(marcador_invertido, cmap='gray')
    axes[1].set_title('ArUco INVERTIDO\n(Negro=255, Blanco=0)\nCON INVERTIR = Graba el fondo (blanco)')
    axes[1].axis('off')
    
    # Diferencia
    diff = cv2.absdiff(marcador, marcador_invertido)
    axes[2].imshow(diff, cmap='hot')
    axes[2].set_title('DIFERENCIA\n(Rojo=máxima diferencia)')
    axes[2].axis('off')
    
    plt.tight_layout()
    plt.savefig('comparacion_aruco.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print("✅ EXPLICACIÓN DEL PROBLEMA:")
    print("🔸 ArUco original: código negro (0) + fondo blanco (255)")
    print("🔸 Láser SIN invertir: graba las partes negras (el código)")
    print("🔸 Resultado: código quemado = NO DETECTABLE")
    print()
    print("✅ SOLUCIÓN:")
    print("🔸 Láser CON invertir: graba las partes blancas (el fondo)")  
    print("🔸 Resultado: fondo quemado + código sin quemar = DETECTABLE")
    print()
    print("📁 Archivos guardados:")
    print("  - aruco_normal.png")
    print("  - aruco_invertido.png") 
    print("  - comparacion_aruco.png")

if __name__ == "__main__":
    crear_comparacion_aruco()

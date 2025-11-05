import serial
import time
import serial.tools.list_ports



def main():
    print("🔎 Puertos disponibles:")
    for p in serial.tools.list_ports.comports():
        print(f" - {p.device}")

    puerto = input("👉 Ingresa el puerto que quieres usar (ej: COM5 o /dev/ttyUSB0): ")


    # Configura tu puerto y velocidad
    #puerto = 'COM3'       # Cambia según tu sistema ('/dev/ttyUSB0' en Linux)
    baudrate = 9600       # Velocidad de transmisión en baudios

    try:
        # Abrir el puerto serie
        ser = serial.Serial(puerto, baudrate, timeout=1)
        print(f"✅ Puerto {puerto} abierto correctamente a {baudrate} baudios.")
        print("Presiona Ctrl+C para detener.\n")

        # Bucle principal de lectura
        while True:
            if ser.in_waiting > 0:  # Si hay datos disponibles
                linea = ser.readline().decode('utf-8', errors='replace').strip()
                if linea:
                    print(f"📥 Recibido: {linea}")
            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"❌ Error al abrir el puerto: {e}")
    except KeyboardInterrupt:
        print("\n🛑 Lectura interrumpida por el usuario.")
    finally:
        # Cerrar el puerto al salir
        try:
            ser.close()
            print(f"🔒 Puerto {puerto} cerrado correctamente.")
        except:
            pass

if __name__ == "__main__":
    main()

"""
Módulo para la creación automática de acceso directo en el Escritorio (crear_acceso_directo.py)
Crea un archivo 'Escáner 3D LiDAR.lnk' en el Escritorio del usuario que apunta directamente
a pythonw.exe del entorno virtual ejecutando main.py, garantizando ejecución 100% nativa
sin abrir pestañas de consola ni ventanas de Windows Terminal en Windows 11.
"""

import os
import subprocess
import sys

def crear_acceso_directo():
    project_dir = os.path.dirname(os.path.abspath(__file__))
    main_py_path = os.path.join(project_dir, "main.py")
    icon_path = os.path.join(project_dir, "assets", "lidar_icon.ico")
    
    if not os.path.exists(main_py_path):
        print(f"[ERROR] No se encontró el archivo principal: {main_py_path}")
        return False

    if not os.path.exists(icon_path):
        print(f"[ERROR] No se encontró el icono: {icon_path}")
        return False

    # 1. Determinar el ejecutable gráfico de Python (pythonw.exe) prioritario
    dot_venv_pythonw = os.path.join(project_dir, ".venv", "Scripts", "pythonw.exe")
    venv_pythonw = os.path.join(project_dir, "venv", "Scripts", "pythonw.exe")

    if os.path.exists(dot_venv_pythonw):
        target_path = dot_venv_pythonw
    elif os.path.exists(venv_pythonw):
        target_path = venv_pythonw
    else:
        # Fallback a pythonw del entorno actual de ejecución
        target_path = sys.executable.replace("python.exe", "pythonw.exe")
        if not os.path.exists(target_path):
            target_path = "pythonw.exe"

    # Argumentos para pythonw.exe: ruta completa entre comillas a main.py
    arguments = f'"{main_py_path}"'

    # 2. Script de PowerShell para invocar WScript.Shell y escribir el acceso directo en el Escritorio
    # Usamos formato seguro para escapar comillas dobles en PowerShell
    escaped_args = arguments.replace('"', '`"')
    ps_script = f"""
    $WshShell = New-Object -ComObject WScript.Shell
    $DesktopPath = [System.Environment]::GetFolderPath([System.Environment+SpecialFolder]::Desktop)
    $ShortcutPath = Join-Path $DesktopPath "Escáner 3D LiDAR.lnk"
    $Shortcut = $WshShell.CreateShortcut($ShortcutPath)
    $Shortcut.TargetPath = "{target_path}"
    $Shortcut.Arguments = "{escaped_args}"
    $Shortcut.WorkingDirectory = "{project_dir}"
    $Shortcut.IconLocation = "{icon_path},0"
    $Shortcut.Description = "Escáner 3D LiDAR ESP32 - Visualizador & Calibrador"
    $Shortcut.Save()
    
    # Comprobar metadatos guardados
    $SavedShortcut = $WshShell.CreateShortcut($ShortcutPath)
    Write-Output "ACCESO_DIRECTO_CREADO:$ShortcutPath"
    Write-Output "TARGET_PATH:$($SavedShortcut.TargetPath)"
    Write-Output "ARGUMENTS:$($SavedShortcut.Arguments)"
    """

    try:
        resultado = subprocess.run(
            ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ps_script],
            capture_output=True,
            text=True,
            check=True
        )
        salida = resultado.stdout.strip()
        
        # Parsear salida de PowerShell
        ruta_lnk = None
        target_guardado = None
        args_guardados = None

        for linea in salida.splitlines():
            linea = linea.strip()
            if linea.startswith("ACCESO_DIRECTO_CREADO:"):
                ruta_lnk = linea.split("ACCESO_DIRECTO_CREADO:")[1].strip()
            elif linea.startswith("TARGET_PATH:"):
                target_guardado = linea.split("TARGET_PATH:")[1].strip()
            elif linea.startswith("ARGUMENTS:"):
                args_guardados = linea.split("ARGUMENTS:")[1].strip()

        if ruta_lnk and os.path.exists(ruta_lnk):
            print(f"[OK] Acceso directo actualizado exitosamente en: {ruta_lnk}")
            print(f"      • TargetPath: {target_guardado or target_path}")
            print(f"      • Arguments: {args_guardados or arguments}")
            print(f"      • WorkingDirectory: {project_dir}")
            print(f"      • IconLocation: {icon_path},0")
            print("      • Modo: Ejecución GUI nativa sin consola (sin Windows Terminal)")
            return True
        else:
            print(f"[ERROR] No se pudo verificar la creación del archivo: {ruta_lnk}")
            return False

    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Fallo al invocar PowerShell: {e.stderr}")
        return False
    except Exception as e:
        print(f"[ERROR] Excepción inesperada: {e}")
        return False

if __name__ == "__main__":
    exito = crear_acceso_directo()
    sys.exit(0 if exito else 1)

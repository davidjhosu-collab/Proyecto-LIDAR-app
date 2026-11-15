"""
Módulo de Gestión de Configuración Persistente (config_manager.py)
Permite almacenar y cargar la calibración de hardware, límites y offsets en config.json
utilizando rutas absolutas, escritura atómica segura y acceso flexible por sección o notación por puntos.
"""

import json
import os
import sys

# Ruta absoluta al archivo config.json en el mismo directorio que este script
DEFAULT_CONFIG_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "config.json")
)

DEFAULT_CONFIG = {
    "servos": {
        "pan_min": 0,
        "pan_max": 150,
        "tilt_min": 0,
        "tilt_max": 180,
        "invert_pan": False,
        "invert_tilt": False
    },
    "offsets": {
        "pan_offset": 0.0,
        "tilt_offset": 0.0,
        "dist_offset_mm": 0.0
    },
    "geometry": {
        "sensor_height_mm": 65.0
    },
    "scan": {
        "resolution": 1.0
    },
    "connection": {
        "baudrate": 115200,
        "last_port": ""
    },
    "sor_filter": {
        "enabled_export": True,
        "k_neighbors": 16,
        "std_ratio": 1.5
    }
}


class ConfigManager:
    """Manejador singleton/instancia para lectura/escritura atómica de config.json."""

    def __init__(self, filepath=DEFAULT_CONFIG_PATH):
        self.filepath = os.path.abspath(filepath)
        self.data = {}
        self.load()

    def load(self):
        """Carga la configuración desde config.json o crea los valores por defecto si no existe."""
        if not os.path.exists(self.filepath):
            self.data = json.loads(json.dumps(DEFAULT_CONFIG))
            self.save()
            return self.data

        try:
            with open(self.filepath, 'r', encoding='utf-8') as f:
                loaded = json.load(f)

            # Combinar asegurando que todas las secciones y claves por defecto existan
            self.data = json.loads(json.dumps(DEFAULT_CONFIG))
            for section, values in loaded.items():
                if section in self.data and isinstance(values, dict):
                    self.data[section].update(values)
                else:
                    self.data[section] = values
        except Exception as e:
            if sys.stdout is not None:
                print(f"[ConfigManager] Advertencia al leer {self.filepath}: {e}. Usando valores por defecto.")
            self.data = json.loads(json.dumps(DEFAULT_CONFIG))
            self.save()

        return self.data

    def save(self):
        """
        Guarda la configuración actual en config.json de manera atómica
        escribiendo primero a un archivo temporal y reemplazando luego con os.replace.
        """
        tmp_filepath = self.filepath + ".tmp"
        try:
            parent_dir = os.path.dirname(self.filepath)
            if parent_dir and not os.path.exists(parent_dir):
                os.makedirs(parent_dir, exist_ok=True)

            with open(tmp_filepath, 'w', encoding='utf-8') as f:
                json.dump(self.data, f, indent=4)
                f.flush()
                os.fsync(f.fileno())

            # Reemplazo atómico en el sistema de archivos
            os.replace(tmp_filepath, self.filepath)
            return True
        except Exception as e:
            if sys.stdout is not None:
                print(f"[ConfigManager] Error crítico al guardar {self.filepath}: {e}")
            if os.path.exists(tmp_filepath):
                try:
                    os.remove(tmp_filepath)
                except Exception:
                    pass
            return False

    def get(self, section, key=None, default=None):
        """
        Permite acceder tanto mediante sección y clave:
            config.get("servos", "pan_min", 0)
        como mediante notación por puntos:
            config.get("servos.pan_min", 0)
        """
        if isinstance(section, str) and "." in section:
            parts = section.split(".", 1)
            # En notación de puntos con 2 argumentos: get("servos.pan_min", 0), key es el valor por defecto
            real_default = key if default is None else default
            return self.data.get(parts[0], {}).get(parts[1], real_default)

        if key is not None:
            return self.data.get(section, {}).get(key, default)

        return self.data.get(section, default)

    def set(self, section, key=None, value=None):
        """
        Permite asignar mediante sección y clave:
            config.set("servos", "pan_min", 0)
        o mediante notación por puntos:
            config.set("servos.pan_min", 0)
        """
        if isinstance(section, str) and "." in section and value is None:
            parts = section.split(".", 1)
            sec, k, val = parts[0], parts[1], key
        else:
            sec, k, val = section, key, value

        if sec not in self.data:
            self.data[sec] = {}
        self.data[sec][k] = val

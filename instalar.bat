@echo off
title Instalador - Escaner 3D LiDAR
cd /d "%~dp0"

echo [1/3] Creando entorno virtual de Python...
python -m venv venv

echo [2/3] Instalando dependencias...
call venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt

echo [3/3] Creando acceso directo en el Escritorio...
python crear_acceso_directo.py

echo.
echo Instalacion completada. Ya puedes abrir "Escaner 3D LiDAR" desde tu Escritorio.
pause
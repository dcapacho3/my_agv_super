#!/usr/bin/env python3

import sqlite3
from ament_index_python.packages import get_package_share_directory
import os

def create_database():
    # Obtener la ruta del directorio actual del script
    
    # Construir la ruta relativa a la carpeta database
  
  #  bringup_dir = get_package_share_directory('my_agv_super')
   # launch_dir = os.path.join(bringup_dir, 'launch')
    
    connection = sqlite3.connect("database/products.db")
    cursor = connection.cursor()
    
    # Crear tabla si no existe
    cursor.execute('''CREATE TABLE IF NOT EXISTS products (
                        id INTEGER PRIMARY KEY,
                        name TEXT NOT NULL,
                        x REAL NOT NULL,
                        y REAL NOT NULL
                    )''')
    
    connection.commit()
    connection.close()

if __name__ == '__main__':
    create_database()
    print("Database and table created successfully.")


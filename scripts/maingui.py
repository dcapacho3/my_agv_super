#!/usr/bin/env python3

import sqlite3
import customtkinter as ctk
from tkinter import simpledialog
import os
import datetime
from navigationgui import NavigationWindow
import signal


class ProductManager:
    def __init__(self, root):
        self.root = root
        self.checkbox_vars = {}
        self.after_id = None

        # Configuración de la interfaz
        self.setup_ui()
        signal.signal(signal.SIGINT, self.signal_handler)

        # Configurar el cierre de ventana con la "X"
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)


    def setup_ui(self):
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.root.title("Shop Vision")
        self.root.geometry("%dx%d+0+0" % (self.root.winfo_screenwidth(), self.root.winfo_screenheight()))
        self.root.resizable(width=1, height=1)

        # Frame en la parte superior
        top_frame = ctk.CTkFrame(self.root, height=200)
        top_frame.pack(side=ctk.TOP, fill=ctk.X)
        title_label = ctk.CTkLabel(top_frame, text="Product Management Interface", font=("Helvetica", 28, "bold"))
        title_label.pack(padx=20, pady=10)

        # Frame a la izquierda
        left_frame = ctk.CTkFrame(self.root, width=200)
        left_frame.pack(side=ctk.LEFT, fill=ctk.Y)

        # Reloj
        self.label_reloj = ctk.CTkLabel(left_frame, font=('ARIAL', 18, 'bold'))
        self.label_reloj.pack(side=ctk.TOP, padx=10, pady=10)

        # Fecha
        self.label_fecha = ctk.CTkLabel(left_frame, font=('ARIAL', 18, 'bold'))
        self.label_fecha.pack(side=ctk.TOP, padx=10, pady=70)

        # Actualizar reloj y fecha
        self.actualizar_reloj_y_fecha()

        # Texto Shop Vision
        shop_vision_label = ctk.CTkLabel(left_frame, text="Shop Vision", font=('Helvetica', 20, 'bold'))
        shop_vision_label.pack(side=ctk.BOTTOM, padx=10, pady=10)

        # Frame de búsqueda dentro del frame izquierdo
        search_frame = ctk.CTkFrame(left_frame)
        search_frame.pack(side=ctk.TOP, fill=ctk.X, padx=10, pady=10)

        # Campo de entrada para búsqueda
        self.search_entry = ctk.CTkEntry(search_frame, font=("Arial", 12))
        self.search_entry.pack(side=ctk.TOP, padx=10, pady=5)

        # Botón de búsqueda
        search_button = ctk.CTkButton(search_frame, text="Buscar", command=self.perform_search)
        search_button.pack(side=ctk.TOP, padx=10, pady=5)

        # Frame para lista de productos
        frame1 = ctk.CTkFrame(self.root, width=350)
        frame1.pack(side=ctk.LEFT, fill=ctk.BOTH, padx=20, pady=20, expand=True)

        # Frame para productos seleccionados
        frame2 = ctk.CTkFrame(self.root, width=350)
        frame2.pack(side=ctk.RIGHT, fill=ctk.BOTH, padx=20, pady=20, expand=True)

        # Título para la lista de productos
        products_label = ctk.CTkLabel(frame1, text="Productos", font=("Helvetica", 24, "bold"))
        products_label.pack(pady=10)

        # Scrollable Frame para la lista de productos
        self.treeview_frame = ctk.CTkScrollableFrame(frame1, width=300, height=400)
        self.treeview_frame.pack(fill=ctk.BOTH, expand=True, pady=10)

        select_button = ctk.CTkButton(frame1, text="Select Products", command=self.select_products, width=200, height=50, font=("Helvetica", 16))
        select_button.pack(pady=10)

        # Título para productos seleccionados
        selected_label = ctk.CTkLabel(frame2, text="Productos Seleccionados", font=("Helvetica", 24, "bold"))
        selected_label.pack(pady=10)

        self.selected_frame = ctk.CTkScrollableFrame(frame2, width=300, height=400)
        self.selected_frame.pack(fill=ctk.BOTH, expand=True, pady=10)
        
        go_to_products_button = ctk.CTkButton(frame2, text="Dirigirse a productos", command=self.navigation_window, width=200, height=50, font=("Helvetica", 16))
        go_to_products_button.pack(pady=10)

        self.refresh_treeview()

    def show_info(self, message, title="Info"):
        info_window = ctk.CTkToplevel()
        info_window.title(title)
        info_window.geometry("300x150")
        label = ctk.CTkLabel(info_window, text=message, padx=20, pady=20)
        label.pack(expand=True)
        ok_button = ctk.CTkButton(info_window, text="OK", command=info_window.destroy)
        ok_button.pack(pady=10)

    def refresh_treeview(self, search_term=""):
        # Limpiar el frame de productos
        for widget in self.treeview_frame.winfo_children():
            widget.destroy()

        db_dir = os.path.join('src/my_agv_super/database/products.db')
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()

        # Modificar la consulta para filtrar por el término de búsqueda
        cursor.execute("SELECT * FROM products WHERE name LIKE ?", ('%' + search_term + '%',))

        for row in cursor.fetchall():
            if row[0] not in self.checkbox_vars:
                self.checkbox_vars[row[0]] = ctk.IntVar()
            checkbox = ctk.CTkCheckBox(self.treeview_frame, text=f"{row[1]} ({row[2]}, {row[3]})", variable=self.checkbox_vars[row[0]])
            checkbox.pack(anchor="w", padx=10, pady=5)
            # Restaurar el estado del checkbox
            checkbox.select() if self.checkbox_vars[row[0]].get() else checkbox.deselect()

        connection.close()

    def select_products(self):
        selected_items = [key for key, var in self.checkbox_vars.items() if var.get()]
        if selected_items:
            db_dir = os.path.join('src/my_agv_super/database/products.db')
            connection = sqlite3.connect(db_dir)
            cursor = connection.cursor()
            
            cursor.execute("DELETE FROM selected_products")
            
            for product_id in selected_items:
                cursor.execute("SELECT name, x, y FROM products WHERE id = ?", (product_id,))
                product_info = cursor.fetchone()
                cursor.execute("INSERT INTO selected_products (name, x, y) VALUES (?, ?, ?)",
                               (product_info[0], product_info[1], product_info[2]))
            
            connection.commit()
            connection.close()
            self.show_info("Products selected successfully.")
            self.view_selected_products()
        else:
            self.show_info("No items selected.", title="Error")

    def view_selected_products(self):
        for widget in self.selected_frame.winfo_children():
            widget.destroy()

        db_dir = os.path.join('src/my_agv_super/database/products.db')
        connection = sqlite3.connect(db_dir)
        cursor = connection.cursor()
        cursor.execute("SELECT * FROM selected_products")
        for row in cursor.fetchall():
            ctk.CTkLabel(self.selected_frame, text=f"{row[0]} ({row[1]}, {row[2]})").pack(anchor="w", padx=10, pady=5)
        connection.close()

    def perform_search(self):
        search_term = self.search_entry.get()
        self.refresh_treeview(search_term)

    def actualizar_reloj_y_fecha(self):
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after_id = self.root.after(1000, self.actualizar_reloj_y_fecha)  # Guarda el ID del after

    def navigation_window(self):
        if self.after_id is not None:
           self.root.after_cancel(self.after_id) 
        print('y que esperabas un dulce')
        self.new_window = NavigationWindow()  # Crea una nueva ventana
        
        #self.root.destroy()  # Cierra la ventana actual
        self.new_window.mainloop()

    def signal_handler(self, sig, frame):
        print("Ctrl+C detectado, cerrando la aplicación...")
        self.on_closing()

    def on_closing(self):
        """Método para manejar el cierre controlado de la ventana."""
        print("Cerrando la ventana correctamente...")
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        self.root.quit()  # Salir del bucle principal de tkinter
        self.root.destroy()  # Cerrar la ventana completamente


if __name__ == '__main__':
    root = ctk.CTk()
    app = ProductManager(root)
    root.mainloop()


#!/usr/bin/env python3

import customtkinter as ctk
import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class NavigationWindow(ctk.CTk):  # Hereda de ctk.CTk para ser una ventana
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.title("Navigation Interface")
        self.geometry("%dx%d+0+0" % (self.winfo_screenwidth(), self.winfo_screenheight()))  # Se usa `self` en lugar de `master`
        self.resizable(width=1, height=1)

        # Frame en la parte superior
        self.top_frame = ctk.CTkFrame(self, height=200)
        self.top_frame.pack(side=ctk.TOP, fill=ctk.X)
        title_label = ctk.CTkLabel(self.top_frame, text="Navigation Interface", font=("Helvetica", 28, "bold"))
        title_label.pack(padx=20, pady=10)

        # Frame a la izquierda
        self.left_frame = ctk.CTkFrame(self, width=200)
        self.left_frame.pack(side=ctk.LEFT, fill=ctk.Y)

        # Reloj
        self.label_reloj = ctk.CTkLabel(self.left_frame, font=('ARIAL', 18, 'bold'))
        self.label_reloj.pack(side=ctk.TOP, padx=10, pady=10)

        # Fecha
        self.label_fecha = ctk.CTkLabel(self.left_frame, font=('ARIAL', 18, 'bold'))
        self.label_fecha.pack(side=ctk.TOP, padx=10, pady=70)

        # Actualizar reloj y fecha
        self.actualizar_reloj_y_fecha()

        # Frame para el gráfico
        self.graph_frame = ctk.CTkFrame(self)
        self.graph_frame.pack(side=ctk.RIGHT, fill=ctk.BOTH, expand=True, padx=20, pady=20)

        # Crear y mostrar el gráfico
        self.create_plot(self.graph_frame)

    def actualizar_reloj_y_fecha(self):
        now = datetime.datetime.now()
        self.label_reloj.configure(text=now.strftime("%H:%M:%S"))
        self.label_fecha.configure(text=now.strftime("%Y-%m-%d"))
        self.after(1000, self.actualizar_reloj_y_fecha)  # Actualiza cada segundo

    def create_plot(self, frame):
        # Crear una figura y un eje
        fig, ax = plt.subplots(figsize=(6, 4), dpi=100)
        x = [1, 2, 3, 4, 5]
        y = [1, 4, 9, 16, 25]
        ax.plot(x, y)
        ax.set_title("Sample Plot")
        ax.set_xlabel("X Axis")
        ax.set_ylabel("Y Axis")

        # Mostrar el gráfico en la ventana
        canvas = FigureCanvasTkAgg(fig, master=frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=ctk.BOTH, expand=True)

if __name__ == '__main__':
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")
    
    root = ctk.CTk()  # La ventana principal
    app = NavigationWindow(master=root)
    app.mainloop()


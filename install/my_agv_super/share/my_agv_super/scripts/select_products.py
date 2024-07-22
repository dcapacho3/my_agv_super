#!/usr/bin/env python3

import sqlite3
import tkinter as tk
from tkinter import messagebox, simpledialog

def refresh_listbox():
    listbox.delete(0, tk.END)
    connection = sqlite3.connect("database/products.db")
    cursor = connection.cursor()
    cursor.execute("SELECT * FROM products")
    for row in cursor.fetchall():
        listbox.insert(tk.END, row)
    connection.close()

def add_product():
    name = simpledialog.askstring("Input", "Enter product name:")
    x = simpledialog.askfloat("Input", "Enter x coordinate:")
    y = simpledialog.askfloat("Input", "Enter y coordinate:")
    if name and x is not None and y is not None:
        connection = sqlite3.connect("database/products.db")
        cursor = connection.cursor()
        cursor.execute("INSERT INTO products (name, x, y) VALUES (?, ?, ?)", (name, x, y))
        connection.commit()
        connection.close()
        refresh_listbox()
    else:
        messagebox.showerror("Error", "All fields must be filled out.")

def delete_product():
    selected_items = listbox.curselection()
    if selected_items:
        for selected_item in selected_items:
            product_id = listbox.get(selected_item)[0]
            connection = sqlite3.connect("database/products.db")
            cursor = connection.cursor()
            cursor.execute("DELETE FROM products WHERE id = ?", (product_id,))
            connection.commit()
            connection.close()
        refresh_listbox()
    else:
        messagebox.showerror("Error", "No item selected.")

def update_product():
    selected_item = listbox.curselection()
    if selected_item:
        product_id = listbox.get(selected_item)[0]
        name = simpledialog.askstring("Input", "Enter new product name:")
        x = simpledialog.askfloat("Input", "Enter new x coordinate:")
        y = simpledialog.askfloat("Input", "Enter new y coordinate:")
        if name and x is not None and y is not None:
            connection = sqlite3.connect("database/products.db")
            cursor = connection.cursor()
            cursor.execute("UPDATE products SET name = ?, x = ?, y = ? WHERE id = ?", (name, x, y, product_id))
            connection.commit()
            connection.close()
            refresh_listbox()
        else:
            messagebox.showerror("Error", "All fields must be filled out.")
    else:
        messagebox.showerror("Error", "No item selected.")

def select_products():
    selected_items = listbox.curselection()
    if selected_items:
        connection = sqlite3.connect("database/products.db")
        cursor = connection.cursor()
        
        # Limpiar la tabla temporal
        cursor.execute("DELETE FROM selected_products")
        
        for item in selected_items:
            product_id = listbox.get(item)[0]
            cursor.execute("SELECT name, x, y FROM products WHERE id = ?", (product_id,))
            product_info = cursor.fetchone()
            cursor.execute("INSERT INTO selected_products (name, x, y) VALUES (?, ?, ?)",
                           (product_info[0], product_info[1], product_info[2]))
        
        connection.commit()
        connection.close()
        messagebox.showinfo("Info", "Products selected successfully.")
    else:
        messagebox.showerror("Error", "No items selected.")

def view_selected_products():
    selected_listbox.delete(0, tk.END)
    connection = sqlite3.connect("database/products.db")
    cursor = connection.cursor()
    cursor.execute("SELECT * FROM selected_products")
    for row in cursor.fetchall():
        selected_listbox.insert(tk.END, row)
    connection.close()

if __name__ == '__main__':
    connection = sqlite3.connect("database/products.db")
    cursor = connection.cursor()
    
    # Eliminar la tabla temporal si ya existe
    cursor.execute("DROP TABLE IF EXISTS selected_products")
    
    # Crear la tabla temporal sin columna de id
    cursor.execute('''CREATE TABLE IF NOT EXISTS selected_products (
                        name TEXT,
                        x REAL,
                        y REAL,
                        PRIMARY KEY (name, x, y)
                    )''')
    connection.commit()
    connection.close()

    root = tk.Tk()
    root.title("Product Manager")

    frame1 = tk.Frame(root)
    frame1.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

    frame2 = tk.Frame(root)
    frame2.pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

    listbox = tk.Listbox(frame1, selectmode=tk.MULTIPLE)
    listbox.pack(fill=tk.BOTH, expand=1)

    refresh_button = tk.Button(frame1, text="Refresh", command=refresh_listbox)
    refresh_button.pack()

    add_button = tk.Button(frame1, text="Add Product", command=add_product)
    add_button.pack()

    update_button = tk.Button(frame1, text="Update Product", command=update_product)
    update_button.pack()

    delete_button = tk.Button(frame1, text="Delete Product", command=delete_product)
    delete_button.pack()

    select_button = tk.Button(frame1, text="Select Products", command=select_products)
    select_button.pack()

    selected_listbox = tk.Listbox(frame2)
    selected_listbox.pack(fill=tk.BOTH, expand=1)

    view_selected_button = tk.Button(frame2, text="View Selected Products", command=view_selected_products)
    view_selected_button.pack()

    refresh_listbox()
    root.mainloop()


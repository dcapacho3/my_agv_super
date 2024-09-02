#!/usr/bin/env python3

import sqlite3
import tkinter as tk
from tkinter import messagebox, simpledialog
from ament_index_python.packages import get_package_share_directory
import os


def refresh_listbox():

  
    bringup_dir = get_package_share_directory('my_agv_super')
    db_dir = os.path.join(bringup_dir, 'database/products.db')
    print(db_dir)
    
    
    listbox.delete(0, tk.END)
    connection = sqlite3.connect( db_dir)
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
    selected_item = listbox.curselection()
    if selected_item:
        product_id = listbox.get(selected_item)[0]
        connection = sqlite3.connect( db_dir)
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
            connection = sqlite3.connect( db_dir)
            cursor = connection.cursor()
            cursor.execute("UPDATE products SET name = ?, x = ?, y = ? WHERE id = ?", (name, x, y, product_id))
            connection.commit()
            connection.close()
            refresh_listbox()
        else:
            messagebox.showerror("Error", "All fields must be filled out.")
    else:
        messagebox.showerror("Error", "No item selected.")

if __name__ == '__main__':
    root = tk.Tk()
    root.title("Product Manager")

    listbox = tk.Listbox(root)
    listbox.pack(fill=tk.BOTH, expand=1)

    refresh_button = tk.Button(root, text="Refresh", command=refresh_listbox)
    refresh_button.pack()

    add_button = tk.Button(root, text="Add Product", command=add_product)
    add_button.pack()

    update_button = tk.Button(root, text="Update Product", command=update_product)
    update_button.pack()

    delete_button = tk.Button(root, text="Delete Product", command=delete_product)
    delete_button.pack()

    refresh_listbox()
    root.mainloop()


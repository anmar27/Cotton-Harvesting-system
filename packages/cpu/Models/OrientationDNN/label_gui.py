import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import os
import math
import ctypes
import re

class VectorAnnotationTool:
    def __init__(self, root):
        self.root = root
        self.root.title("Vector Annotation Tool")
        
        #Get system DPI scaling
        self.dpi_scale = self.get_dpi_scaling()
        print("DPI_SCALE: " + str(self.dpi_scale))
        #Apply DPI scaling to default dimensions
        default_width = int(800 * self.dpi_scale)
        default_height = int(600 * self.dpi_scale)
        
        #Window dimensions
        self.canvas_width = default_width
        self.canvas_height = default_height
        
        #Variables
        self.image_path = None
        self.image = None
        self.photo = None
        self.start_x = None
        self.start_y = None
        self.current_vector = None
        self.vector_info = None  # (start_x, start_y, end_x, end_y, confidence)
        self.vector_annotation = None
        
        default_font_size = int(10 * self.dpi_scale)
        font_config = ("TkDefaultFont", default_font_size)
        
        self.image_frame = tk.Frame(root, bd=2, relief=tk.SUNKEN)
        self.image_frame.grid(row=0, column=0, padx=10, pady=10)
        
        self.canvas = tk.Canvas(self.image_frame, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.pack()
        
        self.control_frame = tk.Frame(root)
        self.control_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")
        
        button_width = int(15 * self.dpi_scale)
        
        #Buttons
        self.load_btn = tk.Button(self.control_frame, text="Load Image", command=self.load_image, 
                                 font=font_config, width=button_width)
        self.load_btn.grid(row=0, column=0, pady=5, padx=5, sticky="ew")
        
        self.load_folder_btn = tk.Button(self.control_frame, text="Load Folder", command=self.load_folder,
                                         font=font_config, width=button_width)
        self.load_folder_btn.grid(row=1, column=0, pady=5, padx=5, sticky="ew")
        
        self.save_btn = tk.Button(self.control_frame, text="Save Annotation", command=self.save_annotation, 
                                 font=font_config, width=button_width)
        self.save_btn.grid(row=2, column=0, pady=5, padx=5, sticky="ew")
        self.save_btn.config(state=tk.DISABLED)
        
        self.prev_btn = tk.Button(self.control_frame, text="Previous", command=self.prev_image, state=tk.DISABLED,
                                  font=font_config, width=button_width)
        self.prev_btn.grid(row=3, column=0, pady=5, padx=5, sticky="ew")
        
        self.next_btn = tk.Button(self.control_frame, text="Next", command=self.next_image, state=tk.DISABLED,
                                  font=font_config, width=button_width)
        self.next_btn.grid(row=4, column=0, pady=5, padx=5, sticky="ew")
        
        self.clear_btn = tk.Button(self.control_frame, text="Clear Vector", command=self.clear_vector, 
                                  font=font_config, width=button_width)
        self.clear_btn.grid(row=5, column=0, pady=5, padx=5, sticky="ew")
        self.clear_btn.config(state=tk.DISABLED)
        
        self.info_frame = tk.LabelFrame(self.control_frame, text="Vector Information", font=font_config)
        self.info_frame.grid(row=6, column=0, pady=10, padx=5, sticky="ew")
        
        pad_x = int(5 * self.dpi_scale)
        pad_y = int(2 * self.dpi_scale)
        
        tk.Label(self.info_frame, text="Start:", font=font_config).grid(row=0, column=0, sticky="w", padx=pad_x, pady=pad_y)
        self.start_pos_label = tk.Label(self.info_frame, text="(-, -)", font=font_config)
        self.start_pos_label.grid(row=0, column=1, sticky="w", padx=pad_x, pady=pad_y)
        
        tk.Label(self.info_frame, text="End:", font=font_config).grid(row=1, column=0, sticky="w", padx=pad_x, pady=pad_y)
        self.end_pos_label = tk.Label(self.info_frame, text="(-, -)", font=font_config)
        self.end_pos_label.grid(row=1, column=1, sticky="w", padx=pad_x, pady=pad_y)
        
        tk.Label(self.info_frame, text="Vector:", font=font_config).grid(row=2, column=0, sticky="w", padx=pad_x, pady=pad_y)
        self.vector_label = tk.Label(self.info_frame, text="(-, -)", font=font_config)
        self.vector_label.grid(row=2, column=1, sticky="w", padx=pad_x, pady=pad_y)
        
        tk.Label(self.info_frame, text="Magnitude:", font=font_config).grid(row=3, column=0, sticky="w", padx=pad_x, pady=pad_y)
        self.magnitude_label = tk.Label(self.info_frame, text="-", font=font_config)
        self.magnitude_label.grid(row=3, column=1, sticky="w", padx=pad_x, pady=pad_y)
        
        tk.Label(self.info_frame, text="Confidence:", font=font_config).grid(row=4, column=0, sticky="w", padx=pad_x, pady=pad_y)
        self.confidence_label = tk.Label(self.info_frame, text="-", font=font_config)
        self.confidence_label.grid(row=4, column=1, sticky="w", padx=pad_x, pady=pad_y)
        
        #Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready. Load an image to begin.")
        self.status_bar = tk.Label(root, textvariable=self.status_var, bd=1, relief=tk.SUNKEN, anchor=tk.W, font=font_config)
        self.status_bar.grid(row=1, column=0, columnspan=2, sticky="ew")
        
        #Bind
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)
        
        self.max_magnitude_for_confidence = 200 * self.dpi_scale
    
    def get_dpi_scaling(self):
        """Get the DPI scaling factor from the system"""
        try:
            if os.name == 'nt':  # Windows
                user32 = ctypes.windll.user32
                user32.SetProcessDPIAware()
                return user32.GetDpiForSystem() / 96.0
            else:  # Mac/Linux
                return self.root.winfo_screenwidth() / 1920.0
        except:
            return 1.0
    
    def load_folder(self):
        folder_path = filedialog.askdirectory(title="Select Image Folder")
        if folder_path:
            self.image_folder = folder_path
            self.image_files = [f for f in os.listdir(folder_path) if f.lower().endswith(('jpg', 'jpeg', 'png', 'bmp', 'gif'))]
            self.image_files.sort()

            if self.image_files:
                self.current_index = 0
                image_path = os.path.join(self.image_folder, self.image_files[self.current_index])
                self.load_image(image_path)  # Use the unified load_image() method
                self.update_navigation_buttons()
            else:
                messagebox.showwarning("Warning", "No images found in the selected folder.")

    def load_vector_from_label(self, label_file):
        """Check for a vector label file and display the vector on the canvas."""

        if os.path.exists(label_file):
            with open(label_file, "r") as file:
                content = file.read()
                
                start_match = re.search(r"Start: \((\d+), (\d+)\)", content)
                end_match = re.search(r"End: \((\d+), (\d+)\)", content)
                
                if start_match and end_match:
                    start_x, start_y = map(int, start_match.groups())
                    end_x, end_y = map(int, end_match.groups())
                    
                    self.vector_annotation = self.canvas.create_line(start_x, start_y, end_x, end_y, fill="red", width=2, arrow=tk.LAST, tags="vector")
                    self.status_var.set(f"Vector Loaded from {os.path.basename(label_file)}")
        else:
            print(f"No label file found, expected: {label_file}")
            
    def display_image(self):
        if self.current_index < 0 or self.current_index >= len(self.image_files):
            return
        
        self.canvas.delete("vector")
        image_path = os.path.join(self.image_folder, self.image_files[self.current_index])
        self.image_path = image_path
        self.image = Image.open(image_path)
        
        img_width, img_height = self.image.size
        ratio = min(self.canvas_width / img_width, self.canvas_height / img_height)
        new_width, new_height = int(img_width * ratio), int(img_height * ratio)
        self.image = self.image.resize((new_width, new_height), Image.LANCZOS)
        
        self.photo = ImageTk.PhotoImage(self.image)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
        
        label_path = os.path.join(self.image_folder, "../labels", f"{os.path.splitext(self.image_files[self.current_index])[0]}.txt")
        
        if os.path.exists(label_path):
            self.status_var.set(f"{self.current_index + 1}/{len(self.image_files)} - Label Exists: {os.path.basename(label_path)}")
            self.load_vector_from_label(label_path)
        else:
            self.status_var.set(f"{self.current_index + 1}/{len(self.image_files)} - No Label")
    
    def update_navigation_buttons(self):
        self.prev_btn.config(state=tk.NORMAL if self.current_index > 0 else tk.DISABLED)
        self.next_btn.config(state=tk.NORMAL if self.current_index < len(self.image_files) - 1 else tk.DISABLED)
    
    def prev_image(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.display_image()
            self.update_navigation_buttons()
    
    def next_image(self):
        if self.current_index < len(self.image_files) - 1:
            self.current_index += 1
            self.display_image()
            self.update_navigation_buttons()
    
    def load_image(self, image_path=None):
        """Load an image from a file or from the folder"""
        if image_path is None:
            image_path = filedialog.askopenfilename(
                title="Select Image",
                filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.gif")]
            )
            
        if image_path:
            try:
                self.image_path = image_path  # Ensure the image path is stored correctly
                self.image = Image.open(image_path)
    
                # Resize if too large
                img_width, img_height = self.image.size
                ratio = min(self.canvas_width / img_width, self.canvas_height / img_height)
                new_width = int(img_width * ratio)
                new_height = int(img_height * ratio)
                self.image = self.image.resize((new_width, new_height), Image.LANCZOS)
    
                self.photo = ImageTk.PhotoImage(self.image)
                self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)
    
                self.status_var.set(f"Loaded: {os.path.basename(image_path)}")
                self.clear_vector()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load image: {str(e)}")
    
    
    def on_mouse_down(self, event):
        """Handle mouse press event to start a vector"""
        if not self.image_path:
            return
            
        self.clear_vector()
        self.start_x = event.x
        self.start_y = event.y
        self.start_pos_label.config(text=f"({self.start_x}, {self.start_y})")
        
        #Draw a dot at the start position (scaled by DPI)
        dot_size = int(3 * self.dpi_scale)
        self.canvas.create_oval(
            self.start_x-dot_size, self.start_y-dot_size, 
            self.start_x+dot_size, self.start_y+dot_size, 
            fill="red", 
            tags="vector"
        )
        
        self.status_var.set("Drawing vector. Release mouse to set endpoint.")
    
    def on_mouse_drag(self, event):
        """Handle mouse drag event to update the vector"""
        if self.start_x is None or self.start_y is None:
            return
            
        if self.current_vector:
            self.canvas.delete(self.current_vector)
            
        line_width = int(2 * self.dpi_scale)
        self.current_vector = self.canvas.create_line(
            self.start_x, self.start_y, 
            event.x, event.y, 
            fill="blue", 
            width=line_width, 
            arrow=tk.LAST,
            tags="vector"
        )
        
        #Update labels
        self.end_pos_label.config(text=f"({event.x}, {event.y})")
        
        vector_x = event.x - self.start_x
        vector_y = event.y - self.start_y
        self.vector_label.config(text=f"({vector_x}, {vector_y})")
        
        magnitude = math.sqrt(vector_x**2 + vector_y**2)
        self.magnitude_label.config(text=f"{magnitude:.2f}")
        
        confidence = min(1.0, magnitude / self.max_magnitude_for_confidence)
        self.confidence_label.config(text=f"{confidence:.2f}")
    
    def on_mouse_up(self, event):
        """Handle mouse release event to finalize the vector"""
        if self.start_x is None or self.start_y is None:
            return
            
        end_x = event.x
        end_y = event.y
        
        vector_x = end_x - self.start_x
        vector_y = end_y - self.start_y
        magnitude = math.sqrt(vector_x**2 + vector_y**2)
        
        confidence = min(1.0, magnitude / self.max_magnitude_for_confidence)
        self.vector_info = (self.start_x, self.start_y, end_x, end_y, vector_x, vector_y, confidence)
        
        #Update UI
        self.end_pos_label.config(text=f"({end_x}, {end_y})")
        self.vector_label.config(text=f"({vector_x}, {vector_y})")
        self.magnitude_label.config(text=f"{magnitude:.2f}")
        self.confidence_label.config(text=f"{confidence:.2f}")
        
        self.save_btn.config(state=tk.NORMAL)
        self.clear_btn.config(state=tk.NORMAL)
        self.status_var.set("Vector created. You can save the annotation or clear and redraw.")
    
    def clear_vector(self):
        """Clear the current vector annotation"""
        self.canvas.delete("vector")
        self.current_vector = None
        self.start_x = None
        self.start_y = None
        self.vector_info = None
        
        # Reset labels
        self.start_pos_label.config(text="(-, -)")
        self.end_pos_label.config(text="(-, -)")
        self.vector_label.config(text="(-, -)")
        self.magnitude_label.config(text="-")
        self.confidence_label.config(text="-")
        
        self.save_btn.config(state=tk.DISABLED)
        self.clear_btn.config(state=tk.DISABLED)
        
        if self.image_path:
            self.status_var.set(f"Ready to annotate: {os.path.basename(self.image_path)}")
    
    def save_annotation(self):
        """Save vector annotation to a text file"""
        if not self.vector_info or not self.image_path:
            messagebox.showwarning("Warning", "No vector to save.")
            return
            
        base_name = os.path.splitext(os.path.basename(self.image_path))[0]
        
        image_dir = os.path.dirname(self.image_path)
        labels_dir = os.path.join(image_dir, '..', 'labels')
        print(labels_dir)
        
        if not os.path.exists(labels_dir):
            os.makedirs(labels_dir) #Create folder in case of not existing
        
        save_path = os.path.join(labels_dir, f"{base_name}.txt")
        print(save_path)
        
        start_x, start_y, end_x, end_y, vector_x, vector_y, confidence = self.vector_info
        
        try:
            with open(save_path, 'w') as f:
                f.write(f"Start: ({start_x}, {start_y})\n")
                f.write(f"End: ({end_x}, {end_y})\n")
                f.write(f"Vector: ({vector_x}, {vector_y})\n")
                f.write(f"Confidence: {confidence:.2f}\n")
            
            self.status_var.set(f"Annotation saved to: {os.path.basename(save_path)}")
            messagebox.showinfo("Success", f"Annotation saved to:\n{save_path}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save annotation: {str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    
    try:
        if os.name == 'nt':  # Windows
            from ctypes import windll
            windll.shcore.SetProcessDpiAwareness(1)
    except:
        pass
        
    app = VectorAnnotationTool(root)
    
    #Calculate window dimensions based on DPI scale
    dpi_scale = app.dpi_scale
    window_width = int(1050 * dpi_scale)  # Base width + control panel
    window_height = int(650 * dpi_scale)  # Base height + status bar
    
    #Center the window on screen
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    
    
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    root.minsize(int(800 * dpi_scale), int(600 * dpi_scale))
    
    root.mainloop()
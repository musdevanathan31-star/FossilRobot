#!/usr/bin/env python3
"""
Python project to control the fossil extractor arm 
"""

import sys
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from serial.tools import list_ports
import time
import serial
import cv2
from PIL import Image, ImageTk
import numpy as np
from scipy.spatial.transform import Rotation as R

def find_arduino_port() -> str|None:
    """List all available serial ports"""
    ports = list_ports.comports()
    
    if not ports:
        print("No serial ports found.")
        return None
    
    print(f"Found {len(ports)} serial port(s):")
    print("-" * 60)
    
    device_port = None
    for i, port in enumerate(ports, 1):
        print(f"{i}. Port: {port.device}")
        print(f"   Description: {port.description}")
        print(f"   Hardware ID: {port.hwid}")
        print()
        if (port.description.lower().find("arduino") != -1):
            device_port = port.device

    if device_port is not None:
        print(f"Arduino device found at port {device_port}") 
    
    return device_port


def send_angles(angles: list[float], chiselOn: bool, ser: serial.Serial):
    thetas = [0,0,0,0,0]
    thetas[0] = angles[0]  # base rotation
    thetas[1] = angles[1]  # shoulder  
    thetas[2] = angles[2]  # elbow
    thetas[3] = angles[3]  # wrist
    thetas[4] = 1 if chiselOn else 0  # chisel state (1 for on, 0 for off)
    """Send angles to the serial port"""
    if ser is not None:
        try:
            theta_Str = ','.join(str(int(round(theta))) for theta in thetas)
            theta_Str += ','
            ser.write(theta_Str.encode())
            print(f"Sent angles: {theta_Str}")
        except Exception as e:
            print(f"Failed to send angles: {e}")    
    else:
        print("Serial connection not established. Cannot send angles.")

def create_serial()->serial.Serial|None:
    print("Looking for Arduino")
    print("=" * 60)
    device_port = find_arduino_port()
    ser = None
    if device_port is not None:
        print(f"Attempting to connect to {device_port}...")
        try:
            ser = serial.Serial(device_port, 9600, timeout=1)
            time.sleep(2)  # Wait for the connection to establish
            print("Connected successfully!")
            input_str = ''
            #while not input_str.lower().startswith('exit'):
            #    input_str = input("Enter 'exit' to quit: ")
            #    ser.write(input_str.encode());
            return ser
        except Exception as e:
            print(f"Failed to connect: {e}")
            if ser is not None:
                ser.close()
    return None

def move_to(position: list[float], chisel_angle: float, chiselOn: bool, ser: serial.Serial):
    """Convert x, y, z position to servo angles using inverse kinematics and send"""
    try:
        x, y, z = position
        l1 = 20
        l2 = 14
        chisel = 17
        # Calculate servo angles using inverse kinematics
        phi = np.atan2(y, x) # base rotation angle
        
        r = np.sqrt(x**2 + y**2)
        # adjust r and z to account for the chisel length
        r = max(0, r - chisel * np.cos(np.radians(chisel_angle)))
        z = max(0, z - chisel * np.sin(np.radians(chisel_angle)))
        gamma = np.arccos((l1**2 + l2**2 - r**2 - z**2) / (2 * l1 * l2))

        theta2 = 180 - np.degrees(gamma)  # elbow angle
        beta = np.arctan2(z, r)
        alpha = np.arccos((l1**2 + r**2 + z**2 - l2**2) / (2 * l1 * np.sqrt(r**2 + z**2)))

        theta1 = np.degrees(alpha) + np.degrees(beta)  # shoulder angle

        servo_angles = [0, 0, 0, 0]
        servo_angles[0] = np.degrees(phi)  # base rotation angle
        servo_angles[1] = theta1  # shoulder angle
        servo_angles[2] = theta2  # elbow angle
        servo_angles[3] = chisel_angle  # wrist angle

        print(f"Position: x={x}, y={y}, z={z}")
        print(f"Servo angles: {[f'{angle:.2f}°' for angle in servo_angles]}")
        
        # Send angles to robot
        send_angles(servo_angles, chiselOn, ser)
        
    except Exception as e:
        print(f"Failed to calculate servo angles: {e}")


class PositionInputGUI:
    def __init__(self, root, ser: serial.Serial):
        self.root = root
        self.ser = ser
        self.root.title("Fossil Robot")
        self.root.geometry("800x600")
        self.root.resizable(False, False)
        
        # Initialize webcam
        self.available_cameras = self.get_available_cameras()
        self.selected_camera_var = tk.StringVar()
        default_camera = "2" if 2 in self.available_cameras else str(self.available_cameras[0])
        self.selected_camera_var.set(default_camera)
        self.cap = cv2.VideoCapture(int(default_camera))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        # Create main frame
        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=2)
        main_frame.columnconfigure(1, weight=1)
        
        # Left frame for webcam
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, rowspan=6, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10)
        
        # Webcam title row
        webcam_title_row = ttk.Frame(left_frame)
        webcam_title_row.pack(pady=(0, 10), fill=tk.X)

        webcam_title = ttk.Label(webcam_title_row, text="Camera", font=("Arial", 12, "bold"))
        webcam_title.pack(side=tk.LEFT)

        camera_options = [str(index) for index in self.available_cameras]
        self.camera_selector = ttk.Combobox(
            webcam_title_row,
            textvariable=self.selected_camera_var,
            values=camera_options,
            state="readonly",
            width=4,
        )
        self.camera_selector.pack(side=tk.LEFT, padx=(8, 0))
        self.camera_selector.bind("<<ComboboxSelected>>", self.on_camera_selected)
        
        self.webcam_label = ttk.Label(left_frame, background="black")
        self.webcam_label.pack()
        # Bind click event to webcam label
        self.webcam_label.bind("<Button-1>", self.on_webcam_click)
        
        # Label to display click coordinates
        self.click_label = ttk.Label(left_frame, text="Click coordinates: N/A", font=("Arial", 9))
        self.click_label.pack(pady=(5, 0))
        
        # Right frame for inputs
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=10)
        
        # Title label
        title_label = ttk.Label(right_frame, text="Enter Position (x, y, z)", font=("Arial", 12, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 15))
        
        # X input
        ttk.Label(right_frame, text="X:").grid(row=1, column=0, sticky=tk.W, pady=10)
        self.input_x = ttk.Entry(right_frame, width=15)
        self.input_x.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=10)
        self.input_x.insert(0, "30.0")
        # Y input
        ttk.Label(right_frame, text="Y:").grid(row=2, column=0, sticky=tk.W, pady=10)
        self.input_y = ttk.Entry(right_frame, width=15)
        self.input_y.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=10)
        self.input_y.insert(0, "0.0")
        
        # Z input with step controls
        ttk.Label(right_frame, text="Z:").grid(row=3, column=0, sticky=tk.W, pady=10)
        z_frame = ttk.Frame(right_frame)
        z_frame.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=10)
        self.input_z = ttk.Entry(z_frame, width=10)
        self.input_z.pack(side=tk.LEFT)
        self.input_z.insert(0, "25.0")
        z_minus_btn = ttk.Button(z_frame, text="-", width=3, command=self.decrement_z)
        z_minus_btn.pack(side=tk.LEFT, padx=(5, 2))
        z_plus_btn = ttk.Button(z_frame, text="+", width=3, command=self.increment_z)
        z_plus_btn.pack(side=tk.LEFT)

        # Chisel angle input
        ttk.Label(right_frame, text="Chisel Angle:").grid(row=4, column=0, sticky=tk.W, pady=10)
        self.input_chisel_angle = ttk.Entry(right_frame, width=15)
        self.input_chisel_angle.grid(row=4, column=1, sticky=(tk.W, tk.E), pady=10)
        self.input_chisel_angle.insert(0, "45.0")

        # Button frame (placed directly below Z input)
        button_frame = ttk.Frame(right_frame)
        button_frame.grid(row=5, column=0, columnspan=2, pady=15)
        
        # Move button
        submit_btn = ttk.Button(button_frame, text="Move", command=self.submit)
        submit_btn.pack(side=tk.LEFT, padx=5)
        
        # Home button
        clear_btn = ttk.Button(button_frame, text="Home", command=self.home)
        clear_btn.pack(side=tk.LEFT, padx=5)

        # Target color input for camera overlay
        ttk.Label(right_frame, text="Rock Color (#RRGGBB):").grid(row=6, column=0, sticky=tk.W, pady=10)
        self.input_color = ttk.Entry(right_frame, width=15)
        self.input_color.grid(row=6, column=1, sticky=(tk.W, tk.E), pady=10)
        self.input_color.insert(0, "#F08C8C")

        # Color tolerance slider
        self.color_tolerance_var = tk.IntVar(value=100)
        ttk.Label(right_frame, text="Tolerance:").grid(row=7, column=0, sticky=tk.W, pady=5)
        tolerance_frame = ttk.Frame(right_frame)
        tolerance_frame.grid(row=7, column=1, sticky=(tk.W, tk.E), pady=5)
        self.tolerance_slider = ttk.Scale(
            tolerance_frame,
            from_=0,
            to=255,
            orient=tk.HORIZONTAL,
            command=self.on_tolerance_change,
        )
        self.tolerance_slider.set(self.color_tolerance_var.get())
        self.tolerance_slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.tolerance_value_label = ttk.Label(tolerance_frame, text=str(self.color_tolerance_var.get()), width=4)
        self.tolerance_value_label.pack(side=tk.LEFT, padx=(6, 0))

        # Minimum region area slider
        self.min_region_area_var = tk.IntVar(value=150)
        ttk.Label(right_frame, text="Min Area:").grid(row=8, column=0, sticky=tk.W, pady=5)
        area_frame = ttk.Frame(right_frame)
        area_frame.grid(row=8, column=1, sticky=(tk.W, tk.E), pady=5)
        self.area_slider = ttk.Scale(
            area_frame,
            from_=0,
            to=5000,
            orient=tk.HORIZONTAL,
            command=self.on_min_area_change,
        )
        self.area_slider.set(self.min_region_area_var.get())
        self.area_slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.area_value_label = ttk.Label(area_frame, text=str(self.min_region_area_var.get()), width=4)
        self.area_value_label.pack(side=tk.LEFT, padx=(6, 0))

        # Morphology kernel size slider (odd values are used)
        self.kernel_size_var = tk.IntVar(value=5)
        ttk.Label(right_frame, text="Kernel:").grid(row=9, column=0, sticky=tk.W, pady=5)
        kernel_frame = ttk.Frame(right_frame)
        kernel_frame.grid(row=9, column=1, sticky=(tk.W, tk.E), pady=5)
        self.kernel_slider = ttk.Scale(
            kernel_frame,
            from_=1,
            to=21,
            orient=tk.HORIZONTAL,
            command=self.on_kernel_size_change,
        )
        self.kernel_slider.set(self.kernel_size_var.get())
        self.kernel_slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.kernel_value_label = ttk.Label(kernel_frame, text=str(self.kernel_size_var.get()), width=4)
        self.kernel_value_label.pack(side=tk.LEFT, padx=(6, 0))

        # Excavate button (below sliders)
        self.excavate_btn = ttk.Button(right_frame, text="Excavate!", command=self.excavate)
        self.excavate_btn.grid(row=10, column=0, columnspan=2, pady=(10, 5))
   
        # Status label
        self.status_label = ttk.Label(right_frame, text="", foreground="green", font=("Arial", 9))
        self.status_label.grid(row=11, column=0, columnspan=2, pady=10)
        
        # Store current frame for click detection
        self.current_frame = None
        self.frame_width = 320
        self.frame_height = 240
        self.color_tolerance = self.color_tolerance_var.get()
        self.min_region_area = self.min_region_area_var.get()
        self.kernel_size = self.kernel_size_var.get()
        self.mask_kernel = np.ones((self.kernel_size, self.kernel_size), dtype=np.uint8)
        self.last_filled_mask = None
        self._stop_excavate = threading.Event()
        self._excavate_thread: threading.Thread | None = None
        
        # Start webcam update
        self.update_webcam()

    def get_available_cameras(self, max_index: int = 10) -> list[int]:
        """Probe camera indexes and return the ones that can capture frames."""
        available: list[int] = []
        for index in range(max_index):
            probe = cv2.VideoCapture(index)
            if probe is not None and probe.isOpened():
                ok, _ = probe.read()
                if ok:
                    available.append(index)
            if probe is not None:
                probe.release()

        if not available:
            available.append(0)

        return available

    def set_camera(self, camera_index: int):
        """Switch to selected camera index."""
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()

        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    def on_camera_selected(self, _event=None):
        """Handle camera dropdown selection changes."""
        try:
            camera_index = int(self.selected_camera_var.get())
        except ValueError:
            return

        self.set_camera(camera_index)

    def on_tolerance_change(self, value):
        """Update tolerance from UI slider."""
        self.color_tolerance = int(float(value))
        self.color_tolerance_var.set(self.color_tolerance)
        if hasattr(self, "tolerance_value_label"):
            self.tolerance_value_label.config(text=str(self.color_tolerance))

    def on_min_area_change(self, value):
        """Update minimum contour area from UI slider."""
        self.min_region_area = int(float(value))
        self.min_region_area_var.set(self.min_region_area)
        if hasattr(self, "area_value_label"):
            self.area_value_label.config(text=str(self.min_region_area))

    def on_kernel_size_change(self, value):
        """Update morphology kernel size from UI slider."""
        kernel_size = int(float(value))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel_size = max(1, min(21, kernel_size))

        self.kernel_size = kernel_size
        self.kernel_size_var.set(self.kernel_size)
        if hasattr(self, "kernel_value_label"):
            self.kernel_value_label.config(text=str(self.kernel_size))
        self.mask_kernel = np.ones((self.kernel_size, self.kernel_size), dtype=np.uint8)

    def parse_target_color_bgr(self) -> tuple[int, int, int] | None:
        """Parse #RRGGBB input into a BGR tuple used by OpenCV."""
        color_text = self.input_color.get().strip()
        if color_text.startswith("#"):
            color_text = color_text[1:]

        if len(color_text) != 6:
            return None

        try:
            red = int(color_text[0:2], 16)
            green = int(color_text[2:4], 16)
            blue = int(color_text[4:6], 16)
        except ValueError:
            return None

        return (blue, green, red)
    
    def on_webcam_click(self, event):
        """Handle webcam click and set selected color from clicked pixel."""
        if self.current_frame is None:
            return

        label_width = max(1, self.webcam_label.winfo_width())
        label_height = max(1, self.webcam_label.winfo_height())

        x_click = int(np.clip(event.x, 0, label_width - 1))
        y_click = int(np.clip(event.y, 0, label_height - 1))

        frame_x = int(x_click * self.frame_width / label_width)
        frame_y = int(y_click * self.frame_height / label_height)
        frame_x = int(np.clip(frame_x, 0, self.frame_width - 1))
        frame_y = int(np.clip(frame_y, 0, self.frame_height - 1))

        blue, green, red = self.current_frame[frame_y, frame_x]
        selected_color = f"#{int(red):02X}{int(green):02X}{int(blue):02X}"

        self.input_color.delete(0, tk.END)
        self.input_color.insert(0, selected_color)

        print(
            f"Webcam click at (x={x_click}, y={y_click}) -> "
            f"frame (x={frame_x}, y={frame_y}), color={selected_color}"
        )
        self.click_label.config(text=f"Click: x={frame_x}, y={frame_y}, color={selected_color}")
    
    def update_webcam(self):
        """Update webcam feed display"""
        ret, frame = self.cap.read()
        if ret:
            # Store current frame for click detection
            self.current_frame = frame
            self.frame_height, self.frame_width = frame.shape[:2]
            self.last_filled_mask = np.zeros((self.frame_height, self.frame_width), dtype=np.uint8)

            # Highlight regions near selected color
            target_bgr = self.parse_target_color_bgr()
            if target_bgr is not None:
                target = np.array(target_bgr, dtype=np.int16)
                diff = frame.astype(np.int16) - target
                distance = np.sqrt(np.sum(diff * diff, axis=2))
                mask = (distance < self.color_tolerance).astype(np.uint8) * 255

                # Fill gaps and remove noise in the thresholded mask
                closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.mask_kernel, iterations=2)
                cleaned_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, self.mask_kernel, iterations=1)

                contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                filled_mask = np.zeros_like(cleaned_mask)
                valid_contours = []

                for contour in contours:
                    if cv2.contourArea(contour) >= self.min_region_area:
                        valid_contours.append(contour)
                        cv2.drawContours(filled_mask, [contour], -1, 255, thickness=cv2.FILLED)

                overlay = frame.copy()
                overlay[filled_mask > 0] = (0, 255, 0)
                frame = cv2.addWeighted(frame, 0.6, overlay, 0.4, 0)
                self.last_filled_mask = filled_mask

                if valid_contours:
                    cv2.drawContours(frame, valid_contours, -1, (0, 0, 255), 2)
            
            # Convert BGR to RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Convert to PIL Image
            image = Image.fromarray(frame)
            # Convert to PhotoImage
            photo = ImageTk.PhotoImage(image)
            # Update label
            self.webcam_label.config(image=photo)
            self.webcam_label.image = photo
        
        # Schedule next update (30ms = ~33 FPS)
        self.root.after(30, self.update_webcam)

    def excavate(self):
        """Start excavation in a background thread."""
        if self._excavate_thread is not None and self._excavate_thread.is_alive():
            return  # already running

        if self.last_filled_mask is None:
            print("Excavate: no mask available yet.")
            return

        indices = np.argwhere(self.last_filled_mask > 0)
        if indices.size == 0:
            print("Excavate: no points found (filled_mask > 0 is empty).")
            return

        self._stop_excavate.clear()
        self.excavate_btn.config(text="Stop", command=self.stop_excavate)

        self._excavate_thread = threading.Thread(
            target=self._excavate_worker, args=(indices,), daemon=True
        )
        self._excavate_thread.start()

    def stop_excavate(self):
        """Signal the running excavation to stop."""
        self._stop_excavate.set()

    def _excavate_worker(self, indices):
        """Background worker: iterate mask points until done or stopped."""
        try:
            center_x = float(self.input_x.get())
            center_y = float(self.input_y.get())
            center_z = float(self.input_z.get())
            chisel_angle = float(self.input_chisel_angle.get())
            i_range = 320
            j_range = 240
            y_range = 14
            x_range = 9
            num_points = len(indices)
            print(f"Excavate: found {len(indices)} point(s).")
            ctr = 1
            for i, j in indices:
                self.status_label.config(text=f"Excavating point {ctr} of {num_points}...", foreground="yellow")
                print(f"i={int(i)}, j={int(j)}")
                move_to([center_x + (i - i_range/2) * x_range / i_range, center_y + (j - j_range/2) * y_range / j_range, center_z+1.0], chisel_angle, True, self.ser)
                time.sleep(0.5)  # pause to allow movement
                if self._stop_excavate.is_set():
                    print("Excavate: stopped.")
                    self.status_label.config(text="Excavate: stopped.", foreground="red")
                    move_to([center_x + (i - i_range/2) * x_range / i_range, center_y + (j - j_range/2) * y_range / j_range, center_z+2.0], chisel_angle, False, self.ser)
                    break
                move_to([center_x + (i - i_range/2) * x_range / i_range, center_y + (j - j_range/2) * y_range / j_range, center_z-1.0], chisel_angle, True, self.ser)
                # Sleep in small increments so stop is responsive
                for _ in range(50):
                    if self._stop_excavate.is_set():
                        break
                    time.sleep(0.1)
            else:
                print("Excavate: complete.")
                self.status_label.config(text="Excavate: complete.", foreground="green")
        finally:
            self.root.after(0, self._on_excavate_done)

    def _on_excavate_done(self):
        """Restore the Excavate button after the worker finishes."""
        self.excavate_btn.config(text="Excavate!", command=self.excavate)


    def adjust_z_and_submit(self, delta: float):
        """Apply a Z step and immediately submit the new position."""
        try:
            current_z = float(self.input_z.get())
        except ValueError:
            messagebox.showerror("Error", "Z value must be a valid floating-point number")
            return

        new_z = current_z + delta
        self.input_z.delete(0, tk.END)
        self.input_z.insert(0, f"{new_z:.1f}")
        self.submit()

    def increment_z(self):
        """Increase Z by 0.5 and submit."""
        self.adjust_z_and_submit(0.5)

    def decrement_z(self):
        """Decrease Z by 0.5 and submit."""
        self.adjust_z_and_submit(-0.5)
    
    def submit(self):
        """Handle submit button click"""
        try:
            x = float(self.input_x.get())
            y = float(self.input_y.get())
            z = float(self.input_z.get())
            chisel_angle = float(self.input_chisel_angle.get())  # Validate chisel angle input
            position = [x, y, z]
            print(f"Submitting position: x={x}, y={y}, z={z}")
            move_to(position, chisel_angle, False, self.ser)
            self.status_label.config(text="Angles sent successfully!", foreground="green")
        except ValueError:
            messagebox.showerror("Error", "Please enter valid floating-point numbers")
            self.status_label.config(text="Error: Invalid input", foreground="red")
    
    def home(self):
        """Clear all input fields"""
        self.input_x.delete(0, tk.END)
        self.input_y.delete(0, tk.END)
        self.input_z.delete(0, tk.END)
        self.input_x.insert(0, "30.0")
        self.input_y.insert(0, "0.0")
        self.input_z.insert(0, "25.0")
        self.submit()

def main():
    ser = create_serial()
    root = tk.Tk()
    app = PositionInputGUI(root, ser)
    
    def on_closing():
        """Handle window closing"""
        app.cap.release()
        if ser is not None:
            ser.close()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()

import tkinter as tk
from tkinter import ttk, messagebox
import math

import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from robot import Robot


class RobotSimulatorApp:

    def __init__(self, root):
        self.root = root
        self.root.title("Differential Drive Robot Simulator")
        self.root.geometry("1150x720")
        self.root.minsize(950, 600)

        self.dt = 0.05
        self.animation_delay = 30  # Délai entre les images, en ms

        self.animation_id = None
        self.current_frame = 0
        self.trajectory = []

        self.create_interface()
        self.draw_empty_graph()

    # --------------------------------------------------
    # INTERFACE
    # --------------------------------------------------

    def create_interface(self):

        main_frame = ttk.Frame(self.root, padding=15)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Panneau des paramètres
        control_frame = ttk.LabelFrame(
            main_frame,
            text="Robot parameters",
            padding=15
        )
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 15))

        self.entries = {}

        parameters = [
            ("Initial orientation (deg)", "90"),
            ("Wheel radius (cm)", "3"),
            ("Wheel base (cm)", "20"),
            ("Left wheel speed (rad/s)", "2"),
            ("Right wheel speed (rad/s)", "2"),
            ("Simulation time (s)", "5")
        ]

        for label_text, default_value in parameters:

            label = ttk.Label(control_frame, text=label_text)
            label.pack(anchor=tk.W, pady=(8, 2))

            entry = ttk.Entry(control_frame, width=25)
            entry.insert(0, default_value)
            entry.pack(fill=tk.X, pady=(0, 5))

            self.entries[label_text] = entry

        # Boutons
        self.start_button = ttk.Button(
            control_frame,
            text="▶ Start simulation",
            command=self.start_simulation
        )
        self.start_button.pack(fill=tk.X, pady=(20, 5))

        self.pause_button = ttk.Button(
            control_frame,
            text="⏸ Pause",
            command=self.pause_simulation,
            state=tk.DISABLED
        )
        self.pause_button.pack(fill=tk.X, pady=5)

        self.reset_button = ttk.Button(
            control_frame,
            text="↻ Reset",
            command=self.reset_simulation
        )
        self.reset_button.pack(fill=tk.X, pady=5)

        # Informations
        info_frame = ttk.LabelFrame(
            control_frame,
            text="Robot state",
            padding=10
        )
        info_frame.pack(fill=tk.X, pady=(20, 5))

        self.position_label = ttk.Label(
            info_frame,
            text="Position: (0.00, 0.00) cm"
        )
        self.position_label.pack(anchor=tk.W, pady=3)

        self.orientation_label = ttk.Label(
            info_frame,
            text="Orientation: 90.00°"
        )
        self.orientation_label.pack(anchor=tk.W, pady=3)

        self.time_label = ttk.Label(
            info_frame,
            text="Time: 0.00 s"
        )
        self.time_label.pack(anchor=tk.W, pady=3)

        self.status_label = ttk.Label(
            control_frame,
            text="Ready",
            wraplength=230
        )
        self.status_label.pack(anchor=tk.W, pady=(15, 0))

        # Panneau du graphique
        graph_frame = ttk.LabelFrame(
            main_frame,
            text="Robot trajectory",
            padding=10
        )
        graph_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(7, 6), dpi=100)
        self.ax = self.figure.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(
            self.figure,
            master=graph_frame
        )
        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True
        )

    # --------------------------------------------------
    # GRAPHIQUE INITIAL
    # --------------------------------------------------

    def draw_empty_graph(self):

        self.ax.clear()

        self.ax.set_title("Differential Drive Robot")
        self.ax.set_xlabel("X position (cm)")
        self.ax.set_ylabel("Y position (cm)")

        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.set_aspect("equal", adjustable="datalim")

        self.ax.plot(
            0, 0,
            marker="o",
            markersize=8,
            label="Initial position"
        )

        self.ax.legend()
        self.canvas.draw_idle()

    # --------------------------------------------------
    # LECTURE DES PARAMÈTRES
    # --------------------------------------------------

    def get_parameters(self):

        try:
            theta_deg = float(
                self.entries["Initial orientation (deg)"].get()
            )

            wheel_radius = float(
                self.entries["Wheel radius (cm)"].get()
            )

            wheel_base = float(
                self.entries["Wheel base (cm)"].get()
            )

            omega_left = float(
                self.entries["Left wheel speed (rad/s)"].get()
            )

            omega_right = float(
                self.entries["Right wheel speed (rad/s)"].get()
            )

            duration = float(
                self.entries["Simulation time (s)"].get()
            )

        except ValueError:
            raise ValueError(
                "Enter valid numerical values for all parameters."
            )

        if wheel_radius <= 0:
            raise ValueError("Wheel radius must be positive.")

        if wheel_base <= 0:
            raise ValueError("Wheel base must be positive.")

        if duration <= 0:
            raise ValueError("Simulation time must be positive.")

        return (
            math.radians(theta_deg),
            wheel_radius,
            wheel_base,
            omega_left,
            omega_right,
            duration
        )

    # --------------------------------------------------
    # PRÉPARATION DE LA SIMULATION
    # --------------------------------------------------

    def prepare_simulation(self):

        (
            theta,
            wheel_radius,
            wheel_base,
            self.omega_left,
            self.omega_right,
            duration
        ) = self.get_parameters()

        self.duration = duration

        # Le robot démarre à l'origine
        self.robot = Robot(
            x=0,
            y=0,
            theta_orientation=theta,
            wheel_radius=wheel_radius,
            wheel_base=wheel_base
        )

        self.initial_theta = theta

        # Calcul de la trajectoire
        self.trajectory = []

        total_steps = max(1, math.ceil(duration / self.dt))

        for step in range(total_steps + 1):

            self.trajectory.append((
                self.robot.x_position,
                self.robot.y_position,
                self.robot.theta_orientation,
                min(step * self.dt, duration)
            ))

            if step < total_steps:
                current_dt = min(
                    self.dt,
                    duration - step * self.dt
                )

                self.robot.update_pose(
                    self.omega_left,
                    self.omega_right,
                    current_dt
                )

        # Restaurer le robot à son état initial
        self.robot = Robot(
            x=0,
            y=0,
            theta_orientation=theta,
            wheel_radius=wheel_radius,
            wheel_base=wheel_base
        )

        self.current_frame = 0

    # --------------------------------------------------
    # AFFICHAGE DE LA TRAJECTOIRE
    # --------------------------------------------------

    def setup_animation(self):

        self.ax.clear()

        self.ax.set_title("Differential Drive Robot")
        self.ax.set_xlabel("X position (cm)")
        self.ax.set_ylabel("Y position (cm)")

        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.set_aspect("equal", adjustable="datalim")

        x_values = [state[0] for state in self.trajectory]
        y_values = [state[1] for state in self.trajectory]

        # Déterminer les limites du graphique
        all_x = x_values + [0]
        all_y = y_values + [0]

        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)

        span = max(
            x_max - x_min,
            y_max - y_min,
            10
        )

        margin = span * 0.2 + 2

        self.ax.set_xlim(
            x_min - margin,
            x_max + margin
        )

        self.ax.set_ylim(
            y_min - margin,
            y_max + margin
        )

        # Position de départ
        self.ax.plot(
            0, 0,
            marker="o",
            markersize=7,
            label="Start"
        )

        # Trajectoire qui sera dessinée progressivement
        self.path_line, = self.ax.plot(
            [], [],
            linewidth=2,
            label="Trajectory"
        )

        # Point représentant le centre du robot
        self.robot_point, = self.ax.plot(
            [], [],
            marker="o",
            markersize=10,
            linestyle="None",
            label="Robot"
        )

        # Flèche indiquant l'orientation du robot
        self.heading_arrow = None

        self.ax.legend(loc="best")
        self.canvas.draw_idle()

    # --------------------------------------------------
    # DÉMARRAGE
    # --------------------------------------------------

    def start_simulation(self):

        self.stop_animation()

        try:
            self.prepare_simulation()

        except ValueError as error:
            messagebox.showerror(
                "Invalid parameters",
                str(error)
            )
            return

        self.setup_animation()

        self.start_button.config(state=tk.DISABLED)
        self.pause_button.config(state=tk.NORMAL)

        self.status_label.config(text="Simulation running...")

        self.animate()

    # --------------------------------------------------
    # ANIMATION
    # --------------------------------------------------

    def animate(self):

        if self.current_frame >= len(self.trajectory):
            self.finish_simulation()
            return

        x, y, theta, elapsed_time = self.trajectory[
            self.current_frame
        ]

        # Afficher le chemin parcouru jusqu'à cet instant
        path = self.trajectory[:self.current_frame + 1]

        path_x = [state[0] for state in path]
        path_y = [state[1] for state in path]

        self.path_line.set_data(path_x, path_y)

        # Position actuelle du robot
        self.robot_point.set_data([x], [y])

        # Supprimer l'ancienne flèche
        if self.heading_arrow is not None:
            self.heading_arrow.remove()

        # Longueur de la flèche d'orientation
        arrow_length = max(
            self.robot.wheel_base * 0.7,
            2
        )

        # Nouvelle flèche
        self.heading_arrow = self.ax.annotate(
            "",
            xy=(
                x + arrow_length * math.cos(theta),
                y + arrow_length * math.sin(theta)
            ),
            xytext=(x, y),
            arrowprops=dict(
                arrowstyle="->",
                linewidth=2
            )
        )

        # Mise à jour des informations
        self.position_label.config(
            text=f"Position: ({x:.2f}, {y:.2f}) cm"
        )

        theta_deg = math.degrees(theta)

        self.orientation_label.config(
            text=f"Orientation: {theta_deg:.2f}°"
        )

        self.time_label.config(
            text=f"Time: {elapsed_time:.2f} s"
        )

        self.canvas.draw_idle()

        self.current_frame += 1

        self.animation_id = self.root.after(
            self.animation_delay,
            self.animate
        )

    # --------------------------------------------------
    # PAUSE
    # --------------------------------------------------

    def pause_simulation(self):

        if self.animation_id is not None:

            self.root.after_cancel(self.animation_id)
            self.animation_id = None

            self.pause_button.config(
                text="▶ Resume",
                command=self.resume_simulation
            )

            self.status_label.config(text="Simulation paused.")

    def resume_simulation(self):

        self.pause_button.config(
            text="⏸ Pause",
            command=self.pause_simulation
        )

        self.status_label.config(text="Simulation running...")

        self.animate()

    # --------------------------------------------------
    # FIN DE SIMULATION
    # --------------------------------------------------

    def finish_simulation(self):

        self.stop_animation()

        self.start_button.config(state=tk.NORMAL)
        self.pause_button.config(
            state=tk.DISABLED,
            text="⏸ Pause",
            command=self.pause_simulation
        )

        self.status_label.config(text="Simulation completed.")

    # --------------------------------------------------
    # ARRÊT ET RÉINITIALISATION
    # --------------------------------------------------

    def stop_animation(self):

        if self.animation_id is not None:

            self.root.after_cancel(self.animation_id)
            self.animation_id = None

    def reset_simulation(self):

        self.stop_animation()

        self.current_frame = 0
        self.trajectory = []

        self.start_button.config(state=tk.NORMAL)

        self.pause_button.config(
            state=tk.DISABLED,
            text="⏸ Pause",
            command=self.pause_simulation
        )

        self.position_label.config(
            text="Position: (0.00, 0.00) cm"
        )

        self.orientation_label.config(
            text="Orientation: 0.00°"
        )

        self.time_label.config(
            text="Time: 0.00 s"
        )

        self.status_label.config(text="Ready")

        self.draw_empty_graph()


# ------------------------------------------------------
# PROGRAMME PRINCIPAL
# ------------------------------------------------------

if __name__ == "__main__":

    root = tk.Tk()

    app = RobotSimulatorApp(root)

    root.mainloop()
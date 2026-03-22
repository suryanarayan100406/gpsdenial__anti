import subprocess
import tkinter as tk
from tkinter import messagebox


class SimulationLauncherApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title('Drone Simulation Launcher')
        self.root.geometry('460x180')
        self.root.resizable(False, False)

        self.status_var = tk.StringVar(value='Ready')

        title = tk.Label(
            root,
            text='GPS-Denied Drone Navigation Demo',
            font=('Segoe UI', 13, 'bold')
        )
        title.pack(pady=(18, 8))

        run_btn = tk.Button(
            root,
            text='Run Simulation',
            font=('Segoe UI', 11, 'bold'),
            width=20,
            height=2,
            command=self.run_simulation,
            bg='#2e7d32',
            fg='white',
            activebackground='#1b5e20',
            activeforeground='white'
        )
        run_btn.pack(pady=6)

        status = tk.Label(root, textvariable=self.status_var, font=('Segoe UI', 10))
        status.pack(pady=(10, 0))

    def run_simulation(self) -> None:
        try:
            check_cmd = (
                "pgrep -af '/home/surya/webots/webots|webots_ros2_driver/driver|"
                "drone_nav_launch.py'"
            )
            check_result = subprocess.run(
                ['wsl.exe', 'bash', '-lc', check_cmd],
                capture_output=True,
                text=True,
            )

            if check_result.returncode == 0 and check_result.stdout.strip():
                self.status_var.set('Simulation already running')
                messagebox.showinfo('Simulation Running', 'A simulation process is already active.')
                return

            launch_cmd = (
                "source /opt/ros/jazzy/setup.bash; "
                "source /mnt/c/Users/samai/Desktop/drone/install/setup.bash; "
                "export WEBOTS_HOME=/home/surya/webots; "
                "cd /mnt/c/Users/samai/Desktop/drone; "
                "ros2 launch drone_nav_2d drone_nav_launch.py "
                "world_profile:=realistic bag_output:=bags/presentation_run"
            )

            subprocess.Popen(
                ['wsl.exe', 'bash', '-lc', launch_cmd],
                creationflags=subprocess.CREATE_NEW_CONSOLE,
            )

            self.status_var.set('Simulation started (new terminal opened)')
        except Exception as exc:
            self.status_var.set('Failed to start simulation')
            messagebox.showerror('Launch Error', f'Could not start simulation:\n{exc}')


if __name__ == '__main__':
    app_root = tk.Tk()
    SimulationLauncherApp(app_root)
    app_root.mainloop()

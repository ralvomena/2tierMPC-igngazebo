import tkinter as tk
from tkinter import ttk
from datetime import datetime

class SupervisorGui:
    """
    Class that builds the Supervisor GUI with Tkinter.
    """
    def __init__(self, master, node_instance):
        master.title('Supervisor GUI')
        master.columnconfigure(0, weight=1)

        self.node = node_instance

        self.agv_list = []
        self.mpc_row = 1
        self.priority_row = 1
        self.tree_inserts = {}

        self.main = ttk.Frame(master)
        self.main.grid()

        top_label = ttk.Label(self.main, text="AGVs in Gazebo")
        top_label.grid(column=0, row=0, columnspan=5, padx=5, pady=5)

        columns = ('agv_id', 'pose', 'velocity', 'priority', 'mpc_edge', 'mpc_local')
        self.agv_tree = ttk.Treeview(self.main, columns=columns, show='headings')

        self.agv_tree.column("# 1", anchor="center", stretch="no", width=75) # width=190
        self.agv_tree.column("# 2", anchor="center", stretch="no", width=200)
        self.agv_tree.column("# 3", anchor="center", stretch="no", width=230)
        self.agv_tree.column("# 4", anchor="center", stretch="no", width=100)
        self.agv_tree.column("# 5", anchor="center", stretch="no", width=100)
        self.agv_tree.column("# 6", anchor="center", stretch="no", width=100)

        self.agv_tree.heading('agv_id', text='AGV ID')
        self.agv_tree.heading('pose', text='Pose [x(m), y(m), \u03B8(\u00B0)]')
        self.agv_tree.heading('velocity', text='Velocity [v(m/s), \u03C9(rad/s)]')
        self.agv_tree.heading('priority', text='Priority')
        self.agv_tree.heading('mpc_edge', text='Edge MPC')
        self.agv_tree.heading('mpc_local', text='Local MPC')

        self.agv_tree.grid(row=1, column=0, columnspan=5)

        self.scroll_agv_tree = ttk.Scrollbar(self.main, orient='vertical', command=self.agv_tree.yview)
        self.scroll_agv_tree.grid(column=5, row=1, sticky='ns')
        self.agv_tree.configure(yscrollcommand=self.scroll_agv_tree.set)

        separator = ttk.Separator(self.main, orient='horizontal')
        separator.grid(column=0, row=2, columnspan=6,sticky=tk.NSEW, padx=5, pady=5)

        mpc_button = ttk.Button(self.main, text ="MPC Service", command = self.open_mpc_window)
        mpc_button.grid(column=0, row=3, columnspan=2, sticky="EW", padx=5, pady=5)

        priority_button = ttk.Button(self.main, text ="Set priorities", command=self.open_prioriry_window)
        priority_button.grid(column=2, row=3, sticky="EW", padx=5, pady=5)

        scn_button = ttk.Button(self.main, text ="Start scenario", command=self.node.start_scenario)
        scn_button.grid(column=3, row=3, sticky="EW", padx=5, pady=5)

        stop_scn_button = ttk.Button(self.main, text ="Stop", command=self.node.stop_scenario)
        stop_scn_button.grid(column=4, row=3, columnspan=2, sticky="EW", padx=5, pady=5)

        separator = ttk.Separator(self.main, orient='horizontal')
        separator.grid(column=0, row=4, columnspan=6,sticky=tk.NSEW, padx=5, pady=5)

        info_label = ttk.Label(self.main, text="Information panel")
        info_label.grid(column=0, row=5, columnspan=5)

        self.info_text = tk.Text(self.main)
        self.info_text.grid(column=0, columnspan=5, row=6, sticky="NSEW")
        
        self.scroll_info = ttk.Scrollbar(self.main, orient='vertical', command=self.info_text.yview)
        self.scroll_info.grid(column=5, row=6, sticky='ns')
        self.info_text.configure(yscrollcommand=self.scroll_info.set)

    def insert_agv_to_tree(self, agv, pose, velocity, priority, mpc_edge_status, mpc_local_status):
            x = str(round(pose[0], 2))
            y = str(round(pose[1], 2))
            linear = str(round(velocity[0], 2))
            angular = str(round(velocity[1], 2))
            theta = str(round(pose[2], 2))
            insert = self.agv_tree.insert('', 'end', values=(agv.upper(), f'[{x}, {y}, {theta}]', f'[{linear}, {angular}]', priority, mpc_edge_status, mpc_local_status))
            self.tree_inserts[agv] = insert
            self.agv_list.append(agv)

    def open_mpc_window(self):
	
        self.mpc_window = tk.Toplevel(self.main)
        self.mpc_window.columnconfigure(0, weight=0)
        self.mpc_window.title("MPC Service")
    
        for agv in self.agv_list:
            self.add_mpc_entry(agv)

    def open_prioriry_window(self):
        self.priority_window = tk.Toplevel(self.main)
        self.priority_window.columnconfigure(0, weight=0)
        self.priority_window.title("Ajustar prioridades")

        for agv in self.agv_list:
            self.add_priority_entry(agv)

    def add_mpc_entry(self, agv):
        agv_label = ttk.Label(self.mpc_window, text=agv.upper() + ':')
        agv_label.grid(row=self.mpc_row, column = 0, sticky="W", padx=5, pady=5) # sticky="W", padx=5, pady=5

        x_label = ttk.Label(self.mpc_window, text="x(m)")
        x_label.grid(row=self.mpc_row, column = 1, sticky="W", padx=5, pady=5)

        x = ttk.Entry(self.mpc_window, width=5)
        x.grid(row=self.mpc_row, column = 2, sticky="EW", padx=5, pady=5)

        y_label = ttk.Label(self.mpc_window, text="y(m)")
        y_label.grid(row=self.mpc_row, column = 3, sticky="W", padx=5, pady=5)

        y = ttk.Entry(self.mpc_window, width=5)
        y.grid(row=self.mpc_row, column = 4, sticky="EW", padx=5, pady=5)

        theta_label = ttk.Label(self.mpc_window, text='\u03B8(\u00B0)')
        theta_label.grid(row=self.mpc_row, column = 5, sticky="W", padx=5, pady=5)

        theta = ttk.Entry(self.mpc_window, width=5)
        theta.grid(row=self.mpc_row, column = 6, sticky="EW", padx=5, pady=5)

        call = tk.Button(self.mpc_window, text= "Call", command=lambda:
                        self.mpc_service_call(agv, 'move', x.get(), y.get(), theta.get()))
        call.grid(row=self.mpc_row, column = 7, sticky="EW", padx=5, pady=5)

        stop = tk.Button(self.mpc_window, text= "Stop", command=lambda:self.stop_mpc(agv))
        stop.grid(row=self.mpc_row, column = 8, sticky="EW", padx=5, pady=5)

        self.mpc_row += 1

    def add_priority_entry(self, agv):
        agv_label = ttk.Label(self.priority_window, text=agv.upper() + ':')
        agv_label.grid(row=self.priority_row, column = 0, sticky="W", padx=5, pady=5) # sticky="W", padx=5, pady=5

        selected_priority = tk.StringVar()
        priority = ttk.Combobox(self.priority_window, textvariable=selected_priority)
        priority ["values"] = ("Low", "Medium", "High")
        priority ["state"] = "readonly"  # "normal is the counterpart"
        priority.current(2)
        priority.grid(row=self.priority_row, column = 1, sticky="W", padx=5, pady=5)

        set = tk.Button(self.priority_window, text= "Set", command=lambda:self.set_priority(agv, selected_priority.get()))
        set.grid(row=self.priority_row, column = 2, sticky="EW", padx=5, pady=5)

        self.priority_row += 1

    def mpc_service_call(self, agv, action, x, y, theta):
        for agv_instance in self.node.agv_instances:
            if agv_instance.agv_id == agv:
                agv_instance.call_mpc(action, float(x), float(y), float(theta))
                return None

    def stop_mpc(self, agv):
        for agv_instance in self.node.agv_instances:
            if agv_instance.agv_id == agv:
                agv_instance.call_mpc('stop', 0.0, 0.0, 0.0)
                self.info_text.insert(tk.END, f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] Stopping {agv.upper()}.\n")
                return None

    def set_priority(self, agv, priority):
        if priority == 'High':
            priority = 'high'
        elif priority == 'Medium':
            priority = 'medium'
        else:
            priority = 'low'

        self.node.set_agv_priority(agv, priority)
        self.info_text.insert(tk.END, f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] {agv.upper()} priority set to {priority}.\n")

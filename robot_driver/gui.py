# gui.py
import customtkinter as ctk
import sys, io, threading, time
from backend import ODriveManager
from PIL import Image, ImageTk
import numpy as np
from customtkinter import CTkImage
import math
import serial



# ========== redirect stdout / stderr ==========
class TextRedirector(io.TextIOBase):
    def __init__(self, text_widget, tag="stdout"):
        self.text_widget = text_widget
        self.tag = tag
    def write(self, s):
        self.text_widget.configure(state="normal")
        if self.tag == "stderr":
            self.text_widget.insert("end", s, "error")
        else:
            self.text_widget.insert("end", s)
        self.text_widget.see("end")
        self.text_widget.update_idletasks()   # force redraw
        self.text_widget.configure(state="disabled")
    def flush(self): ...

    
    

# ========== GUI ==========
class ODriveGUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("ODrive CAN GUI (Linux Only)")
        self.geometry("1850x900+25+75")
        ctk.set_default_color_theme("blue")
        ctk.set_widget_scaling(1.1)

        self.latest = {}  # cache
        self.status_running = False # bg poller flag    

        # === keyboard jogging state ===
        self.jog_target = {0: 0.0, 1: 0.0, 2: 0.0}
        self.keyboard_enabled = False

        self.font_header = ("Roboto", 14)
        self.font_button = ("Roboto", 14)  # , 'bold'
        self.font_label = ("Roboto", 14)
        self.font_label_bold = ("Roboto", 15)

        self.last_valid_cv_time = time.time()

        self.mgr = ODriveManager()
        self.heartbeat_running = False
        self.error_running = False

        self.cv_pipeline = None


        # Canonical mapping: which CV joint controls which ODrive node
        self.node0 = "right_elbow"
        self.node1 = "left_shoulder"
        self.node2 = "left_elbow"
        self.node3 = "right_shoulder"

        # Single source of truth
        self.node_to_cv = {
            0: self.node0,
            1: self.node1,
            2: self.node2,
            3: self.node3
        }

        self.node_to_cv_inv = {
            self.node0: 0,
            self.node1: 1,
            self.node2: 2,
        }

        # Reverse lookup, used when you have a joint name and need the node id
        self.cv_to_node = {name: node for node, name in self.node_to_cv.items()}

        # Mapping CV joint names → URDF joint names used in pybullet  (diff format to self.node_to_cv)
        # No node 3 in urdf
        self.joint_map = {
            "right_elbow":  "node0",   
            "left_shoulder": "node1",
            "left_elbow":    "node2",
        }

        # individual node follow-state flags
        self.node_follow = {
            0: False,
            1: False,
            2: False,
            3: False
        }


        # ---------- main frame ----------
        main = ctk.CTkFrame(self)
        main.pack(padx=20, pady=10, fill="both", expand=True)

        RIGHT_W = 450  # pick what you like (e.g., 480–600)

        # Left takes remaining space, right is fixed
        main.grid_columnconfigure(0, weight=1)             # left grows
        main.grid_columnconfigure(1, weight=0, minsize=RIGHT_W)  # right fixed

        # Left column uses tabs
        self.tabview = ctk.CTkTabview(main)
        self.tabview.grid(row=0, column=0, sticky="nsew", padx=(0,10))

        # Create tabs
        self.tab_cv  = self.tabview.add("CV Driver")
        self.tab_pos = self.tabview.add("Control")
        self.tab_vel = self.tabview.add("Velocity Config")


        # Wrap each tab in a full-width frame (keeps tab width consistent)
        self.left_col = ctk.CTkFrame(self.tab_pos)
        self.left_col.pack(fill="both", expand=True, padx=10, pady=10)

        self.vel_col = ctk.CTkFrame(self.tab_vel)
        self.vel_col.pack(fill="both", expand=True, padx=10, pady=10)

        # Right column (fixed width)
        self.right_col = ctk.CTkFrame(main, width=RIGHT_W)
        self.right_col.grid(row=0, column=1, sticky="ns", padx=(10,0))  # no 'ew' so it won't stretch
        self.right_col.grid_propagate(False)  # keep width at RIGHT_W regardless of contents




###### I. CV Tab ######
        # Frame inside CV tab
        self.cv_col = ctk.CTkFrame(self.tab_cv)
        self.cv_col.pack(fill="both", expand=True, padx=10, pady=0)

        # ---------- CV Control Buttons ----------
        cv_btn_frame = ctk.CTkFrame(self.cv_col)
        cv_btn_frame.pack(fill="x", pady=0)

        # Start CV (green)
        self.cv_start_btn = ctk.CTkButton(
            cv_btn_frame,
            text="System Start",
            font=self.font_button,
            width=200,
            fg_color="#26714A",       # green
            hover_color="#1E5A3A",
            command=self.threaded(self.start_cv)
        )
        self.cv_start_btn.pack(side="left", padx=(100,20), expand=True)

        # Shutdown CV (red)
        self.cv_stop_btn = ctk.CTkButton(
            cv_btn_frame,
            text="Shutdown",
            font=self.font_button,
            width=200,
            fg_color="#A93226",       # red
            hover_color="#922B21",
            command=self.threaded(self.stop_cv)
        )
        self.cv_stop_btn.pack(side="left", padx=(20,100), expand=True)


        # ---------- 3-Column Layout Below Buttons ----------
        cv_columns = ctk.CTkFrame(self.cv_col)
        cv_columns.pack(fill="both", expand=True, padx=10, pady=0)

        # 3 equal columns, same width
        cv_columns.grid_columnconfigure(0, weight=1, uniform="cvcols")
        cv_columns.grid_columnconfigure(1, weight=1, uniform="cvcols")
        cv_columns.grid_columnconfigure(2, weight=1, uniform="cvcols")

        cv_columns.grid_rowconfigure(0, weight=1)


        ########### Left column: CV webcam output
        self.cv_cam_frame = ctk.CTkFrame(cv_columns, fg_color="gray18", corner_radius=5)
        self.cv_cam_frame.grid(row=0, column=0, padx=10, pady=4, sticky="nsew")

        self.cv_cam_frame.grid_rowconfigure(0, weight=0)   # header
        self.cv_cam_frame.grid_rowconfigure(1, weight=0)   # webcam image
        self.cv_cam_frame.grid_rowconfigure(2, weight=0)   # angle table
        self.cv_cam_frame.grid_columnconfigure(0, weight=1)

        # Left column header
        self.cv_cam_header = ctk.CTkLabel(self.cv_cam_frame, text="CV", font=self.font_label_bold)
        self.cv_cam_header.grid(row=0, column=0, sticky="ew", pady=(5, 5))

        # ---------- Webcam embed-ready label ----------
        # webcam image
        self.cv_cam_image = None
        self.cv_cam_label = ctk.CTkLabel(self.cv_cam_frame, text="")
        self.cv_cam_label.grid(row=1, column=0, padx=10, pady=(0, 0), sticky="n")
        self.cv_cam_label.configure(width=333, height=250)


        # ---------- CV Angle Readouts ----------
        self.cv_angle_frame = ctk.CTkFrame(self.cv_cam_frame, fg_color="gray14", corner_radius=8)
        self.cv_angle_frame.grid(row=2, column=0, padx=10, pady=(5, 10), sticky="ew")

        # Configure 3 clean columns
        self.cv_angle_frame.grid_columnconfigure(0, weight=1)   # angle name
        self.cv_angle_frame.grid_columnconfigure(1, weight=1)   # value
        self.cv_angle_frame.grid_columnconfigure(2, weight=1)   # mapped joint

        # ---- Header Row ----
        ctk.CTkLabel(
            self.cv_angle_frame,
            text="CV Arm Joint",
            font=self.font_label_bold
        ).grid(row=0, column=0, sticky="w", padx=10, pady=(4, 6))

        ctk.CTkLabel(
            self.cv_angle_frame,
            text="Angle",
            font=self.font_label_bold
        ).grid(row=0, column=1, sticky="w", padx=10, pady=(4, 6))

        ctk.CTkLabel(
            self.cv_angle_frame,
            text="Mapped Node",
            font=self.font_label_bold
        ).grid(row=0, column=2, sticky="w", padx=10, pady=(4, 6))


        # ---- RIGHT ELBOW ROW : Node 0----

        ctk.CTkLabel(self.cv_angle_frame, text=self.pretty(self.node0), font=self.font_label)\
            .grid(row=1, column=0, sticky="w", padx=10, pady=4)

        self.cv_angle_node0_val = ctk.CTkLabel(self.cv_angle_frame, text="--°", font=self.font_label)
        self.cv_angle_node0_val.grid(row=1, column=1, sticky="w", padx=10, pady=4)

        ctk.CTkLabel(self.cv_angle_frame, text="→   Node 0", font=self.font_label)\
            .grid(row=1, column=2, sticky="w", padx=10, pady=4)


        # ---- LEFT SHOLDER ROW : Node 1 ----
        ctk.CTkLabel(self.cv_angle_frame, text=self.pretty(self.node1), font=self.font_label)\
            .grid(row=2, column=0, sticky="w", padx=10, pady=4)

        self.cv_angle_node1_val = ctk.CTkLabel(self.cv_angle_frame, text="--°", font=self.font_label)
        self.cv_angle_node1_val.grid(row=2, column=1, sticky="w", padx=10, pady=4)

        ctk.CTkLabel(self.cv_angle_frame, text="→   Node 1", font=self.font_label)\
            .grid(row=2, column=2, sticky="w", padx=10, pady=4)


        # ---- LEFT ELBOW ROW : Node  2----
        ctk.CTkLabel(self.cv_angle_frame, text=self.pretty(self.node2), font=self.font_label)\
            .grid(row=3, column=0, sticky="w", padx=10, pady=4)

        self.cv_angle_node2_val = ctk.CTkLabel(self.cv_angle_frame, text="--°", font=self.font_label)
        self.cv_angle_node2_val.grid(row=3, column=1, sticky="w", padx=10, pady=4)

        ctk.CTkLabel(self.cv_angle_frame, text="→   Node 2", font=self.font_label)\
            .grid(row=3, column=2, sticky="w", padx=10, pady=4)
        

        # ---- RIGHT SHOULDER ROW : Node  3----
        ctk.CTkLabel(self.cv_angle_frame, text=self.pretty(self.node3), font=self.font_label)\
            .grid(row=4, column=0, sticky="w", padx=10, pady=4)

        self.cv_angle_node3_val = ctk.CTkLabel(self.cv_angle_frame, text="--°", font=self.font_label)
        self.cv_angle_node3_val.grid(row=4, column=1, sticky="w", padx=10, pady=4)

        ctk.CTkLabel(self.cv_angle_frame, text="→   Node 3", font=self.font_label)\
            .grid(row=4, column=2, sticky="w", padx=10, pady=4)




        ########## Middle column: PyBullet renderer
        self.cv_coldetech_frame = ctk.CTkFrame(cv_columns, fg_color="gray18", corner_radius=5)
        self.cv_coldetech_frame.grid(row=0, column=1, padx=10, pady=4, sticky="nsew")

        self.cv_coldetech_frame.grid_rowconfigure(0, weight=0)
        self.cv_coldetech_frame.grid_rowconfigure(1, weight=0)
        self.cv_coldetech_frame.grid_rowconfigure(2, weight=0)   # new joint-label row
        self.cv_coldetech_frame.grid_rowconfigure(3, weight=1)   # report box grows
        self.cv_coldetech_frame.grid_columnconfigure(0, weight=1)


        # Middle column header
        self.cv_coldetech_header = ctk.CTkLabel(self.cv_coldetech_frame, text="Collision Detection", font=self.font_label_bold)
        self.cv_coldetech_header.grid(row=0, column=0, sticky="ew", pady=(5, 5))

        # ---------- PyBullet embed-ready label ----------
        self.cv_coldetech_image = None
        self.cv_coldetech_label = ctk.CTkLabel(self.cv_coldetech_frame, text="")
        self.cv_coldetech_label.grid(row=1, column=0, padx=10, pady=(0, 0), sticky="n")
        self.cv_coldetech_label.configure(width=640, height=350)

        # ---------- Node angle debug line (between sim and report) ----------
        self.coldetec_joint_label = ctk.CTkLabel(
            self.cv_coldetech_frame,
            text="Node 0: --°   |   Node 1: --°   |   Node 2: --°\nNode 3: n/a",
            font=self.font_label,
            anchor="center",
        )
        self.coldetec_joint_label.grid(
            row=2, column=0,
            padx=10, pady=(5, 0),
            sticky="ew",
        )

        # ---------- Collision report box (under sim window) ----------
        self.collision_report_box = ctk.CTkTextbox(
            self.cv_coldetech_frame,
            width=333,
            height=80,
        )
        self.collision_report_box.grid(
            row=3, column=0,
            padx=10, pady=(5, 10),
            sticky="nsew",
        )
        self.collision_report_box.insert("1.0", "Collision report will appear here.")
        self.collision_report_box.configure(state="disabled")


        # ---------- Startup Black Images ----------
        blank = np.zeros((350, 640, 3), dtype=np.uint8)
        self.update_webcam_frame(blank)
        self.update_sim_frame(blank)



        ############ Right column: Status / text panel
        self.cv_motorcoms_frame = ctk.CTkFrame(cv_columns, fg_color="gray18", corner_radius=5)
        self.cv_motorcoms_frame.grid(row=0, column=2, padx=10, pady=4, sticky="nsew")

        # rows: 0 header, 1 axis state buttons, 2 zero/home controls, 3 command table
        self.cv_motorcoms_frame.grid_rowconfigure(0, weight=0)
        self.cv_motorcoms_frame.grid_rowconfigure(1, weight=0)
        self.cv_motorcoms_frame.grid_rowconfigure(2, weight=0)
        self.cv_motorcoms_frame.grid_rowconfigure(3, weight=0)
        self.cv_motorcoms_frame.grid_rowconfigure(4, weight=1)
        self.cv_motorcoms_frame.grid_columnconfigure(0, weight=1)

        # Right column header
        self.cv_motorcoms_header = ctk.CTkLabel(
            self.cv_motorcoms_frame,
            text="Motor Commands",
            font=self.font_label_bold
        )
        self.cv_motorcoms_header.grid(row=0, column=0, sticky="ew", pady=(5, 5))


        # === Follow CV big toggle button ===
        self.cv_follow_state = False    # internal flag

        self.cv_follow_btn = ctk.CTkButton(
            self.cv_motorcoms_frame,
            text="follow CV: off",
            width=180,
            height=40,
            fg_color="#4a4a4a",       # gray when OFF
            hover_color="#5c5c5c",
            font=self.font_button,
            command=self._on_cv_follow_pressed
        )

        self.cv_follow_btn.grid(row=1, column=0, padx=10, pady=(0, 10), sticky="ew")

        # Sub-frame for node follow buttons (4 columns)
        self.node_follow_frame = ctk.CTkFrame(self.cv_motorcoms_frame, fg_color="transparent")
        self.node_follow_frame.grid(row=2, column=0, padx=10, pady=(0, 10), sticky="ew")

        # Allow 4 equal columns inside this frame
        for c in range(4):
            self.node_follow_frame.grid_columnconfigure(c, weight=1)

        self.node_follow_btns = {}

        for i in range(4):
            btn = ctk.CTkButton(
                self.node_follow_frame,  
                text=f"Node {i}",
                width=60,
                height=28,
                fg_color="#4a4a4a",
                hover_color="#5c5c5c",
                font=self.font_button_small if hasattr(self, "font_button_small") else self.font_button,
                command=lambda idx=i: self._on_node_follow_pressed(idx)
            )
            btn.grid(row=0, column=i, padx=4, pady=2, sticky="ew")
            self.node_follow_btns[i] = btn




        # === Axis state controls (Idle / Closed loop) ===
        axis_frame = ctk.CTkFrame(self.cv_motorcoms_frame, fg_color="transparent")
        axis_frame.grid(row=3, column=0, padx=10, pady=(0, 10), sticky="ew")
        axis_frame.grid_columnconfigure(0, weight=1)
        axis_frame.grid_columnconfigure(1, weight=1)

        idle_btn_cv = ctk.CTkButton(
            axis_frame,
            text="Idle",
            fg_color="#272727",
            hover_color="#525252",
            width=120,
            command=self.threaded(lambda: self.set_axis_state_all(1)),
            font=self.font_button,
        )
        idle_btn_cv.grid(row=0, column=0, padx=(0, 10), pady=5, sticky="ew")

        loop_btn_cv = ctk.CTkButton(
            axis_frame,
            text="Closed-loop",
            fg_color="#2F9962",
            hover_color="#37A76D",
            width=120,
            command=self.threaded(lambda: self.set_axis_state_all(8)),
            font=self.font_button,
        )
        loop_btn_cv.grid(row=0, column=1, padx=(10, 0), pady=5, sticky="ew")



        # === Zero encoders and home controls ===
        zero_frame_cv = ctk.CTkFrame(self.cv_motorcoms_frame, fg_color="transparent")
        zero_frame_cv.grid(row=4, column=0, padx=10, pady=(0, 10), sticky="ew")

        # -------- Row 0: Zero Node 0,1,2 (unchanged style) --------
        for i in range(4):
            btn = ctk.CTkButton(
                zero_frame_cv,
                text=f"Zero {i}",
                width=75,
                command=self.threaded(lambda n=i: self.zero_node(n)),
                font=self.font_button,
            )
            btn.grid(row=0, column=i, padx=5, pady=5, sticky="ew")

        # small spacer so the row stretches nicely
        zero_frame_cv.grid_columnconfigure(3, weight=1)

        # -------- Row 1: Zero All | Go Home (centered pair) --------
        row1_frame = ctk.CTkFrame(zero_frame_cv, fg_color="transparent")
        row1_frame.grid(row=1, column=0, columnspan=4, pady=(2, 5), sticky="ew")

        # columns: [spacer] [Zero All] [Go Home] [spacer]
        row1_frame.grid_columnconfigure(0, weight=1)
        row1_frame.grid_columnconfigure(1, weight=0)
        row1_frame.grid_columnconfigure(2, weight=0)
        row1_frame.grid_columnconfigure(3, weight=1)

        btn_zero_all_cv = ctk.CTkButton(
            row1_frame,
            text="Zero All",
            fg_color="#1650CC",
            hover_color="#1650CC",
            width=110,
            command=self.threaded(self.zero_all),
            font=self.font_button,
        )
        btn_zero_all_cv.grid(row=0, column=1, padx=5, sticky="w")

        btn_go_home_cv = ctk.CTkButton(
            row1_frame,
            text="Go Home",
            fg_color="#7D3C98",
            hover_color="#693284",
            width=110,
            command=self.threaded(self.go_home),
            font=self.font_button,
        )
        btn_go_home_cv.grid(row=0, column=2, padx=5, sticky="w")



        # === Command table: node, target deg, target turns ===
        cmd_table = ctk.CTkFrame(self.cv_motorcoms_frame, fg_color="gray14", corner_radius=6)
        cmd_table.grid(row=5, column=0, padx=10, pady=(0, 10), sticky="nsew")

        cmd_table.grid_columnconfigure(0, weight=1)
        cmd_table.grid_columnconfigure(1, weight=1)
        cmd_table.grid_columnconfigure(2, weight=1)

        # Header row
        ctk.CTkLabel(cmd_table, text="Node", font=self.font_label_bold, anchor="w")\
            .grid(row=0, column=0, padx=8, pady=(6, 4), sticky="w")
        ctk.CTkLabel(cmd_table, text="Target (deg)", font=self.font_label_bold, anchor="w")\
            .grid(row=0, column=1, padx=8, pady=(6, 4), sticky="w")
        ctk.CTkLabel(cmd_table, text="Target (turns)", font=self.font_label_bold, anchor="w")\
            .grid(row=0, column=2, padx=8, pady=(6, 4), sticky="w")

        self.cv_cmd_deg_labels = {}
        self.cv_cmd_turn_labels = {}

        for node_id in range(4):   # nodes 0,1,2,3
            # Node label
            ctk.CTkLabel(
                cmd_table,
                text=f"Node {node_id}",
                font=self.font_label,
                anchor="w",
            ).grid(row=node_id + 1, column=0, padx=8, pady=2, sticky="w")

            # Target in degrees (processed CV → robot)
            deg_lbl = ctk.CTkLabel(
                cmd_table,
                text="--°",
                font=self.font_label,
                anchor="w",
            )
            deg_lbl.grid(row=node_id + 1, column=1, padx=8, pady=2, sticky="w")
            self.cv_cmd_deg_labels[node_id] = deg_lbl

            # Target in turns (for node0 1 2) or deg only for node3
            if node_id == 3:
                txt = "n/a"
            else:
                txt = "--"

            turn_lbl = ctk.CTkLabel(
                cmd_table,
                text=txt,
                font=self.font_label,
                anchor="w",
            )
            turn_lbl.grid(row=node_id + 1, column=2, padx=8, pady=2, sticky="w")
            self.cv_cmd_turn_labels[node_id] = turn_lbl




###### II. Control Tab ######
        # ---------- 1. Connect ----------
        sec1 = ctk.CTkFrame(self.left_col)
        sec1.pack(fill="x", pady=10)
        # CAN connect text
        ctk.CTkLabel( sec1, text="1. Connect to CAN (enter password in terminal if prompted):", font=self.font_header).pack(side="left", padx=10)
        # CAN connect button
        ctk.CTkButton( sec1, text="Connect", command=self.threaded(self.connect_can), font=self.font_button ).pack(side="left", padx=60)

        # ----------- Gimbal UI additions -----------
        import serial.tools.list_ports

        # Find available COM ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if len(ports) == 0:
            ports = ["No Ports"]  # fallback so dropdown isn't empty

        # Default to /dev/ttyACM0 when present so users don't have to pick it manually.
        default_port = next((p for p in ports if "ACM0" in p), None)
        if default_port is None:
            default_port = ports[0]

        # Dropdown for serial ports
        self.gimbal_port_var = ctk.StringVar(value=default_port)
        self.gimbal_port_dropdown = ctk.CTkOptionMenu(
            sec1,
            variable=self.gimbal_port_var,
            values=ports,
            width=140,
            fg_color="gray25",
            button_color="gray30",
            button_hover_color="gray20"
        )
        self.gimbal_port_dropdown.pack(side="left", padx=10)

        # Connect gimbal button
        ctk.CTkButton(
            sec1,
            text="Connect Gimbal",
            command=self.threaded(self.connect_gimbal),
            font=self.font_button,
            fg_color="#1650CC",
            hover_color="#0F3A99"
        ).pack(side="left", padx=10)


        # ---------- 2. Enumerate ----------
        sec2 = ctk.CTkFrame(self.left_col)
        sec2.pack(fill="x", pady=10)
        ctk.CTkLabel(sec2, text="2. Enumerate ODrives (discover nodes):", font=self.font_header).pack(side="left", padx=10)
        ctk.CTkButton(sec2, text="Enumerate", command=self.threaded(self.enumerate_nodes), font=self.font_button).pack(side="left", padx=15)

        ctk.CTkLabel(sec2, text="(Optional) Calibrate:", font=self.font_header).pack(side="left", padx=(35,10))

        for node_id in [0, 1, 2]:
            ctk.CTkButton(
                sec2,
                text=f"Calibrate Node {node_id}",
                command=self.threaded(lambda nid=node_id: self.calibrate_one(nid)),
                font=self.font_button
            ).pack(side="left", padx=5)

        # ---------- 3. Axis State ----------   
        sec3 = ctk.CTkFrame(self.left_col)
        sec3.pack(fill="x", pady=10)

        ctk.CTkLabel(sec3, text="3. Set Axis State:", font=self.font_header).grid(row=0, column=0, padx=10, sticky="w")

        # custom input + apply button
        self.axis_state_entry = ctk.CTkEntry(sec3, width=80)
        self.axis_state_entry.grid(row=0, column=1, padx=5)

        apply_btn = ctk.CTkButton(
            sec3,
            text="Apply Custom",
            font=self.font_button,
            command=self.threaded(self.set_axis_state_all)
        )
        apply_btn.grid(row=0, column=2, padx=10)


        # small spacer
        ctk.CTkLabel(sec3, text="      ").grid(row=0, column=3, padx=5)

        # IDLE button (state = 1)
        idle_btn = ctk.CTkButton(
            sec3,
            text="Idle",
            fg_color="#272727",
            hover_color="#525252",
            width=150,
            command=self.threaded(lambda: self.set_axis_state_all(1)),
            font=self.font_button
        )
        idle_btn.grid(row=0, column=4, padx=(50,40))

        # CLOSED LOOP button (state = 8)
        loop_btn = ctk.CTkButton(
            sec3,
            text="Closed-loop",
            fg_color="#2F9962",  # a green tone
            hover_color="#37A76D",
            width=150,
            command=self.threaded(lambda: self.set_axis_state_all(8)),
            font=self.font_button
        )
        loop_btn.grid(row=0, column=5, padx=10)


        # ---------- 4. Zero Encoders ----------
        sec4 = ctk.CTkFrame(self.left_col)
        sec4.pack(fill="x", pady=10)
        ctk.CTkLabel(sec4, text="4. Zero Encoder Offsets (per node):", font=self.font_header).grid(row=0, column=0, padx=10, sticky="w")

        self.zero_offsets = {}
        self.zero_buttons = {}

        for i in range(3):
            btn = ctk.CTkButton(sec4, text=f"Zero Node {i}", command=self.threaded(lambda n=i: self.zero_node(n)), font=self.font_button)
            btn.grid(row=0, column=i+1, padx=10)
            self.zero_buttons[i] = btn

        btn_all = ctk.CTkButton(
            sec4,
            text="Zero All",
            fg_color="#1650CC",      # blue
            hover_color="#1650CC",   # darker blue
            font=self.font_button,
            command=self.threaded(self.zero_all)
        )
        btn_all.grid(row=0, column=5, padx=10)   # same row as the others

        go_home_btn = ctk.CTkButton(
            sec4,
            text="Go Home",
            fg_color="#7D3C98",    # purple
            hover_color="#693284",
            font=self.font_button,
            command=self.threaded(self.go_home)
        )
        go_home_btn.grid(row=0, column=6, padx=10)



        # ---------- 5. Position Control ----------
        sec5 = ctk.CTkFrame(self.left_col)
        sec5.pack(fill="x", pady=10)
        ctk.CTkLabel(sec5, text="5. Position Control (relative turns):", font=self.font_header).grid(row=0, column=0, padx=5, sticky="w")

        self.pos_entries = {}
        for i in range(3):
            ctk.CTkLabel(sec5, text=f"Node {i}:").grid(row=0, column=2*i+1, padx=5)
            e = ctk.CTkEntry(sec5, width=80)
            e.grid(row=0, column=2*i+2, padx=5)
            self.pos_entries[i] = e

        ctk.CTkButton(sec5, text="Send Control", command=self.threaded(self.send_positions), font=self.font_button, width=50).grid(row=0, column=8, padx=5)


        # ---------- 5b. Gimbal Position (degrees) ----------
        ctk.CTkLabel(sec5, text="Gimbal angle (deg):").grid(row=0, column=9, padx=(40,5), pady=5, sticky="w")

        self.gimbal_deg_entry = ctk.CTkEntry(sec5, width=80)
        self.gimbal_deg_entry.grid(row=0, column=10, padx=5)

        ctk.CTkButton(
            sec5,
            text="Send Gimbal Angle",
            command=self.threaded(self._send_gimbal_angle_from_entry),
            font=self.font_button
        ).grid(row=0, column=11, padx=10)


        # ---------- 6. Error Handling ----------
        sec6 = ctk.CTkFrame(self.left_col)
        sec6.pack(fill="x", pady=10)
        ctk.CTkLabel(sec6, text="6. Error Listening / Clearing:", font=self.font_header).pack(side="left", padx=10)
        self.error_btn = ctk.CTkButton(sec6, text="Start Listening", command=self.toggle_error_listener)
        self.error_btn.pack(side="left", padx=5)
        ctk.CTkButton(sec6, text="Clear All Errors", command=self.threaded(self.clear_errors), font=self.font_button).pack(side="left", padx=5)

        # Add an error display area below buttons
        self.error_output = ctk.CTkTextbox(sec6, height=80, width=420, font=("Consolas", 12))
        self.error_output.pack(fill="x", padx=10, pady=5)
        self.error_output.insert("end", "Error listener inactive.\n")
        self.error_output.configure(state="disabled")

        # ---------- 7. Shutdown ----------
        sec7 = ctk.CTkFrame(self.left_col)
        sec7.pack(fill="x", pady=10)
        ctk.CTkLabel(sec7, text="7. Shutdown CAN connection when finished:", font=self.font_header).pack(side="left", padx=10)
        ctk.CTkButton(sec7, text="Shutdown", command=self.threaded(self.shutdown_can), font=self.font_button).pack(side="left", padx=10)

        # ---------- Log box at bottom ----------
        self.log_box = ctk.CTkTextbox(self, width=800, height=350)
        self.log_box.pack(padx=20, pady=10, fill="x")
        self.log_box.tag_config("error", foreground="red")

        # make it read-only for user typing
        self.log_box.configure(state="disabled")
        self.log_box.bind("<Key>", lambda e: "break")  # ignore keypresses in this widget

        sys.stdout = TextRedirector(self.log_box, "stdout")
        sys.stderr = TextRedirector(self.log_box, "stderr")


        ###### Right Column Status Panel ######
        # ---------- Right Column Status Panels ----------
        self.right_col.grid_rowconfigure("all", weight=0)
        self.right_col.grid_columnconfigure(0, weight=1)

        # === 1. Connection Section ===
        sec_r1 = ctk.CTkFrame(self.right_col)
        sec_r1.pack(fill="x", pady=10)
        ctk.CTkLabel(sec_r1, text="Connection Status", font=self.font_header).grid(row=0, column=0, sticky="w", padx=10)
        self.conn_status = ctk.CTkLabel(sec_r1, text="🔌 Disconnected", text_color="gray", font=self.font_header)
        self.conn_status.grid(row=0, column=1, sticky="e", padx=10)

        # === 2. Enumerate Section ===
        sec_r2 = ctk.CTkFrame(self.right_col)
        sec_r2.pack(fill="x", pady=10)
        ctk.CTkLabel(sec_r2, text="Enumerate",font=self.font_header).grid(row=0, column=0, sticky="w", padx=10)
        self.enum_status = ctk.CTkLabel(sec_r2, text="Not Enumerated", text_color="gray", font=self.font_header)
        self.enum_status.grid(row=0, column=1, sticky="e", padx=10)

        # === 3. Zero Reference Section (all inline) ===
        sec_r3 = ctk.CTkFrame(self.right_col)
        sec_r3.pack(fill="x", pady=10)
        ctk.CTkLabel(sec_r3, text="Zero References",font=self.font_header).grid(row=0, column=0, sticky="w", padx=10)
        self.zero_status = {}
        for i in range(3):
            lbl = ctk.CTkLabel(sec_r3, text=f"Node {i}: ---", text_color="gray", font=self.font_header)
            lbl.grid(row=0, column=i+1, padx=10, sticky="w")
            self.zero_status[i] = lbl

        # === 4. Heartbeat Section (two-column layout) ===
        sec_r4 = ctk.CTkFrame(self.right_col)
        sec_r4.pack(fill="x", pady=10)

        # Header
        ctk.CTkLabel(sec_r4, text="Heartbeat", font=self.font_header).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=10
        )

        # ---- LEFT COLUMN: Per-node State ----
        self.state_labels = {}
        for i in range(3):
            lbl = ctk.CTkLabel(
                sec_r4,
                text=f"Node {i}: —",
                font=self.font_header,
                text_color="gray"
            )
            lbl.grid(row=i+1, column=0, sticky="nsew", padx=15, pady=10)
            self.state_labels[i] = lbl

        # ---- RIGHT COLUMN: Heartbeat per node ----
        self.heartbeat_labels = {}
        for i in range(3):
            lbl = ctk.CTkLabel(
                sec_r4,
                text=f"Node {i}: state=--- err=---",
                text_color="gray",
                font=self.font_header
            )
            lbl.grid(row=i+1, column=1, sticky="w", padx=20, pady=2)
            self.heartbeat_labels[i] = lbl

        # make columns expand evenly
        sec_r4.grid_columnconfigure(0, weight=1)
        sec_r4.grid_columnconfigure(1, weight=3)

        # === 5. Feedback (pos / vel) Section (one per node) ===
        sec_r5 = ctk.CTkFrame(self.right_col)
        sec_r5.pack(fill="x", pady=10)
        ctk.CTkLabel(sec_r5, text="Node Feedback (pos & vel)", font=self.font_header).grid(row=0, column=0, sticky="w", padx=10)
        self.feedback_labels = {}
        for i in range(3):
            lbl = ctk.CTkLabel(sec_r5, text=f"Node {i}: pos=--- vel=---", text_color="gray", font=self.font_header)
            lbl.grid(row=i+1, column=0, columnspan=2, sticky="w", padx=20, pady=2)
            self.feedback_labels[i] = lbl



###### III. Velocity Config TAB ######
        # ---------- Velocity Config TAB ----------
        sec_vel = ctk.CTkFrame(self.vel_col)
        sec_vel.pack(fill="x", pady=10)

        ctk.CTkLabel(sec_vel, text="Velocity Control Parameters:", font=self.font_header)\
            .pack(anchor="w", padx=10, pady=(5,10))

        # === per-node selector ===
        node_sel_frame = ctk.CTkFrame(sec_vel)
        node_sel_frame.pack(anchor="w", padx=20, pady=(0,10))

        ctk.CTkLabel(node_sel_frame, text="Select Node:", font=self.font_label)\
            .pack(side="left", padx=(0,5))
        
        # --- horizontal 4-option tab selector ---
        self.node_choice_val = "All"

        tab_container = ctk.CTkFrame(node_sel_frame, fg_color="transparent")
        tab_container.pack(side="left", padx=5)

        self.node_tab_buttons = {}

        def set_node_choice(val):
            self.node_choice_val = val
            for key, b in self.node_tab_buttons.items():
                if key == val:
                    b.configure(fg_color="#1f6aa5", text_color="white")  # selected
                else:
                    b.configure(fg_color="#2b2b2b", text_color="gray80")  # unselected

        for opt in ["All", "0", "1", "2"]:
            btn = ctk.CTkButton(
                tab_container,
                text=opt,
                width=50,
                height=28,
                corner_radius=6,
                fg_color="#2b2b2b",
                text_color="gray80",
                command=lambda o=opt: set_node_choice(o),
            )
            btn.pack(side="left", padx=3)
            self.node_tab_buttons[opt] = btn

        set_node_choice("All")

        # === parameter inputs ===
        params = [
            ("Vel Limit [turn/s]", "vel_limit", 2.0),
            ("Vel Limit Tolerance", "vel_limit_tolerance", 1e9),
            ("Position Gain (pos_gain)", "pos_gain", "250"),  # no default
            ("Velocity Gain (vel_gain)", "vel_gain", "0.375"),  # no default
            ("Velocity Integrator Gain (vel_integrator_gain)", "vel_integrator_gain", "0.9"),  # no default
            ("Torque Soft Min", "torque_soft_min", -1e9),
            ("Torque Soft Max", "torque_soft_max", 1e9),
        ]
        self.vel_param_entries = {}

        form = ctk.CTkFrame(sec_vel)
        form.pack(anchor="w", padx=20, pady=10)

        for i, (label, key, default) in enumerate(params):
            ctk.CTkLabel(form, text=label, font=self.font_label, width=200, anchor="w")\
                .grid(row=i, column=0, padx=5, pady=5, sticky="w")
            entry = ctk.CTkEntry(form, width=120)
            entry.insert(0, str(default))
            entry.grid(row=i, column=1, padx=10, pady=5)
            self.vel_param_entries[key] = entry
        
        

        # === Buttons ===
        btn_frame = ctk.CTkFrame(sec_vel)
        btn_frame.pack(fill="x", padx=10, pady=15)

        ctk.CTkButton(btn_frame, text="Push Config", 
                    command=self.threaded(self.push_vel_config), font=self.font_button)\
                    .pack(side="left", padx=5)

        ctk.CTkButton(btn_frame, text="Save + Reboot", 
                    command=self.threaded(self.save_and_reboot), font=self.font_button)\
                    .pack(side="left", padx=5)

        ctk.CTkButton(btn_frame, text="Calibrate Selected", 
                    command=self.threaded(self.calibrate_selected_node), font=self.font_button)\
                    .pack(side="left", padx=5)
        



        # keyboard handlers
        self.keys_down = set()
        self.bind_all("<KeyPress>", self.on_key_press)
        self.bind_all("<KeyRelease>", self.on_key_release)

        # jog loop
        self.after(30, self.keyboard_control_loop)




    def push_vel_config(self):
        """Generate and push configs for selected or all nodes."""
        node_sel = self.node_choice_val
        if node_sel == "All":
            nodes = getattr(self.mgr, "nodes", [0, 1, 2])
        else:
            nodes = [int(node_sel)]

        vals = {}
        for k, entry in self.vel_param_entries.items():
            raw = entry.get()
            print(f"[DEBUG] {k} raw='{raw}' repr={repr(raw)}")
            
            raw = entry.get().strip()
            if raw == "":
                vals[k] = None
            else:
                try:
                    vals[k] = float(raw)
                except ValueError:
                    print(f"⚠️ Invalid value for {k}: '{raw}' (ignored)")
                    vals[k] = None

        # push config over CAN
        self.mgr.flash_config_over_can(
            nodes=nodes,
            vel_limit=vals["vel_limit"],
            vel_tol=vals["vel_limit_tolerance"],
            torque_min=vals["torque_soft_min"],
            torque_max=vals["torque_soft_max"],
            pos_gain=vals.get("pos_gain"),
            vel_gain=vals.get("vel_gain"),
            vel_int_gain=vals.get("vel_integrator_gain"),
        )

    def save_and_reboot(self):
        """Send SAVE_CONFIG and REBOOT over CAN."""
        node_sel = self.node_choice_val

        # determine which nodes
        if node_sel == "All":
            nodes = getattr(self.mgr, "nodes", [0,1,2])
        else:
            nodes = [int(node_sel)]

        print(f"💾 Saving + rebooting node(s): {nodes}")
        self.mgr.save_and_reboot_nodes(nodes)

    def calibrate_selected_node(self):
        """Calibrate selected or all nodes using existing calibrate_one()."""
        node_sel = self.node_choice_val
        if node_sel == "All":
            print("🧭 Calibrating ALL nodes...")
            for n in getattr(self.mgr, "nodes", []):
                self.calibrate_one(n)
        else:
            node_id = int(node_sel)
            print(f"🧭 Calibrating node {node_id}...")
            self.calibrate_one(node_id)


    # ============================================================
    # threaded wrapper
    # ============================================================
    def threaded(self, fn):
        def run_safe():
            try:
                fn()
            except Exception:
                import traceback
                traceback.print_exc()
        return lambda: threading.Thread(target=run_safe, daemon=True).start()

    # ============================================================
    # backend actions
    # ============================================================
    def connect_can(self):
        self.shutdown_can()
        # prevent spamming if already connected
        if getattr(self.mgr, "bus", None) is not None:
            print("⚠️ CAN already connected — please shutdown before reconnecting.")
            return

        # attempt connection
        ok = self.mgr.setup_can_interface()   # returns True/False

        if ok:
            self.conn_status.configure(text="✅ Connected", text_color="green")
            self.status_running = True
            self.mgr.start_listener() 
            self.poll_heartbeat()
            self.update_feedback_display()
            print("✅ CAN connection established successfully.")
        else:
            self.conn_status.configure(text="❌ Failed to Connect", text_color="red")
            self.status_running = False
            print("⚠️ CAN connection failed. A connection may be active — shutdown and retry.")


    def enumerate_nodes(self):
        self.mgr.enumerate_odrives()
        nodes_text = ", ".join(f"Node {n}" for n in self.mgr.nodes)
        self.enum_status.configure(text=f"Enumerated: {nodes_text}", text_color="gray")

    def calibrate_one(self, node_id):
        """Calibrate a specific node."""
        print(f"⚙️ Calibrating node {node_id}...")
        self.mgr.calibrate_node(node_id)

    def send_positions(self):
        """Move to absolute position in the zeroed frame: target_abs = zero + delta."""
        for node_id, entry in self.pos_entries.items():
            val = entry.get().strip()
            if not val:
                continue
            try:
                delta = float(val)

                # 1) get zero; if missing, use latest known pos
                zero = self.zero_offsets.get(node_id)
                if zero is None:
                    zero = self.latest.get(node_id, {}).get("pos", 0.0)
                    self.zero_offsets[node_id] = zero
                    print(f"ℹ️ Node {node_id} had no zero; using cached pos={zero:.3f}")
                    if node_id in self.zero_status:
                        self.zero_status[node_id].configure(
                            text=f"Node {node_id}: {zero:.3f} turns", text_color="orange"
                        )

                # 2) compute target and send    immediately
                target_abs = zero + delta
                self.mgr.set_position(node_id, target_abs)

                # 3) update cache and GUI log
                self.latest.setdefault(node_id, {}).update({
                    "pos": target_abs,
                    "updated": time.time(),
                })
                print(f"🎯 Node {node_id}: abs_target={target_abs:.3f} (zero={zero:.3f}, Δ={delta:+.3f})")

            except ValueError:
                print(f"⚠️ Invalid input for node {node_id}")
            except Exception as e:
                print(f"⚠️ Error moving node {node_id}: {e}")




    def clear_errors(self):
        for n in self.mgr.nodes:
            self.mgr.clear_errors_and_idle(n)


    def go_home(self):
        """Send all enumerated nodes to their relative home: abs_target = zero_offsets[node]."""
        nodes = getattr(self.mgr, "nodes", [])

        if not nodes:
            print("⚠️ No nodes enumerated. Cannot go home.")
            return

        print("🏠 Go Home command issued...")

        for n in nodes:
            try:
                # get the stored zero; if missing, fall back to latest pos
                zero = self.zero_offsets.get(n)
                if zero is None:
                    zero = self.latest.get(n, {}).get("pos", 0.0)
                    self.zero_offsets[n] = zero
                    print(f"ℹ️ Node {n} had no zero; using cached pos={zero:.3f}")

                print(f"   → Node {n}: moving to home (abs_target={zero:.3f})")
                self.mgr.set_position(n, zero)

                # update the cached latest position
                self.latest.setdefault(n, {}).update({
                    "pos": zero,
                    "updated": time.time(),
                })

            except Exception as e:
                print(f"⚠️ Failed to home node {n}: {e}")


    def shutdown_can(self):
        self.mgr.shutdown_can_interface()
        self.conn_status.configure(text="🔌 Disconnected", text_color="gray")
        self.status_running = False


    def set_axis_state_all(self, state=None):
        """Set all nodes to given axis state (1=IDLE, 8=CLOSED_LOOP)."""
        try:
            # if called manually, ask from entry box
            if state is None:
                val = self.axis_state_entry.get().strip()
                if not val:
                    print("⚠️ No axis state entered.")
                    return
                state = int(val)

            for n in getattr(self.mgr, "nodes", []):
                self.mgr.set_axis_state(n, state)
                print(f"✅ Node {n} set to state {state}")


            ## Keyboard jogging ish

            # >>> ADD THIS <<<  
            # --- Keyboard jogging init (safe) ---
            if state == 8:  # entering closed loop
                print("Entering closed-loop, waiting for stable feedback before enabling jogging...")

                def init_jog_targets_safely():
                    ready = True
                    for n in getattr(self.mgr, "nodes", []):
                        pos, vel = self.mgr.read_feedback(n)
                        if pos is None:
                            ready = False
                            break

                    if not ready:
                        # try again in 50 ms
                        self.after(50, init_jog_targets_safely)
                        return

                    # we now have valid feedback
                    for n in getattr(self.mgr, "nodes", []):
                        pos, vel = self.mgr.read_feedback(n)
                        self.jog_target[n] = pos

                    self.keyboard_enabled = True
                    print("Keyboard jogging enabled. Targets initialized to actual positions.")

                # kick off safe initialization
                self.after(50, init_jog_targets_safely)

        except Exception as e:
            print(f"⚠️ Failed to set axis state: {e}")


    def zero_node(self, node_id):
        """Reads current encoder pos for a node and stores as zero offset."""
        pos, vel = self.mgr.read_feedback(node_id)
        if pos is not None:
            # store zero offset
            self.zero_offsets[node_id] = pos

            # update cache so latest reflects the real reading
            self.latest.setdefault(node_id, {})
            self.latest[node_id]["pos"] = pos
            self.latest[node_id]["vel"] = vel
            self.latest[node_id]["updated"] = time.time()

            # update GUI
            print(f"🔹 Node {node_id} zeroed at {pos:.2f}")
            self.zero_status[node_id].configure(
                text=f"Node {node_id}: {pos:.2f}",
                text_color="green"
            )
        else:
            print(f"⚠️ Node {node_id}: feedback unavailable — cannot zero.")

    # --- Zero All button ---
    def zero_all(self):
        for i in range(3):
            self.zero_node(i)





    # ============================================================
    # heartbeat & error listener toggles
    # ============================================================
    def toggle_heartbeat(self):
        if not self.heartbeat_running:
            self.heartbeat_running = True
            self.heartbeat_btn.configure(text="Stop")
            threading.Thread(target=self.heartbeat_loop, daemon=True).start()
        else:
            self.heartbeat_running = False
            self.heartbeat_btn.configure(text="Start")

    # no longer using
    def heartbeat_loop(self):
        while self.heartbeat_running:
            for n in self.mgr.nodes:
                result = self.mgr.listen_for_heartbeat(n, duration=0.5)
                if result:
                    error, state = result
                    print(f"💓 Node {n}: state={state}, error={error}")
            time.sleep(1)

    def toggle_error_listener(self):
        if not getattr(self, "error_running", False):
            self.error_running = True
            self.error_btn.configure(text="Stop Listening")
            self.error_output.configure(state="normal")
            self.error_output.insert("end", "⚙️  Starting error listener...\n")
            self.error_output.configure(state="disabled")
            threading.Thread(target=self.error_loop, daemon=True).start()
        else:
            self.error_running = False
            self.error_btn.configure(text="Start Listening")
            self.error_output.configure(state="normal")
            self.error_output.insert("end", "🛑 Error listener stopped.\n")
            self.error_output.configure(state="disabled")


    def error_loop(self):
        while self.error_running:
            if not hasattr(self, "mgr") or not getattr(self.mgr, "nodes", None):
                time.sleep(1)
                continue

            for n in self.mgr.nodes:
                try:
                    # get latest heartbeat (contains error + state)
                    hb = self.mgr.listen_for_heartbeat(n)
                    if hb:
                        err, state = hb
                    else:
                        err, state = None, None
                except Exception as e:
                    err, state = None, None
                    print(f"⚠️ Error reading heartbeat for node {n}: {e}")
                    continue

                # if an error code is active, log it
                if err and err != 0:
                    msg = f"⚠️ Node {n}: error=0x{err:08X}, state={state}\n"
                    print(msg.strip())
                    self.error_output.configure(state="normal")
                    self.error_output.insert("end", msg)
                    self.error_output.see("end")
                    self.error_output.configure(state="disabled")

            time.sleep(1)

    def poll_heartbeat(self):
        """Fast, non-blocking poll for heartbeat + feedback."""
        if not getattr(self.mgr, "bus", None):
            self.after(500, self.poll_heartbeat)
            return

        for n in getattr(self.mgr, "nodes", []):
            try:
                # Non-blocking reads
                err_state = self.mgr.listen_for_heartbeat(n, duration=0)
                if err_state:
                    err, state = err_state
                    self.latest.setdefault(n, {}).update({
                        "error": err,
                        "state": state,
                        "updated": time.time()
                    })

                pos, vel = self.mgr.read_feedback(n, timeout=0)
                if pos is not None:
                    self.latest.setdefault(n, {}).update({
                        "pos": pos,
                        "vel": vel,
                        "updated": time.time()
                    })
            except Exception as e:
                print(f"⚠️ Poll failed for node {n}: {e}")

        # Run again at ~10 Hz
        self.after(50, self.poll_heartbeat)


    def update_feedback_display(self):
        """Refresh right-column displays: per-node heartbeat, state, and feedback."""
        # Enumerate summary
        if hasattr(self.mgr, "nodes"):
            nodes_text = ", ".join(f"Node {n}" for n in self.mgr.nodes) or "None"
            self.enum_status.configure(
                text=f"Enumerated: {nodes_text}",
                text_color=("green" if self.mgr.nodes else "gray")
            )

        # Zero refs
        for nid, lbl in self.zero_status.items():
            if hasattr(self, "zero_offsets") and nid in self.zero_offsets:
                lbl.configure(text=f"Node {nid}: {self.zero_offsets[nid]:.2f}", text_color="green")

        now = time.time()

        # ---------- PER-NODE: heartbeat + big state + feedback ----------
        for nid in self.heartbeat_labels.keys():   # show rows for 0,1,2 regardless
            data = self.mgr.latest.get(nid, {})    # <-- always read backend cache
            hb = data.get("heartbeat")             # (err, state) or None
            fb = data.get("feedback")              # (pos, vel) or None
            updated = data.get("updated", 0.0)
            age = now - updated if updated else 1e9

            # Defaults
            err, state = (None, None) if not hb else hb
            pos, vel = (None, None) if not fb else fb

            # ---- heartbeat line (right column) ----
            pulse = self.tiny_pulse(nid)
            h_lbl = self.heartbeat_labels[nid]
            if hb is None:
                h_lbl.configure(text=f"{pulse} Node {nid}: —", text_color="gray", font=self.font_header)
            else:
                # color: stale -> orange, err!=0 -> red, else green
                if age > 1.5:
                    hb_color = "orange"
                elif err not in (None, 0):
                    hb_color = "red"
                else:
                    hb_color = "green"
                h_lbl.configure(
                    text=f"{pulse} Node {nid}: state={state} err={err}",
                    text_color=hb_color,
                    font=self.font_header
                )

            # ---- big state label (left column) ----
            s_lbl = self.state_labels.get(nid)
            if s_lbl:
                if state == 8:
                    s_lbl.configure(text=f"Node {nid}: CLOSED LOOP", text_color="#00FF7F", font=self.font_header)
                elif state == 1:
                    s_lbl.configure(text=f"Node {nid}: IDLE", text_color="#FFA500", font=self.font_header)
                elif state is None:
                    s_lbl.configure(text=f"Node {nid}: —", text_color="gray", font=self.font_header)
                else:
                    s_lbl.configure(text=f"Node {nid}: STATE {state}", text_color="#7E6C6C", font=self.font_header)

            # ---- feedback (pos / vel) ----
            fb_lbl = self.feedback_labels.get(nid)
            if fb_lbl:
                pulse_fb = self.tiny_pulse(f"fb{nid}")
                if pos is None:
                    fb_lbl.configure(
                        text=f"{pulse_fb} Node {nid}: pos(abs)=— pos(rel)=— vel=—",
                        text_color="gray",
                        font=self.font_header
                    )
                else:
                    zero = self.zero_offsets.get(nid, 0.0)
                    pos_rel = pos - zero
                    # reuse the same color logic as heartbeat line
                    if age > 1.5:
                        fb_color = "orange"
                    elif err not in (None, 0):
                        fb_color = "red"
                    else:
                        fb_color = "green"
                    fb_lbl.configure(
                        text=(f"{pulse_fb} Node {nid}: "
                            f"pos(abs)={pos:7.3f}  pos(rel)={pos_rel:+7.3f}  vel={vel:6.3f}"),
                        text_color=fb_color,
                        font=self.font_header
                    )

        # tick again
        self.after(100, self.update_feedback_display)

    def start_status_thread(self):
        def worker():
            while self.status_running:
                nodes = list(getattr(self.mgr, "nodes", []))
                for n in nodes:
                    # —— heartbeat (state + error) ——
                    try:
                        hb = self.mgr.listen_for_heartbeat(n, duration=0.05)  # your existing backend fn
                        if hb:
                            err, state = hb
                            self.latest.setdefault(n, {})
                            self.latest[n]["error"] = err
                            self.latest[n]["state"] = state
                            self.latest[n]["updated"] = time.time()
                    except Exception as e:
                        print(f"⚠️ heartbeat read failed for node {n}: {e}")

                    # —— pos/vel (best-effort) ——
                    try:
                        pos, vel = self.mgr.read_feedback(n, timeout=0.05)  # returns (None,None) if no frame
                        if pos is not None:
                            self.latest.setdefault(n, {})
                            self.latest[n]["pos"] = pos
                            self.latest[n]["vel"] = vel
                            self.latest[n]["updated"] = time.time()
                    except Exception as e:
                        print(f"⚠️ feedback read failed for node {n}: {e}")

                time.sleep(0.2)  # small cadence; adjust as you like

        threading.Thread(target=worker, daemon=True).start()


    def tiny_pulse(self, node_id):
        """Return a small symbol that flips each update."""
        self._pulse_state = getattr(self, "_pulse_state", {})
        self._pulse_state[node_id] = not self._pulse_state.get(node_id, False)
        return "·" if self._pulse_state[node_id] else " "


        


    def start_listener(self):
        """Continuously read CAN frames in background and cache latest heartbeat/feedback."""
        if not getattr(self, "bus", None):
            raise RuntimeError("CAN bus not initialized")

        def loop():
            import struct, time
            while getattr(self, "bus", None):
                try:
                    msg = self.bus.recv(timeout=0.0)
                    if not msg:
                        time.sleep(0.001)
                        continue

                    node_id = msg.arbitration_id >> 5
                    cmd_id = msg.arbitration_id & 0x1F

                    if cmd_id == 0x01:  # Heartbeat
                        try:
                            error, state, *_ = struct.unpack("<IBBB", msg.data[:7])
                            self.latest.setdefault(node_id, {})["heartbeat"] = (error, state)
                            self.latest[node_id]["updated"] = time.time()
                        except struct.error:
                            pass

                    elif cmd_id == 0x09:  # Feedback (pos, vel)
                        try:
                            pos, vel = struct.unpack("<ff", msg.data[:8])
                            self.latest.setdefault(node_id, {})["feedback"] = (pos, vel)
                            self.latest[node_id]["updated"] = time.time()
                        except struct.error:
                            pass

                except Exception as e:
                    print(f"⚠️ Listener error: {e}")
                    time.sleep(0.1)

        # run in background thread
        import threading
        threading.Thread(target=loop, daemon=True).start()
        print("🟢 CAN listener thread started.")



    def on_key_press(self, event):
        self.keys_down.add(event.keysym)

    def on_key_release(self, event):
        ks = event.keysym
        if ks in self.keys_down:
            self.keys_down.remove(ks)

        # When shift is released, clear all jog keys to avoid stale presses
        if ks in ("Shift_L", "Shift_R"):
            for bad in ["Y", "H", "U", "J", "I", "K"]:
                if bad in self.keys_down:
                    self.keys_down.remove(bad)

    def keyboard_control_loop(self):
        if self.keyboard_enabled and getattr(self.mgr, "bus", None):

            shift = ("Shift_L" in self.keys_down or "Shift_R" in self.keys_down)

            # jogging increment - tune if not smooth
            delta = 0.005  # turns per tick

            if shift and any(k in self.keys_down for k in ["Y","H","U","J","I","K"]):
                affected_nodes = []

                # node 0
                if "Y" in self.keys_down:
                    self.jog_target[0] += delta
                    affected_nodes.append(0)
                if "H" in self.keys_down:
                    self.jog_target[0] -= delta
                    affected_nodes.append(0)

                # node 1
                if "U" in self.keys_down:
                    self.jog_target[1] += delta
                    affected_nodes.append(1)
                if "J" in self.keys_down:
                    self.jog_target[1] -= delta
                    affected_nodes.append(1)

                # node 2
                if "I" in self.keys_down:
                    self.jog_target[2] += delta
                    affected_nodes.append(2)
                if "K" in self.keys_down:
                    self.jog_target[2] -= delta
                    affected_nodes.append(2)

                for n in set(affected_nodes):
                    tgt = self.jog_target[n]
                    print(f"🕹 Jog cmd → node {n}: target={tgt:.4f} turns")
                    self.mgr.set_position(n, tgt)
            

        # schedule again
        self.after(5, self.keyboard_control_loop)    # first arg is time in between in ms 




    def connect_gimbal(self):
        port = self.gimbal_port_var.get()
        baud = 115200   # SimpleFOC default, adjust if yours is different

        try:
            # Open and store the serial connection
            self.gimbal_ser = serial.Serial(
                port,
                baudrate=baud,
                timeout=0.02
            )
            print(f"[GIMBAL] Connected on {port}")

            # Optional background reader for incoming telemetry
            self._gimbal_running = True
            threading.Thread(target=self._gimbal_reader, daemon=True).start()

        except Exception as e:
            print(f"[ERROR] Failed to connect to gimbal on {port}: {e}")



    def _gimbal_reader(self):
        """ Continuously read any Commander output so the GUI never blocks. """
        while getattr(self, "_gimbal_running", False):
            try:
                if self.gimbal_ser.in_waiting:
                    line = self.gimbal_ser.readline().decode(errors="ignore").strip()
                    if line:
                        print("[GIMBAL RX]", line)
            except Exception:
                break

    
    def send_gimbal_cmd(self, angle):
        """Send target angle to SimpleFOC (Commander 'T' command)."""
        if not hasattr(self, "gimbal_ser") or self.gimbal_ser is None:
            print("[GIMBAL ERROR] Not connected.")
            return

        try:
            msg = f"T{angle}\n"
            self.gimbal_ser.write(msg.encode())
            print(f"[GIMBAL TX] {msg.strip()}")
        except Exception as e:
            print(f"[GIMBAL ERROR] Failed to send angle {angle}: {e}")


    def send_gimbal_deg(self, deg):
        if deg is None:
            return
        rad = np.deg2rad(deg)
        self.send_gimbal_cmd(rad)

    def _send_gimbal_angle_from_entry(self):
        raw = self.gimbal_deg_entry.get().strip()
        if not raw:
            print("[GIMBAL] Empty angle field.")
            return
        try:
            deg = float(raw)
            self.send_gimbal_deg(deg)
        except ValueError:
            print(f"[GIMBAL] Invalid angle: {raw}")




    def start_cv(self):
        if self.cv_pipeline is None:
            from cv_pipeline import CVPipeline
            self.cv_pipeline = CVPipeline(cv_to_node = self.cv_to_node)

        # START CV THREAD
        self.cv_pipeline.start()

        # START COLLISION DETECTOR
        if not hasattr(self, "collision_detector") or self.collision_detector is None:
            from collision_detection import CollisionDetector
            self.collision_detector = CollisionDetector(use_gui=False, joint_map = self.joint_map)

        # BEGIN GUI UPDATE LOOPS
        self.cv_update_loop()
        self.coldetec_update_loop()


    def stop_cv(self):
        pass


    def cv_update_loop(self):
        if self.cv_pipeline is None:
            return

        frame = self.cv_pipeline.get_latest_frame()
        if frame is not None:
            self.update_webcam_frame(frame)

        
        # fetch angles
        angles = self.cv_pipeline.get_latest_angles()


        self.node_to_label = {
            0: self.cv_angle_node0_val,
            1: self.cv_angle_node1_val,
            2: self.cv_angle_node2_val,
            3: self.cv_angle_node3_val,
        }

        for node_index, joint_name in self.node_to_cv.items():
            angle = angles.get(joint_name)
            label = self.node_to_label[node_index]

            if angle is None:
                label.configure(text="--°")
            else:
                label.configure(text=f"{angle:.1f}°")


        # re-run every ~30ms
        self.after(30, self.cv_update_loop)



    def update_webcam_frame(self, img_array):
        try:
            img = Image.fromarray(img_array)

            # Apply rounded corners
            img = round_corners(img, radius=40)   # try 20–40 radius

            tk_img = CTkImage(
                light_image=img,
                dark_image=img,
                size=(333, 250)   # fixed size
            )

            self.cv_cam_image = tk_img
            self.cv_cam_label.configure(image=tk_img)

        except Exception as e:
            print(f"Webcam update failed: {e}")




    def update_sim_frame(self, img_array):
        try:
            if img_array is None:
                return

            if img_array.ndim == 3 and img_array.shape[-1] == 4:
                img_array = img_array[:, :, :3]

            img = Image.fromarray(img_array)

            # Apply rounded corners (same as webcam)
            img = round_corners(img, radius=30)

            TARGET_SIZE = (333, 250)

            tk_img = CTkImage(
                light_image=img,
                dark_image=img,
                size=TARGET_SIZE,
            )

            self.cv_coldetech_label.configure(
                width=TARGET_SIZE[0],
                height=TARGET_SIZE[1],
                image=tk_img,
            )

            self.cv_coldetech_image = tk_img

        except Exception as e:
            print(f"Sim frame update failed: {e}")



    def coldetec_update_loop(self):
        if self.cv_pipeline is None or self.collision_detector is None:
            self.after(30, self.coldetec_update_loop)
            return

        # 1. Get angles from CV and run collision detection
        angles = self.cv_pipeline.get_latest_angles()

        # ---- Detect if CV is giving us anything valid ----
        valid = any(
            angles.get(k) is not None
            for k in ["left_shoulder", "left_elbow", "right_elbow", "right_shoulder"]
        )

        now = time.time()

        if valid:
            # CV is producing good angles → update the timestamp
            self.last_valid_cv_time = now
            angles_to_use = angles
        else:
            # CV angles are dead → check how long it's been dead
            dead_time = now - self.last_valid_cv_time

            if dead_time > 0.4:     # 400 ms of no angles = fail-safe engages
                angles_to_use = { 
                    "left_shoulder": 100.0,      # node 1
                    "left_elbow":    100.0,      # node 2
                    # "right_elbow":   100.0,      # node 0 -> doesnt rlly matter
                }
            else:
                # still within tolerance → use last angles or smoothing output
                angles_to_use = angles

        # --- Now run collision detection on angles_to_use ---
        colliding, report, sim_frame = self.collision_detector.get_state(angles_to_use)

        # get processed angles AFTER collision logic
        processed = getattr(self.collision_detector, "last_processed_angles", {})

        # update table if safe, else clear it
        if not colliding and processed:
            self.update_cv_command_table(processed, angles)

            # send to ODrive ONLY if follow-CV is active
            # === RATE LIMIT ODRIVE COMMANDS ===
            now = time.time()
            last = getattr(self, "last_odrive_update", 0)
            if (now - last) >= 0.03:    # 0.05 sec = 20 Hz
                self.last_odrive_update = now

                if self.cv_follow_state:
                    self._apply_odrive_commands(processed, self.node_to_cv_inv)

                    self.send_gimbal_deg(angles.get("right_shoulder"))

        else:
            # collision or no processed angles → clear table
            self.update_cv_command_table({}, angles)



        # 1b. Read processed (robot-space) angles from the collision detector
        processed = getattr(self.collision_detector, "last_processed_angles", {})

        def fmt(a):
            return "--°" if a is None else f"{a:.1f}°"

        node0_angle = processed.get("right_elbow")
        node1_angle = processed.get("left_shoulder")
        node2_angle = processed.get("left_elbow")
        node3_angle = angles.get("right_shoulder")

        if hasattr(self, "coldetec_joint_label"):
            text = (
                f"Node 0: {fmt(node0_angle)}   |   "
                f"Node 1: {fmt(node1_angle)}   |   "
                f"Node 2: {fmt(node2_angle)}\n"
                f"Node 3: {fmt(node3_angle)}"
            )
            self.coldetec_joint_label.configure(text=text)

        # 2. Update the PyBullet window
        if sim_frame is not None:
            self.update_sim_frame(sim_frame)

        # 3. Update collision report box
        try:
            self.collision_report_box.configure(state="normal")
            self.collision_report_box.delete("1.0", "end")
            self.collision_report_box.insert("1.0", report)
            self.collision_report_box.configure(state="disabled")
        except Exception as e:
            print("collision report update failed:", e)

        # 4. Call again
        self.after(50, self.coldetec_update_loop)


    @staticmethod
    def robot_rad_to_turns(angle_rad, gear_ratio=10.0):
        """
        Convert a robot joint angle (in radians) into motor turns.
        """
        return (angle_rad / (2 * math.pi)) * gear_ratio

    @staticmethod
    def robot_deg_to_turns(angle_deg, gear_ratio=10.0):
        """
        Convert a robot joint angle (in degrees) into motor turns.
        """
        return (angle_deg / 360.0) * gear_ratio
    




    def update_cv_command_table(self, processed_angles: dict, angles: dict):
        """
        Update the Motor Commands table (right CV column) from
        processed robot-space angles (degrees).

        processed_angles keys: 'left_shoulder', 'left_elbow', 'right_elbow' 'right_shoulder'
        """


        # First reset everything
        for node_id in range(4):
            # degrees
            if node_id in self.cv_cmd_deg_labels:
                self.cv_cmd_deg_labels[node_id].configure(text="--°")

            # turns
            if node_id in self.cv_cmd_turn_labels:
                if node_id == 3:
                    self.cv_cmd_turn_labels[node_id].configure(text="n/a")
                else:
                    right_shoulder = angles.get("right_shoulder")
                    text = f"{right_shoulder:.1f}°" if right_shoulder is not None else "--°"
                    self.cv_cmd_turn_labels[node_id].configure(text=text)

        # Now fill rows for nodes 0–2 using processed angles
        for cv_name, node_id in self.cv_to_node.items():
            deg = processed_angles.get(cv_name)

            # node 3 (right_shoulder) is not part of processed_angles. Use raw CV angle.
            if node_id == 3:
                deg = angles.get("right_shoulder")
                deg = self.remap(deg, 30.0, 160.0, 0.0, 360)
                if node_id in self.cv_cmd_deg_labels:
                    text = f"{deg:.1f}°" if deg is not None else "--°"
                    self.cv_cmd_deg_labels[node_id].configure(text=text)
                continue

            if deg is None:
                continue

            # degrees column
            if node_id in self.cv_cmd_deg_labels:
                self.cv_cmd_deg_labels[node_id].configure(text=f"{deg:.1f}°")

            # turns column (skip node 3 logic here, we only have 0–2)
            if node_id in self.cv_cmd_turn_labels and node_id != 3:
                # if you have robot_deg_to_turns as a method, use that
                turns = self.robot_deg_to_turns(deg) if hasattr(self, "robot_deg_to_turns") else deg / 360.0
                self.cv_cmd_turn_labels[node_id].configure(text=f"{turns:.4f}")


    def _on_cv_follow_pressed(self):
        self.cv_follow_state = not self.cv_follow_state

        if self.cv_follow_state:
            # turn all nodes ON
            for i in self.node_follow:
                self.node_follow[i] = True
            self._refresh_node_follow_buttons()

            self.cv_follow_btn.configure(
                text="follow CV: on",
                fg_color="#7B2F99",   # purple when all ON
                hover_color="#9748CC"
            )
        else:
            # turn all nodes OFF
            for i in self.node_follow:
                self.node_follow[i] = False
            self._refresh_node_follow_buttons()

            self.cv_follow_btn.configure(
                text="follow CV: off",
                fg_color="#4a4a4a"   # gray when OFF
            )





    def _apply_odrive_commands(self, processed: dict, cv_to_node):
        """
        Apply processed robot-space angles (deg) to ODrive using the
        RELATIVE zeroed frame, not absolute turns.

        Steps per node:
        1. Convert degrees → delta turns
        2. Retrieve zero offset (or infer it)
        3. Compute absolute target = zero + delta
        4. Send to motor
        5. Update self.latest + GUI cache
        """
        if not hasattr(self, "mgr") or self.mgr is None:
            return

        for cv_name, node_id in cv_to_node.items():
            deg = processed.get(cv_name)
            if deg is None:
                continue

            # skip nodes that are NOT following CV
            if not self.node_follow.get(node_id, False):
                continue

            # Step 1: convert deg → delta turns
            delta = self.robot_deg_to_turns(deg)

            # Step 2: get zero offset
            zero = self.zero_offsets.get(node_id)

            # If zero missing: infer it from latest known motor pos
            if zero is None:
                zero = self.latest.get(node_id, {}).get("pos", 0.0)
                self.zero_offsets[node_id] = zero

                print(f"ℹ️ Node {node_id} had no zero; using cached pos={zero:.3f}")

                # update GUI zero status if available
                if hasattr(self, "zero_status") and node_id in self.zero_status:
                    self.zero_status[node_id].configure(
                        text=f"Node {node_id}: {zero:.3f} turns",
                        text_color="orange"
                    )

            # Step 3: compute absolute target
            target_abs = zero + delta

            # Step 4: send command
            try:
                self.mgr.set_position(node_id, target_abs)
            except Exception as e:
                print(f"[ODrive] FAILED sending node {node_id}: {e}")
                continue

            # Step 5: update latest-cache
            self.latest.setdefault(node_id, {}).update({
                "pos": target_abs,
                "updated": time.time(),
            })

            print(f"🎯 Node {node_id}: abs_target={target_abs:.3f} (zero={zero:.3f}, Δ={delta:+.3f})")
    

    def pretty(self, name):
        return name.replace("_", " ").title()
    

    def _on_node_follow_pressed(self, idx):
        # Flip this node's state
        self.node_follow[idx] = not self.node_follow[idx]

        # If ANY node is following, master must be ON
        if any(self.node_follow.values()):
            self.cv_follow_state = True
            self.cv_follow_btn.configure(
                text="follow CV: on",
                fg_color="#2F9976",   # green when ON
                
            )
        else:
            # No nodes are following
            self.cv_follow_state = False
            self.cv_follow_btn.configure(
                text="follow CV: off",
                fg_color="#4a4a4a"
            )

        self._refresh_node_follow_buttons()


    def _refresh_node_follow_buttons(self):
        for idx, btn in self.node_follow_btns.items():
            if self.node_follow[idx]:
                btn.configure(text=f"Node {idx}: on", fg_color="#2F9976", hover_color="#37A785")
            else:
                btn.configure(text=f"Node {idx}: off", fg_color="#4a4a4a")


    @staticmethod
    def remap(val, in_min, in_max, out_min, out_max):
        """Linearly map val from one range to another."""
        if val is None:
            return None
        # clamp to input range first
        if val < in_min: 
            val = in_min
        if val > in_max: 
            val = in_max
        # normalize
        t = (val - in_min) / (in_max - in_min)
        return out_min + t * (out_max - out_min)


        


from PIL import Image, ImageDraw

def round_corners(img: Image.Image, radius: int = 30):
    """Return a copy of the image with rounded corners."""
    # Create rounded mask
    mask = Image.new("L", img.size, 0)
    draw = ImageDraw.Draw(mask)
    draw.rounded_rectangle((0, 0) + img.size, radius=radius, fill=255)

    # Apply mask to image
    rounded = Image.new("RGBA", img.size)
    rounded.paste(img, (0, 0), mask)
    return rounded

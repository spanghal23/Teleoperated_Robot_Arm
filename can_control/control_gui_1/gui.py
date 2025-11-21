# gui.py
import customtkinter as ctk
import sys, io, threading, time
from backend import ODriveManager


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
        self.geometry("1800x800+75+100")
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

        self.mgr = ODriveManager()
        self.heartbeat_running = False
        self.error_running = False

        # ---------- main frame ----------
        main = ctk.CTkFrame(self)
        main.pack(padx=20, pady=10, fill="both", expand=True)

        RIGHT_W = 520  # pick what you like (e.g., 480–600)

        # Left takes remaining space, right is fixed
        main.grid_columnconfigure(0, weight=1)             # left grows
        main.grid_columnconfigure(1, weight=0, minsize=RIGHT_W)  # right fixed

        # Left column uses tabs
        self.tabview = ctk.CTkTabview(main)
        self.tabview.grid(row=0, column=0, sticky="nsew", padx=(0,10))

        # Create tabs
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



        # ---------- 1. Connect ----------
        sec1 = ctk.CTkFrame(self.left_col)
        sec1.pack(fill="x", pady=10)

        ctk.CTkLabel(sec1, text="1. Connect to CAN (enter password in terminal if prompted):", font=self.font_header).pack(side="left", padx=10)
        ctk.CTkButton(sec1, text="Connect", command=self.threaded(self.connect_can), font=self.font_button).pack(side="left", padx=10)

        # ---------- 2. Enumerate ----------
        sec2 = ctk.CTkFrame(self.left_col)
        sec2.pack(fill="x", pady=10)
        ctk.CTkLabel(sec2, text="2. Enumerate ODrives (discover nodes):", font=self.font_header).pack(side="left", padx=10)
        ctk.CTkButton(sec2, text="Enumerate", command=self.threaded(self.enumerate_nodes), font=self.font_button).pack(side="left", padx=5)

        ctk.CTkLabel(sec2, text="(Optional) Calibrate:", font=self.font_header).pack(side="left", padx=10)

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
            fg_color="gray40",
            hover_color="gray30",
            width=100,
            command=self.threaded(lambda: self.set_axis_state_all(1)),
            font=self.font_button
        )
        idle_btn.grid(row=0, column=4, padx=10)

        # CLOSED LOOP button (state = 8)
        loop_btn = ctk.CTkButton(
            sec3,
            text="Closed-loop",
            fg_color="#26714A",  # a green tone
            hover_color="#26714A",
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
        ctk.CTkLabel(sec5, text="5. Position Control (relative turns):", font=self.font_header).grid(row=0, column=0, padx=10, sticky="w")

        self.pos_entries = {}
        for i in range(3):
            ctk.CTkLabel(sec5, text=f"Node {i}:").grid(row=0, column=2*i+1, padx=5)
            e = ctk.CTkEntry(sec5, width=80)
            e.grid(row=0, column=2*i+2, padx=5)
            self.pos_entries[i] = e

        ctk.CTkButton(sec5, text="Send Control", command=self.threaded(self.send_positions), font=self.font_button).grid(row=0, column=8, padx=10)

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
                text=f"Node {node_id}: {pos:.2f} turn(s)",
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
                lbl.configure(text=f"Node {nid}: {self.zero_offsets[nid]:.3f} turns", text_color="green")

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

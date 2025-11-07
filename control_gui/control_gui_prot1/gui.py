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
        if self.tag == "stderr":
            self.text_widget.insert("end", s, "error")
        else:
            self.text_widget.insert("end", s)
        self.text_widget.see("end")
    def flush(self): ...
    

# ========== GUI ==========
class ODriveGUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("ODrive CAN GUI (Linux Only)")
        self.geometry("1600x800+200+100")
        ctk.set_default_color_theme("blue")
        ctk.set_widget_scaling(1.1)

        self.latest = {}  # cache
        self.status_running = False # bg poller flag    

        self.font_header = ("Roboto", 14)
        self.font_button = ("Roboto", 14)  # , 'bold'
        self.font_label = ("Roboto", 14)

        self.mgr = ODriveManager()
        self.heartbeat_running = False
        self.error_running = False

        # ---------- main frame ----------
        main = ctk.CTkFrame(self)
        main.pack(padx=20, pady=10, fill="both", expand=True)

        main.grid_columnconfigure(0, weight=1)   # left = 3× bigger
        main.grid_columnconfigure(1, weight=1)   # right = smaller

        # Left column (all control sections)
        self.left_col = ctk.CTkFrame(main)
        self.left_col.grid(row=0, column=0, sticky="nsew", padx=(0,10))

        # Right column (status panels)
        self.right_col = ctk.CTkFrame(main)
        self.right_col.grid(row=0, column=1, sticky="nsew", padx=(10,0))


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
        ctk.CTkButton(sec2, text="(Optional) Calibrate", command=self.threaded(self.calibrate_all), font=self.font_button).pack(side="left", padx=5)

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

        # ---- LEFT COLUMN: System State ----
        self.system_state_label = ctk.CTkLabel(
            sec_r4,
            text="IDLE",
            font=("Arial", 26, "bold"),
            text_color="#00BFFF"
        )
        self.system_state_label.grid(row=1, column=0, rowspan=4, sticky="nsew", padx=15, pady=10)

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

        # Optional: make columns expand evenly
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


    # ============================================================
    # threaded wrapper
    # ============================================================
    def threaded(self, fn):
        def wrapper():
            def run_safe():
                try:
                    fn()
                except Exception:
                    import traceback
                    traceback.print_exc()
            return lambda: threading.Thread(target=run_safe, daemon=True).start()
        return wrapper()

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

    def calibrate_all(self):
        for n in self.mgr.nodes:
            self.mgr.calibrate_node(n)

    def send_positions(self):
        """Move to absolute position in the zeroed frame: target_abs = zero + delta."""
        for node_id, entry in self.pos_entries.items():
            val = entry.get().strip()
            if not val:
                continue
            try:
                delta = float(val)

                # 1) get zero; if not set yet, try to seed it from feedback once
                zero = self.zero_offsets.get(node_id, None)
                if zero is None:
                    pos0, _ = self.mgr.read_feedback(node_id, timeout=0.3)
                    if pos0 is not None:
                        zero = pos0
                        self.zero_offsets[node_id] = zero
                        print(f"ℹ️ Node {node_id} had no zero; seeding zero={zero:.3f}")
                        self.latest.setdefault(node_id, {})
                        self.latest[node_id].update({"pos": pos0, "updated": time.time()})
                        if node_id in self.zero_status:
                            self.zero_status[node_id].configure(
                                text=f"Node {node_id}: {zero:.3f} turns", text_color="green"
                            )
                    else:
                        zero = 0.0
                        print(f"⚠️ Node {node_id}: no feedback to seed zero; assuming zero=0.000")

                # 2) compute target purely from zeroed frame
                target_abs = zero + delta

                # 🔒 SAFETY CHECK — ensure axis is closed loop
                hb = self.mgr.listen_for_heartbeat(node_id, duration=0.3)
                if hb:
                    err, state = hb
                    if state != 8:
                        print(f"⚠️ Node {node_id} is in state {state} — switch to CLOSED_LOOP_CONTROL (8) before moving.")
                        continue
                else:
                    print(f"⚠️ Node {node_id}: no heartbeat — cannot verify axis state.")
                    continue

                # 3) send command
                self.mgr.set_position(node_id, target_abs)

                # 4) cache for GUI
                self.latest.setdefault(node_id, {})
                self.latest[node_id].update({"pos": target_abs, "updated": time.time()})

                print(f"🎯 Node {node_id}: abs_target={target_abs:.3f} (zero={zero:.3f}, Δ={delta:+.3f})")

            except ValueError:
                print(f"⚠️ Invalid input for node {node_id}")
            except Exception as e:
                print(f"⚠️ Error moving node {node_id}: {e}")



    def clear_errors(self):
        for n in self.mgr.nodes:
            self.mgr.clear_errors_and_idle(n)

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
                    # Example: ODrive or CAN manager should have a method like this:
                    # (You might need to replace this with whatever you use to query errors)
                    err, state = self.mgr.read_errors(n)
                except Exception as e:
                    err = None
                    print(f"Error reading node {n}: {e}")
                    continue

                if err and err != 0:
                    # Log to GUI
                    msg = f"⚠️ Node {n}: error={err}, state={state}\n"
                    print(msg.strip())
                    self.error_output.configure(state="normal")
                    self.error_output.insert("end", msg)
                    self.error_output.see("end")
                    self.error_output.configure(state="disabled")

            time.sleep(1)



    def poll_heartbeat(self):
        """Poll ODrives periodically for heartbeat + feedback."""
        if not getattr(self.mgr, "bus", None):
            self.after(500, self.poll_heartbeat)
            return

        for n in getattr(self.mgr, "nodes", []):
            try:
                # ---- heartbeat ----
                result = self.mgr.listen_for_heartbeat(n, duration=0.05)
                if result:
                    err, state = result
                    self.latest.setdefault(n, {})
                    self.latest[n].update({
                        "error": err,
                        "state": state,
                        "updated": time.time()
                    })

                # ---- feedback (pos/vel) ----
                pos, vel = self.mgr.read_feedback(n, timeout=0.05)
                if pos is not None:
                    self.latest[n].update({"pos": pos, "vel": vel})
            except Exception as e:
                print(f"⚠️ Heartbeat/feedback poll failed for node {n}: {e}")

        # schedule again
        self.after(250, self.poll_heartbeat)


    def update_feedback_display(self):
        """Refresh right-column displays: heartbeat + feedback + system state."""
        # connection / enumerate summary
        if hasattr(self.mgr, "nodes"):
            nodes_text = ", ".join(f"Node {n}" for n in self.mgr.nodes) or "None"
            self.enum_status.configure(
                text=f"Enumerated: {nodes_text}",
                text_color=("green" if self.mgr.nodes else "gray")
            )

        # zero refs (if tracked)
        for nid, lbl in self.zero_status.items():
            if hasattr(self, "zero_offsets") and nid in self.zero_offsets:
                lbl.configure(
                    text=f"Node {nid}: {self.zero_offsets[nid]:.3f} turns",
                    text_color="green"
                )

        now = time.time()
        all_states = []  # collect node states for global display

        # ---------- HEARTBEAT ----------
        for nid, lbl in self.heartbeat_labels.items():
            data = self.latest.get(nid, {})
            state = data.get("state")
            err = data.get("error")
            updated = data.get("updated", 0)
            age = now - updated
            pulse = self.tiny_pulse(nid)

            if state is None:
                lbl.configure(text=f"{pulse} Node {nid}: state=— err=—", text_color="gray")
            elif age > 1.5:
                lbl.configure(text=f"{pulse} Node {nid}: stale (>{age:.1f}s)", text_color="orange")
            elif err not in (None, 0):
                lbl.configure(text=f"{pulse} Node {nid}: ⚠️ err={err} state={state}", text_color="red")
            else:
                lbl.configure(text=f"{pulse} Node {nid}: state={state} err={err}", text_color="green")

            if state:
                all_states.append(state)

        # ---------- SYSTEM STATE SUMMARY ----------
        if all_states:
            # determine most frequent / aggregate state
            if all(s == "CLOSED_LOOP_CONTROL" for s in all_states):
                sys_state, color = "CLOSED LOOP", "#00FF7F"
            elif all(s == "IDLE" for s in all_states):
                sys_state, color = "IDLE", "#FFA500"
            elif any("CALIB" in s for s in all_states):
                sys_state, color = "CALIBRATING", "#1E90FF"
            else:
                sys_state, color = "CUSTOM", "#7E6C6C"
        else:
            sys_state, color = "—", "gray"

        # update label text + color
        if hasattr(self, "system_state_label"):
            self.system_state_label.configure(text=sys_state, text_color=color)

        # ---------- FEEDBACK (pos / vel) ----------
        for nid, lbl in self.feedback_labels.items():
            data = self.latest.get(nid, {})
            pos_abs = data.get("pos")
            vel = data.get("vel")
            err = data.get("error")
            updated = data.get("updated", 0)
            age = now - updated

            zero = self.zero_offsets.get(nid, 0.0)
            pulse = self.tiny_pulse(f"fb{nid}")  # use unique key so it pulses independently

            if pos_abs is None:
                lbl.configure(
                    text=f"{pulse} Node {nid}: pos(abs)=— pos(rel)=— vel=—",
                    text_color="gray",
                )
                continue

            pos_rel = pos_abs - zero
            color = "green"
            if err not in (None, 0):
                color = "red"
            elif age > 1.5:
                color = "orange"

            lbl.configure(
                text=(
                    f"{pulse} Node {nid}: "
                    f"pos(abs)={pos_abs:7.3f}  "
                    f"pos(rel)={pos_rel:+7.3f}  "
                    f"vel={vel:6.3f}"
                ),
                text_color=color,
            )

        # run again in 0.5s
        self.after(500, self.update_feedback_display)





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


        
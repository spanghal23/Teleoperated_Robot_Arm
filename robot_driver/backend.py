""" 
Backend for odrive GUI
Functions:


"""

import can, struct, time, subprocess, os, json


# backend.py (append this at the end of your function definitions)

class ODriveManager:


    BROADCAST_NODE_ID = 0x3F
    ADDRESS_CMD = 0x06
    REBOOT_CMD = 0x16
    CLEAR_ERRORS_CMD = 0x18 
    
    def __init__(self, channel="can0", bitrate=250000):
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.latest = {}  
        self.nodes = []
        self._listener_running = False

    def setup_can_interface(self):
        """Bring up can0 and create a python-can Bus."""
        try:
            subprocess.run(
                ["sudo", "ip", "link", "set", self.channel, "up", "type", "can", "bitrate", str(self.bitrate)],
                check=True,
            )
            self.bus = can.interface.Bus(channel=self.channel, interface="socketcan")
            print(f"✅ CAN interface up @ {self.bitrate} bit/s — connected to {self.bus.channel_info}")
            return True
        except Exception as e:
            print(f"⚠️ Failed to set up CAN: {e}")
            return False
        
    def shutdown_can_interface(self):
        """
        Close the python-can Bus and bring the interface down cleanly.
        """
        try:
            if getattr(self, 'bus', None):
                try:
                    self.bus.shutdown()
                except Exception:
                    # best-effort shutdown
                    pass
                self.bus = None
            # bring the same channel down that was brought up
            subprocess.run(["sudo", "ip", "link", "set", self.channel, "down"], check=True)
            print("🛑 CAN interface shut down successfully.")
            return True
        except subprocess.CalledProcessError as e:
            print(f"⚠️ Failed to shut down CAN: {e}")
        except Exception as e:
            print(f"⚠️ Unexpected error during shutdown: {e}")
        return False

    # Core motion
    def set_axis_state(self, node_id: int, state_id: int):  
        """ IDLE = 1, CLOSED-LOOP CONTROL = 2 """
        if not getattr(self, 'bus', None):
            raise RuntimeError("CAN bus not initialized")
        msg = can.Message(arbitration_id=(node_id << 5) | 0x07,
                        data=struct.pack('<I', state_id),
                        is_extended_id=False)
        self.bus.send(msg)


    def set_position(self, node_id: int, pos: float, vel_ff: int = 0, torque_ff: int = 0):
        if not getattr(self, 'bus', None):
            raise RuntimeError("CAN bus not initialized")
        msg = can.Message(arbitration_id=(node_id << 5) | 0x0C,
                        data=struct.pack('<fhh', pos, vel_ff, torque_ff),
                        is_extended_id=False)
        self.bus.send(msg)


    def set_velocity(self, node_id: int, vel: float):
        if not getattr(self, 'bus', None):
            raise RuntimeError("CAN bus not initialized")
        msg = can.Message(arbitration_id=(node_id << 5) | 0x0D,
                        data=struct.pack('<f', vel),
                        is_extended_id=False)
        self.bus.send(msg)



    # Error handling
    def clear_errors_and_enable(self, node_id: int, reenable_closed_loop: bool = True):
        if not getattr(self, 'bus', None):
            raise RuntimeError("CAN bus not initialized")
        try:
            msg = can.Message(arbitration_id=(node_id << 5) | 0x18, data=b'', is_extended_id=False)
            self.bus.send(msg)
            print(f"✅ Cleared errors on node {node_id}")
            time.sleep(0.1)
            if reenable_closed_loop:
                self.set_axis_state(node_id, 8)  # CLOSED_LOOP_CONTROL
                print(f"✅ Node {node_id} re-entered CLOSED_LOOP_CONTROL")
        except Exception as e:
            print(f"⚠️ Error clearing node {node_id}: {e}")

    def clear_errors_and_idle(self, node_id: int, reenable_closed_loop: bool = True):
        if not getattr(self, 'bus', None):
            raise RuntimeError("CAN bus not initialized")
        try:
            msg = can.Message(arbitration_id=(node_id << 5) | 0x18, data=b'', is_extended_id=False)
            self.bus.send(msg)
            print(f"✅ Cleared errors on node {node_id}")
            time.sleep(0.1)
            if reenable_closed_loop:
                self.set_axis_state(node_id, 1)  # IDLE
                print(f"✅ Node {node_id} entered IDLE")
        except Exception as e:
            print(f"⚠️ Error clearing node {node_id}: {e}")

    def _get_address_msg(self):
        """Broadcast discovery request."""
        return can.Message(
            arbitration_id=(self.BROADCAST_NODE_ID << 5) | self.ADDRESS_CMD,
            is_extended_id=False,
            is_remote_frame=True
        )

    def _set_address_msg(self, sn: int, node_id: int):
        """Assigns a node_id to an ODrive with serial number sn."""
        return can.Message(
            arbitration_id=(self.BROADCAST_NODE_ID << 5) | self.ADDRESS_CMD,
            data=bytes([node_id]) + sn.to_bytes(6, byteorder="little"),
            is_extended_id=False
        )

    def enumerate_odrives(self, timeout=3.0, auto_assign=True):
        if not getattr(self, 'bus', None):
            raise RuntimeError("CAN bus not initialized")

        print("🔍 Scanning for ODrives...")
        discovered = {}
        t_start = time.time()
        iteration = 0
        last_response = None

        while time.time() - t_start < timeout:   # <── outer loop bounded by timeout
            iteration += 1
            self.bus.send(self._get_address_msg())

            # collect any responses for 0.5 s
            t_window = time.time() + 0.5
            while time.time() < t_window:
                msg = self.bus.recv(timeout=0.1)
                if not msg:
                    continue
                cmd_id = msg.arbitration_id & 0x1F
                if cmd_id == self.ADDRESS_CMD and not msg.is_remote_frame:
                    node_id = msg.data[0]
                    serial = int.from_bytes(msg.data[1:7], byteorder="little")
                    if serial not in discovered:
                        node_str = (
                            "unaddressed" if node_id == self.BROADCAST_NODE_ID else f"ID {node_id}"
                        )
                        print(f"🆔 Found ODrive SN {serial:012X} ({node_str})")
                    discovered[serial] = node_id
                    last_response = time.time()

            # after 3 iterations, assign IDs if needed
            if iteration == 3 and auto_assign and discovered:
                free_ids = set(range(0, 0x3F)) - set(discovered.values())
                for serial, node_id in list(discovered.items()):
                    if node_id == self.BROADCAST_NODE_ID and free_ids:
                        new_id = min(free_ids)
                        free_ids.remove(new_id)
                        msg = self._set_address_msg(serial, new_id)
                        self.bus.send(msg)
                        discovered[serial] = new_id
                        print(f"➡️ Assigned node {new_id} to SN {serial:012X}")

            # stop early if no response in N seconds after initial ones
            if last_response and (time.time() - last_response > timeout):
                break

        print(f"✅ Discovery complete — found {len(discovered)} device(s).")
        self.nodes = list(discovered.values())
        return discovered


    def calibrate_node(self, node_id: int, save_config=True):
        """
        Runs a FULL_CALIBRATION_SEQUENCE for a given node and waits until done.
        """
        if not getattr(self, 'bus', None):
            print("⚠️ CAN bus not initialized — cannot calibrate.")
            raise RuntimeError("CAN bus not initialized")

        print(f"⚙️ Starting calibration for node {node_id}...")
        # Clear errors
        msg_clear = can.Message(arbitration_id=(node_id << 5) | 0x18, data=b'', is_extended_id=False)
        self.bus.send(msg_clear)
        time.sleep(0.1)
        # Start full calibration
        msg_cal = can.Message(arbitration_id=(node_id << 5) | 0x07,
                            data=struct.pack('<I', 3),  # 3 = FULL_CALIBRATION_SEQUENCE
                            is_extended_id=False)
        self.bus.send(msg_cal)

        # Wait until state returns to IDLE (1)
        t0 = time.time()
        while time.time() - t0 < 30:  # 30 s timeout
            msg = self.bus.recv(timeout=0.5)
            if not msg:
                continue
            cmd_id = msg.arbitration_id & 0x1F
            nid = msg.arbitration_id >> 5
            if cmd_id == 0x01 and nid == node_id:  # heartbeat
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if error != 0:
                    print(f"⚠️ Node {node_id} error code {error}")
                if state == 1:  # IDLE
                    print("✅ Calibration finished successfully.")
                    break
        else:
            print("⚠️ Timeout waiting for calibration to finish.")

        if save_config:
            print("💾 Saving configuration...")
            msg_save = can.Message(arbitration_id=(node_id << 5) | 0x16,
                                data=bytes([1]),  # 1 = REBOOT_ACTION_SAVE
                                is_extended_id=False)
            self.bus.send(msg_save)
            print("♻️ Node rebooting after save...")


    def listen_for_heartbeat(self, node_id, duration=0.0):
        """Return last cached (error, state) or None."""
        return self.latest.get(node_id, {}).get("heartbeat")

    def read_feedback(self, node_id, timeout=0.0):
        """Return last cached (pos, vel) or (None, None)."""
        return self.latest.get(node_id, {}).get("feedback", (None, None))
    

    def start_listener(self):
        """Continuously read CAN frames and cache latest heartbeat/feedback."""
        if not getattr(self, "bus", None):
            raise RuntimeError("CAN bus not initialized")
        if self._listener_running:
            return
        self._listener_running = True

        def loop():
            import struct, time
            while getattr(self, "bus", None):
                try:
                    msg = self.bus.recv(timeout=0.0)
                    if not msg:
                        time.sleep(0.001)
                        continue

                    node_id = msg.arbitration_id >> 5
                    cmd_id  = msg.arbitration_id & 0x1F

                    if cmd_id == 0x01:        # Heartbeat
                        try:
                            err, state, *_ = struct.unpack("<IBBB", msg.data[:7])
                            self.latest.setdefault(node_id, {})["heartbeat"] = (err, state)
                        except struct.error:
                            pass

                    elif cmd_id == 0x09 and not msg.is_remote_frame:  # Feedback reply
                        try:
                            pos, vel = struct.unpack("<ff", msg.data[:8])
                            self.latest.setdefault(node_id, {})["feedback"] = (pos, vel)
                        except struct.error:
                            pass

                    self.latest.setdefault(node_id, {})["updated"] = time.time()

                except Exception as e:
                    print(f"⚠️ Listener error: {e}")
                    time.sleep(0.05)

            self._listener_running = False

        import threading
        threading.Thread(target=loop, daemon=True).start()
        print("🟢 CAN listener thread started.")




    def make_config_json(
        self,
        node_id: int,
        vel_limit: float,
        vel_tol: float,
        torque_min: float,
        torque_max: float,
        pos_gain: float = None,
        vel_gain: float = None,
        vel_int_gain: float = None,
        save_path: str = None
    ):
        """
        Generate an ODrive config JSON for a given node with custom velocity/torque limits
        and optional controller gains. If save_path is given, write it to file; otherwise
        return the dict.
        """

        cfg = {
            "config.dc_bus_overvoltage_trip_level": 36,
            "config.dc_bus_undervoltage_trip_level": 10.5,
            "config.dc_max_positive_current": 1e9,
            "config.dc_max_negative_current": -1e9,
            "config.brake_resistor0.enable": True,
            "config.brake_resistor0.resistance": 2,

            "axis0.config.motor.motor_type": 0,
            "axis0.config.motor.pole_pairs": 20,
            "axis0.config.motor.torque_constant": 0.09188888888888888,
            "axis0.config.motor.current_soft_max": 22,
            "axis0.config.motor.current_hard_max": 38.6,
            "axis0.config.motor.calibration_current": 18,
            "axis0.config.motor.resistance_calib_max_voltage": 5,

            "axis0.config.calibration_lockin.current": 18,
            "axis0.motor.motor_thermistor.config.enabled": False,

            "axis0.controller.config.control_mode": 3,
            "axis0.controller.config.input_mode": 3,
            "axis0.controller.config.vel_limit": vel_limit,
            "axis0.controller.config.vel_limit_tolerance": vel_tol,

            "axis0.config.torque_soft_min": torque_min,
            "axis0.config.torque_soft_max": torque_max,

            "can.config.protocol": 1,
            "can.config.baud_rate": 250000,

            "axis0.config.can.node_id": node_id,
            "axis0.config.can.heartbeat_msg_rate_ms": 100,
            "axis0.config.can.encoder_msg_rate_ms": 10,
            "axis0.config.can.iq_msg_rate_ms": 10,
            "axis0.config.can.torques_msg_rate_ms": 10,
            "axis0.config.can.error_msg_rate_ms": 10,
            "axis0.config.can.temperature_msg_rate_ms": 10,
            "axis0.config.can.bus_voltage_msg_rate_ms": 10,

            "axis0.config.enable_watchdog": False,
            "axis0.config.load_encoder": 13,
            "axis0.config.commutation_encoder": 13,

            "config.enable_uart_a": False
        }

        # --- optional controller gains ---
        if pos_gain is not None:
            cfg["axis0.controller.config.pos_gain"] = pos_gain
        if vel_gain is not None:
            cfg["axis0.controller.config.vel_gain"] = vel_gain
        if vel_int_gain is not None:
            cfg["axis0.controller.config.vel_integrator_gain"] = vel_int_gain

        

        # --- save to file if requested ---
        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            with open(save_path, "w") as f:
                json.dump(cfg, f, indent=2)
            print(f"💾  Wrote {save_path} for node {node_id}")

        return cfg




    def flash_config_over_can(self, nodes, vel_limit, vel_tol, torque_min, torque_max,
                          pos_gain=None, vel_gain=None, vel_int_gain=None):
        """
        For each node id in `nodes`, generate config.json and push it over CAN
        using ODrive-CAN/can_restore_config.py.
        """
        import os, subprocess

        # --- directory layout ---
        base_dir = os.path.dirname(os.path.abspath(__file__))                 # .../me6705_final_project/robot_driver
        project_root = os.path.abspath(os.path.join(base_dir, ".."))          # .../me6705_final_project
        can_control_dir = os.path.join(project_root, "can_control")
        odrive_can_dir = os.path.join(project_root, "ODrive-CAN")

        # --- critical paths ---
        can_restore_path = os.path.join(odrive_can_dir, "can_restore_config.py")
        config_path = os.path.join(can_control_dir, "config.json")

        # endpoints lives in can_control/, not control_gui_1
        endpoints_candidates = [
            os.path.join(can_control_dir, "flat_endpoints.json"),
            os.path.join(base_dir, "flat_endpoints.json"),
        ]
        endpoints_path = next((p for p in endpoints_candidates if os.path.isfile(p)), None)

        # --- sanity checks ---
        if not os.path.isfile(can_restore_path):
            print(f"❌ Missing can_restore_config.py at: {can_restore_path}")
            return
        if endpoints_path is None:
            print("❌ Could not find flat_endpoints.json. Tried:")
            for p in endpoints_candidates:
                print("   -", p)
            return

        print("\n📂 Using paths:")
        print("   can_restore_config.py:", can_restore_path)
        print("   endpoints JSON       :", endpoints_path)
        print("   config JSON (temp)   :", config_path, "\n")

        for node_id in nodes:
            print(f"⚙️  Generating config.json for node {node_id}")
            self.make_config_json(
                node_id=node_id,
                vel_limit=vel_limit,
                vel_tol=vel_tol,
                torque_min=torque_min,
                torque_max=torque_max,
                pos_gain = pos_gain,
                vel_gain = vel_gain,
                vel_int_gain = vel_int_gain,
                save_path=config_path,
            )

            cmd = [
                "python3",
                can_restore_path,
                "--channel", "can0",
                "--node-id", str(node_id),
                "--endpoints-json", endpoints_path,
                "--config", config_path,
                "--save-config",
            ]

            print(f"🚀  Flashing Node {node_id} via CAN…")
            print("   ↳", " ".join(cmd))

            try:
                result = subprocess.run(cmd, capture_output=True, text=True, check=True)
                if result.stdout.strip():
                    print(result.stdout.strip())
                if result.stderr.strip():
                    print("stderr:", result.stderr.strip())
                print(f"✅  Node {node_id} flashed successfully.\n")
            except subprocess.CalledProcessError as e:
                print(f"❌  Node {node_id} flash failed:")
                if e.stdout:
                    print("stdout:", e.stdout.strip())
                if e.stderr:
                    print("stderr:", e.stderr.strip())
                print()


    def save_and_reboot_nodes(self, nodes):
        """Send SAVE_CONFIG and REBOOT commands via CAN for the given node(s)."""
        try:
            bus = can.interface.Bus(channel='can0', bustype='socketcan')
        except Exception as e:
            print(f"⚠️ Failed to open CAN bus: {e}")
            return

        for node_id in nodes:
            try:
                base = (0x07 << 5) | node_id
                save_id = base + 0x0C
                reboot_id = base + 0x0D

                # send SAVE_CONFIG
                bus.send(can.Message(arbitration_id=save_id,
                                     is_extended_id=False,
                                     data=[]))
                print(f"💾 Sent SAVE_CONFIG to node {node_id} (0x{save_id:X})")

                # small delay between save and reboot
                import time
                time.sleep(0.2)

                # send REBOOT
                bus.send(can.Message(arbitration_id=reboot_id,
                                     is_extended_id=False,
                                     data=[]))
                print(f"🔁 Sent REBOOT to node {node_id} (0x{reboot_id:X})")

            except Exception as e:
                print(f"❌ Error saving/rebooting node {node_id}: {e}")

        bus.shutdown()
        print("✅ Save/Reboot sequence complete.\n" + "-" * 60)



        """
    def listen_for_heartbeat(self, node_id, duration=0.0):
       """ """Return (error, state) if heartbeat frame for this node is already waiting.""""""
        msg = self.bus.recv(duration)     # non-blocking when 0
        if msg is None:
            return None
        cmd_id = msg.arbitration_id & 0x1F
        nid = msg.arbitration_id >> 5
        if cmd_id == 0x01 and nid == node_id:
            try:
                error, state, *_ = struct.unpack('<IBBB', msg.data[:7])
                return (error, state)
            except struct.error:
                pass
        return None

    def read_feedback(self, node_id: int, timeout: float = 0.0):
      """  """Try to read latest feedback frame without blocking.""" """
        if not getattr(self, 'bus', None):
            raise RuntimeError("CAN bus not initialized")


        msg = self.bus.recv(timeout)
        if not msg:
            return None, None

        nid = msg.arbitration_id >> 5
        cmd_id = msg.arbitration_id & 0x1F
        if cmd_id == 0x09 and nid == node_id and not msg.is_remote_frame:
            try:
                pos, vel = struct.unpack('<ff', msg.data[:8])
                return pos, vel
            except struct.error:
                pass
        return None, None
"""

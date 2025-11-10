""" 
Backend for odrive GUI
Functions:


"""

import can, struct, time, subprocess


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

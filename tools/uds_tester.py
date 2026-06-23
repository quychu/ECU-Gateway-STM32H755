"""
uds_tester.py — ECU Gateway Diagnostic & IDS Tester
=====================================================
Target   : STM32H755 ECU Gateway (NUCLEO-H755ZI-Q)
Interface: LAWICEL / slcan  (USB-to-CAN, python-can)

┌─────────────────────────────────────────────────────────┐
│  PANEL 1 — UDS Diagnostics : ISO-TP + UDS services      │
│  PANEL 2 — IDS Injection   : Inject frames, trigger R1-R4│
│  PANEL 3 — Custom Frame    : Free-form CAN frame sender  │
└─────────────────────────────────────────────────────────┘

Requires:
    pip install python-can can-isotp udsoncan
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import queue
import time
import struct

# ── Lazy import guard ──────────────────────────────────────────────────────────
try:
    import can
    import isotp
    import udsoncan
    from udsoncan.connections import PythonIsoTpConnection
    from udsoncan.client import Client
    import udsoncan.services as uds_svc
    LIBS_OK = True
except ImportError as _e:
    LIBS_OK = False
    _MISSING = str(_e)


# ── Constants ──────────────────────────────────────────────────────────────────
UDS_TX_ID   = 0x7E0   # Tester → ECU  (ISO 15765-4 §6.3.2)
UDS_RX_ID   = 0x7E8   # ECU → Tester

DEFAULT_COM = "COM4"
DEFAULT_BIT = "500000"

DID_MAP = {
    0xF187: "SW Part Number (F187)",
    0xF18C: "ECU Serial Number (F18C)",
    0xF195: "SW Supplier Info (F195)",
}

# IDS test frame IDs (must match ids_rule.c whitelist)
IDS_WHITELIST_ID = 0x110   # known-good ID in gateway range (period=500ms)
IDS_UNKNOWN_ID   = 0x200   # outside whitelist → R1
IDS_FLOOD_COUNT  = 30      # frames for R4 burst test


# ── DID Codec ─────────────────────────────────────────────────────────────────
class RawBytesCodec(udsoncan.DidCodec if LIBS_OK else object):
    """Pass-through codec — returns raw bytes. Required by udsoncan before send."""
    def encode(self, val):
        if isinstance(val, (bytes, bytearray)):
            return val
        return bytes(val)

    def decode(self, payload):
        return bytes(payload)

    def __len__(self):
        raise udsoncan.DidCodec.ReadAllRemainingData


def _make_did_codecs():
    if not LIBS_OK:
        return {}
    return {did: RawBytesCodec() for did in DID_MAP}


SESSION_MAP = {
    "Default (0x01)":     0x01,
    "Extended (0x03)":    0x03,
    "Programming (0x02)": 0x02,
}

NRC_DESC = {
    0x10: "generalReject",         0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported", 0x13: "incorrectMessageLength",
    0x22: "conditionsNotCorrect",  0x24: "requestSequenceError",
    0x31: "requestOutOfRange",     0x33: "securityAccessDenied",
    0x35: "invalidKey",            0x36: "exceededNumberOfAttempts",
    0x37: "requiredTimeDelayNotExpired",
    0x78: "requestCorrectlyReceivedResponsePending",
}


# ── Thread-safe stdout redirector ─────────────────────────────────────────────
class QueueRedirector:
    def __init__(self, log_queue: queue.Queue):
        self._q = log_queue

    def write(self, text: str):
        if text.strip():
            self._q.put(text)

    def flush(self):
        pass


# ── CAN Bus wrapper (raw — for IDS + Custom frame sending) ────────────────────
class CanBus:
    """
    Thin wrapper around python-can Bus.
    Shared between UDS worker, IDS worker, and Custom frame sender.
    """
    def __init__(self):
        self._bus = None

    def connect(self, com_port: str, bitrate: int) -> bool:
        try:
            self._bus = can.interface.Bus(
                interface="slcan",
                channel=com_port,
                bitrate=bitrate,
            )
            return True
        except Exception as ex:
            self._bus = None
            raise ex

    def send(self, arb_id: int, data: bytes, dlc: int = None) -> bool:
        """Send a raw CAN frame. dlc defaults to len(data)."""
        if self._bus is None:
            return False
        payload = data[:8]
        actual_dlc = dlc if dlc is not None else len(payload)
        # Pad if DLC > actual data (simulate DLC mismatch for R2 test)
        if actual_dlc > len(payload):
            payload = payload + b'\xCC' * (actual_dlc - len(payload))
        msg = can.Message(
            arbitration_id=arb_id,
            data=payload[:actual_dlc],
            is_extended_id=False,
        )
        try:
            self._bus.send(msg)
            return True
        except Exception:
            return False

    def disconnect(self):
        try:
            if self._bus:
                self._bus.shutdown()
        except Exception:
            pass
        self._bus = None

    @property
    def raw_bus(self):
        return self._bus

    @property
    def connected(self):
        return self._bus is not None


# ── UDS Worker ────────────────────────────────────────────────────────────────
class UdsWorker:
    """ISO-TP + UDS stack on top of shared CanBus."""

    def __init__(self, can_bus: CanBus, log_fn):
        self._bus  = can_bus
        self._log  = log_fn
        self._stack = None
        self._conn  = None

    def setup_isotp(self) -> bool:
        try:
            tp_addr = isotp.Address(
                isotp.AddressingMode.Normal_11bits,
                txid=UDS_TX_ID,
                rxid=UDS_RX_ID,
            )
            tp_params = {
                "stmin": 0, "blocksize": 0, "tx_padding": 0xCC,
                "rx_flowcontrol_timeout": 1000,
                "rx_consecutive_frame_timeout": 150,
            }
            self._stack = isotp.CanStack(
                bus=self._bus.raw_bus,
                address=tp_addr,
                params=tp_params,
            )
            self._conn = PythonIsoTpConnection(self._stack)
            self._conn.open()
            self._log(f"[UDS]  ISO-TP ready  TX=0x{UDS_TX_ID:03X} RX=0x{UDS_RX_ID:03X}")
            return True
        except Exception as ex:
            self._log(f"[UDS]  ISO-TP setup failed: {ex}")
            return False

    def teardown(self):
        try:
            if self._conn:
                self._conn.close()
        except Exception:
            pass
        self._conn  = None
        self._stack = None

    def _client(self, timeout: float = 2.0) -> Client:
        cfg = udsoncan.configs.default_client_config.copy()
        cfg["request_timeout"]  = timeout
        cfg["p2_timeout"]       = timeout
        cfg["p2_star_timeout"]  = timeout
        cfg["data_identifiers"] = _make_did_codecs()
        return Client(self._conn, config=cfg)

    def _nrc_str(self, ex) -> str:
        code = getattr(ex, "code", None)
        if code is None:
            return str(ex)
        return f"NRC 0x{code:02X} ({NRC_DESC.get(code, 'unknownNRC')})"

    # ── UDS services ──────────────────────────────────────────────────────────
    def tester_present(self):
        self._log("[UDS]  TesterPresent (0x3E) →")
        try:
            with self._client() as c:
                c.tester_present()
            self._log("[UDS]  ✓  TesterPresent OK  (S3 timer reset)")
        except Exception as ex:
            self._log(f"[UDS]  ✗  {self._nrc_str(ex)}")

    def change_session(self, session_id: int, session_name: str):
        self._log(f"[UDS]  DiagnosticSessionControl (0x10) → {session_name} ...")
        try:
            with self._client() as c:
                c.change_session(session_id)
            self._log(f"[UDS]  ✓  Session → {session_name}")
        except Exception as ex:
            self._log(f"[UDS]  ✗  {self._nrc_str(ex)}")

    def read_did(self, did: int):
        label = DID_MAP.get(did, f"DID 0x{did:04X}")
        self._log(f"[UDS]  ReadDataByIdentifier (0x22) → {label} ...")
        try:
            with self._client() as c:
                resp = c.read_data_by_identifier([did])
            raw = resp.service_data.values.get(did)
            if isinstance(raw, (bytes, bytearray)):
                text = raw.decode("ascii", errors="replace").strip("\x00")
                self._log(f"[UDS]  ✓  ASCII : {text!r}")
                self._log(f"         HEX   : {raw.hex(' ').upper()}")
            else:
                self._log(f"[UDS]  ✓  {label} = {raw}")
        except Exception as ex:
            self._log(f"[UDS]  ✗  ReadDID 0x{did:04X}: {self._nrc_str(ex)}")

    def read_all_dids(self):
        self._log("[UDS]  ── ReadDID batch (F187 / F18C / F195) ──")
        for did in DID_MAP:
            self.read_did(did)
            time.sleep(0.05)

    def security_access(self):
        self._log("[UDS]  SecurityAccess (0x27) →")
        try:
            with self._client(timeout=3.0) as c:
                c.change_session(0x03)
                self._log("[UDS]  Pre: Extended session OK")
                c.tester_present()

                seed_resp = c.request_seed(0x01)
                raw_seed  = seed_resp.service_data.seed
                seed_int  = int.from_bytes(raw_seed, "big") if isinstance(raw_seed, (bytes, bytearray)) else int(raw_seed)
                self._log(f"[UDS]  Seed : 0x{seed_int:08X}")

                key_int   = (~seed_int) & 0xFFFF_FFFF
                key_bytes = key_int.to_bytes(4, "big")
                self._log(f"[UDS]  Key  : 0x{key_int:08X}  (~seed)")

                c.send_key(0x02, key_bytes)
            self._log("[UDS]  ✓  SecurityAccess UNLOCKED (level 0x01)")
        except udsoncan.exceptions.SecurityAccessDenied:
            self._log("[UDS]  ✗  SecurityAccess DENIED")
        except Exception as ex:
            self._log(f"[UDS]  ✗  {self._nrc_str(ex)}")

    def run_full_sequence(self):
        self._log("=" * 56)
        self._log("[SEQ]  ── Full UDS test sequence START ──")
        self._log("=" * 56)
        self.tester_present();         time.sleep(0.05)
        self.change_session(0x03, "Extended (0x03)"); time.sleep(0.05)
        self.read_all_dids();          time.sleep(0.05)
        self.security_access();        time.sleep(0.05)
        self.change_session(0x01, "Default (0x01)")
        self._log("=" * 56)
        self._log("[SEQ]  ── Full UDS test sequence END ──")
        self._log("=" * 56)


# ── IDS Worker ────────────────────────────────────────────────────────────────
class IdsWorker:
    """
    Sends raw CAN frames to trigger IDS rules on the ECU.
    Does NOT use ISO-TP — injects frames directly at CAN layer.

    Rule test mapping:
      R1 — Send unknown ID (outside ECU whitelist)
      R2 — Send known ID with wrong DLC
      R3 — Send same ID twice rapidly (delta < expected_period - jitter)
      R4 — Flood: many frames in short window (token bucket drain)
    """

    def __init__(self, can_bus: CanBus, log_fn):
        self._bus = can_bus
        self._log = log_fn

    def send_safe_frame(self):
        """Send a clean frame — should pass all R1-R4 rules."""
        self._log(f"[IDS]  ▶  Safe frame  ID=0x{IDS_WHITELIST_ID:03X}  DLC=8  data=0xDE 0xAD ...")
        data = bytes([0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04])
        ok = self._bus.send(IDS_WHITELIST_ID, data)
        self._log(f"[IDS]  {'✓ Sent (should PASS all rules)' if ok else '✗ Send failed'}")

    def trigger_r1_unknown_id(self):
        """R1 — Unknown CAN ID not in ECU whitelist."""
        self._log(f"[IDS]  ▶  R1 test — Unknown ID=0x{IDS_UNKNOWN_ID:03X} (not whitelisted)")
        data = bytes([0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x00, 0x00, 0x00])
        ok = self._bus.send(IDS_UNKNOWN_ID, data)
        self._log(f"[IDS]  {'✓ Sent → ECU should log IDS ALERT R1' if ok else '✗ Send failed'}")

    def trigger_r2_wrong_dlc(self):
        """R2 — Known ID (0x110) with DLC=3 instead of expected DLC=8."""
        self._log(f"[IDS]  ▶  R2 test — ID=0x{IDS_WHITELIST_ID:03X}  DLC=3  (expected 8)")
        data = bytes([0x11, 0x22, 0x33])
        ok = self._bus.send(IDS_WHITELIST_ID, data, dlc=3)
        self._log(f"[IDS]  {'✓ Sent → ECU should log IDS ALERT R2' if ok else '✗ Send failed'}")

    def trigger_r3_rapid_inject(self):
        """R3 — Two frames with same ID separated by only 50 ms (period=500 ms → delta < 450 ms lower bound)."""
        self._log(f"[IDS]  ▶  R3 test — ID=0x{IDS_WHITELIST_ID:03X}  2 frames 50 ms apart (period=500 ms)")
        data = bytes([0xAA, 0xBB, 0xCC, 0xDD, 0x01, 0x02, 0x03, 0x04])
        self._bus.send(IDS_WHITELIST_ID, data)
        self._log(f"[IDS]  Frame 1 sent — waiting 50 ms ...")
        time.sleep(0.05)   # 50 ms << (500 - 50) = 450 ms lower bound → R3 fires
        self._bus.send(IDS_WHITELIST_ID, bytes([0xAA, 0xBB, 0xCC, 0xDD, 0x05, 0x06, 0x07, 0x08]))
        self._log(f"[IDS]  ✓ Frame 2 sent → ECU should log IDS ALERT R3 (delta < 450 ms)")

    def trigger_r4_burst_flood(self):
        """R4 — Flood bus with IDS_FLOOD_COUNT frames rapidly to drain token bucket."""
        self._log(f"[IDS]  ▶  R4 test — Flooding {IDS_FLOOD_COUNT} frames in ~200 ms ...")
        ok_count = 0
        for i in range(IDS_FLOOD_COUNT):
            # 0x55 = neutral data byte (PCI nibble 0x5 = undefined → no ISO-TP parse noise)
            frame_data = bytes([0x55, i & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            can_id = IDS_WHITELIST_ID + (i % 16)   # spread across 0x110-0x11F
            if self._bus.send(can_id, frame_data):
                ok_count += 1
            time.sleep(0.006)  # 6 ms between frames → ~180 ms total for 30 frames
        self._log(f"[IDS]  ✓ Sent {ok_count}/{IDS_FLOOD_COUNT} frames → ECU should log IDS ALERT R4")

    def run_full_ids_sequence(self):
        """Run all 4 IDS attack scenarios with delays between each."""
        self._log("=" * 56)
        self._log("[IDS]  ── Full IDS test sequence START ──")
        self._log("=" * 56)

        self.send_safe_frame()
        time.sleep(1.0)

        self.trigger_r1_unknown_id()
        time.sleep(0.5)

        self.trigger_r2_wrong_dlc()
        time.sleep(0.5)

        self.trigger_r3_rapid_inject()
        time.sleep(1.0)

        self.trigger_r4_burst_flood()
        time.sleep(0.5)

        self._log("=" * 56)
        self._log("[IDS]  ── Full IDS test sequence END ──")
        self._log("[IDS]  Check ECU UART log for IDS_ALERT R1/R2/R3/R4 + LED2 blink")
        self._log("=" * 56)


# ── Main GUI ───────────────────────────────────────────────────────────────────
class App(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("ECU Gateway — UDS / IDS Tester  (STM32H755)")
        self.resizable(True, True)
        self.minsize(780, 700)

        self._log_queue: queue.Queue = queue.Queue()
        self._can_bus   = CanBus()
        self._uds       = None
        self._ids       = None
        self._connected = False

        self._build_ui()
        self._poll_log()

        if not LIBS_OK:
            self._log(f"[WARN]  Missing library: {_MISSING}")
            self._log("[WARN]  Run:  pip install python-can can-isotp udsoncan")

    # ── UI construction ────────────────────────────────────────────────────────
    def _build_ui(self):
        # Init button registry FIRST — panel builders append to it
        self._all_action_buttons = []

        # ── Top bar: COM / bitrate / connect ──────────────────────────────────
        top = ttk.Frame(self, padding=6)
        top.pack(fill="x")

        ttk.Label(top, text="COM Port:").grid(row=0, column=0, sticky="w")
        self._com_var = tk.StringVar(value=DEFAULT_COM)
        ttk.Entry(top, textvariable=self._com_var, width=8).grid(
            row=0, column=1, padx=(2, 12))

        ttk.Label(top, text="Bitrate:").grid(row=0, column=2, sticky="w")
        self._bps_var = tk.StringVar(value=DEFAULT_BIT)
        ttk.Combobox(top, textvariable=self._bps_var,
                     values=["125000", "250000", "500000", "1000000"],
                     width=10).grid(row=0, column=3, padx=(2, 12))

        self._btn_conn = ttk.Button(top, text="Connect",
                                    command=self._toggle_connect, width=12)
        self._btn_conn.grid(row=0, column=4, padx=4)

        self._status_var = tk.StringVar(value="● Disconnected")
        ttk.Label(top, textvariable=self._status_var,
                  foreground="red").grid(row=0, column=5, padx=8)

        # ── Notebook: 3 panels ────────────────────────────────────────────────
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=False, padx=6, pady=4)

        self._tab_uds    = ttk.Frame(nb, padding=6)
        self._tab_ids    = ttk.Frame(nb, padding=6)
        self._tab_custom = ttk.Frame(nb, padding=6)

        nb.add(self._tab_uds,    text="  UDS Diagnostics  ")
        nb.add(self._tab_ids,    text="  IDS Injection  ")
        nb.add(self._tab_custom, text="  Custom Frame  ")

        self._build_uds_panel()
        self._build_ids_panel()
        self._build_custom_panel()

        # ── Progress label ────────────────────────────────────────────────────
        self._prog_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self._prog_var,
                  foreground="navy").pack(anchor="w", padx=8)

        # ── Log area ──────────────────────────────────────────────────────────
        log_frame = ttk.LabelFrame(self, text="Log", padding=4)
        log_frame.pack(fill="both", expand=True, padx=6, pady=4)

        self._log_widget = scrolledtext.ScrolledText(
            log_frame, height=14, font=("Consolas", 9),
            state="disabled", wrap="none")
        self._log_widget.pack(fill="both", expand=True)

        btn_row = ttk.Frame(log_frame)
        btn_row.pack(fill="x", pady=(2, 0))
        ttk.Button(btn_row, text="Clear Log",
                   command=self._clear_log, width=12).pack(side="right")

        self._set_buttons_state("disabled")

    # ── Panel 1: UDS ──────────────────────────────────────────────────────────
    def _build_uds_panel(self):
        f = self._tab_uds

        ttk.Label(f, text="UDS Services (ISO 14229 / ISO-TP over CAN 0x7E0→0x7E8)",
                  font=("", 9, "bold")).grid(row=0, column=0, columnspan=3,
                  sticky="w", pady=(0, 6))

        btn_defs = [
            ("TesterPresent (3E)",    self._do_tester_present),
            ("Default Session (10)",  lambda: self._do_session(0x01, "Default (0x01)")),
            ("Extended Session (10)", lambda: self._do_session(0x03, "Extended (0x03)")),
            ("Read DID F187",         lambda: self._do_read_did(0xF187)),
            ("Read DID F18C",         lambda: self._do_read_did(0xF18C)),
            ("Read DID F195",         lambda: self._do_read_did(0xF195)),
            ("Read ALL DIDs",         self._do_read_all),
            ("SecurityAccess (27)",   self._do_security),
            ("▶  Full UDS Sequence",  self._do_full_uds_seq),
        ]

        cols = 3
        for i, (label, cmd) in enumerate(btn_defs):
            b = ttk.Button(f, text=label, command=cmd, width=22)
            b.grid(row=1 + i // cols, column=i % cols, padx=4, pady=2, sticky="ew")
            self._all_action_buttons.append(b)

        for c in range(cols):
            f.columnconfigure(c, weight=1)

    # ── Panel 2: IDS ──────────────────────────────────────────────────────────
    def _build_ids_panel(self):
        f = self._tab_ids

        ttk.Label(f, text="IDS Injection Tests — triggers ECU IdsRuleTask R1–R4",
                  font=("", 9, "bold")).grid(row=0, column=0, columnspan=2,
                  sticky="w", pady=(0, 6))

        # Info table
        info = ttk.LabelFrame(f, text="Rule Reference", padding=4)
        info.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        rules = [
            ("R1", "ID Whitelist",   f"Unknown ID 0x{IDS_UNKNOWN_ID:03X} → not in ECU whitelist [0x100-0x1FF + 0x7E0]"),
            ("R2", "DLC Check",      f"ID 0x{IDS_WHITELIST_ID:03X} DLC=3 → expected DLC=8"),
            ("R3", "Period Anomaly", f"ID 0x{IDS_WHITELIST_ID:03X} twice 50ms apart → period=500ms, lower_bound=450ms"),
            ("R4", "Burst Flood",    f"Flood {IDS_FLOOD_COUNT} frames → drains token bucket (capacity=15, rate=1/100ms)"),
        ]
        for row_idx, (rule, name, desc) in enumerate(rules):
            ttk.Label(info, text=rule, font=("Consolas", 9, "bold"),
                      foreground="red").grid(row=row_idx, column=0, sticky="w", padx=(2, 6))
            ttk.Label(info, text=name, font=("", 9, "bold"),
                      width=16).grid(row=row_idx, column=1, sticky="w")
            ttk.Label(info, text=desc, font=("Consolas", 8),
                      foreground="#444").grid(row=row_idx, column=2, sticky="w")

        # Buttons
        btn_defs = [
            ("✅  Safe Frame (should pass)",    self._do_ids_safe),
            ("🔴  R1 — Unknown ID inject",      self._do_ids_r1),
            ("🔴  R2 — Wrong DLC inject",       self._do_ids_r2),
            ("🔴  R3 — Rapid period inject",    self._do_ids_r3),
            ("🔴  R4 — Burst flood (DoS)",      self._do_ids_r4),
            ("▶   Full IDS Sequence",            self._do_full_ids_seq),
        ]

        cols = 2
        for i, (label, cmd) in enumerate(btn_defs):
            b = ttk.Button(f, text=label, command=cmd, width=30)
            b.grid(row=3 + i // cols, column=i % cols, padx=4, pady=2, sticky="ew")
            self._all_action_buttons.append(b)

        for c in range(cols):
            f.columnconfigure(c, weight=1)

        ttk.Label(f, text="→ Watch ECU UART log for [CM4] IDS_ALERT R1/R2/R3/R4 + LED2 (yellow) blink",
                  font=("", 8), foreground="navy").grid(
                  row=7, column=0, columnspan=2, sticky="w", pady=(6, 0))

    # ── Panel 3: Custom Frame ─────────────────────────────────────────────────
    def _build_custom_panel(self):
        f = self._tab_custom

        ttk.Label(f, text="Custom CAN Frame Sender — free-form inject",
                  font=("", 9, "bold")).grid(row=0, column=0, columnspan=4,
                  sticky="w", pady=(0, 10))

        # CAN ID
        ttk.Label(f, text="CAN ID (hex):").grid(row=1, column=0, sticky="e", padx=(0,4))
        self._custom_id_var = tk.StringVar(value="110")
        ttk.Entry(f, textvariable=self._custom_id_var, width=8,
                  font=("Consolas", 10)).grid(row=1, column=1, sticky="w")
        ttk.Label(f, text="0x", font=("Consolas", 10),
                  foreground="#666").grid(row=1, column=0, sticky="e", padx=(30, 34))

        # DLC
        ttk.Label(f, text="DLC (0–8):").grid(row=2, column=0, sticky="e", padx=(0, 4), pady=4)
        self._custom_dlc_var = tk.StringVar(value="8")
        ttk.Spinbox(f, textvariable=self._custom_dlc_var,
                    from_=0, to=8, width=4).grid(row=2, column=1, sticky="w")

        # Data
        ttk.Label(f, text="Data (hex bytes,\nspace-separated):").grid(
            row=3, column=0, sticky="ne", padx=(0, 4), pady=4)
        self._custom_data_var = tk.StringVar(value="DE AD BE EF 01 02 03 04")
        data_entry = ttk.Entry(f, textvariable=self._custom_data_var,
                               width=32, font=("Consolas", 10))
        data_entry.grid(row=3, column=1, columnspan=3, sticky="ew", pady=4)

        # Repeat count
        ttk.Label(f, text="Repeat:").grid(row=4, column=0, sticky="e", padx=(0, 4))
        self._custom_repeat_var = tk.StringVar(value="1")
        ttk.Spinbox(f, textvariable=self._custom_repeat_var,
                    from_=1, to=200, width=6).grid(row=4, column=1, sticky="w")

        ttk.Label(f, text="  Interval (ms):").grid(row=4, column=2, sticky="e", padx=(0, 4))
        self._custom_interval_var = tk.StringVar(value="100")
        ttk.Entry(f, textvariable=self._custom_interval_var, width=6).grid(
            row=4, column=3, sticky="w")

        # Send button
        self._btn_custom_send = ttk.Button(
            f, text="▶  Send Custom Frame(s)", command=self._do_custom_send, width=26)
        self._btn_custom_send.grid(row=5, column=0, columnspan=4,
                                   pady=12, sticky="ew")
        self._all_action_buttons.append(self._btn_custom_send)

        # Preview label
        self._custom_preview_var = tk.StringVar(value="")
        ttk.Label(f, textvariable=self._custom_preview_var,
                  font=("Consolas", 9), foreground="navy").grid(
                  row=6, column=0, columnspan=4, sticky="w")

        # Live preview on input change
        for var in [self._custom_id_var, self._custom_dlc_var, self._custom_data_var]:
            var.trace_add("write", lambda *a: self._update_custom_preview())

        self._update_custom_preview()

        for c in range(4):
            f.columnconfigure(c, weight=1)

    def _update_custom_preview(self):
        try:
            cid   = int(self._custom_id_var.get().strip(), 16)
            dlc   = int(self._custom_dlc_var.get().strip())
            data  = bytes(int(x, 16) for x in self._custom_data_var.get().split())
            self._custom_preview_var.set(
                f"Preview: ID=0x{cid:03X}  DLC={dlc}  "
                f"data=[{' '.join(f'{b:02X}' for b in data[:dlc])}]")
        except Exception:
            self._custom_preview_var.set("Preview: (invalid input)")

    # ── Log helpers ───────────────────────────────────────────────────────────
    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self._log_queue.put(f"[{ts}] {msg}\n")

    def _poll_log(self):
        try:
            while True:
                line = self._log_queue.get_nowait()
                self._log_widget.config(state="normal")
                self._log_widget.insert("end", line)
                self._log_widget.see("end")
                self._log_widget.config(state="disabled")
        except queue.Empty:
            pass
        self.after(50, self._poll_log)

    def _clear_log(self):
        self._log_widget.config(state="normal")
        self._log_widget.delete("1.0", "end")
        self._log_widget.config(state="disabled")

    # ── Connect / Disconnect ──────────────────────────────────────────────────
    def _toggle_connect(self):
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if not LIBS_OK:
            messagebox.showerror("Missing libs",
                "Run:  pip install python-can can-isotp udsoncan")
            return
        com = self._com_var.get().strip()
        try:
            bps = int(self._bps_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid bitrate.")
            return

        self._btn_conn.config(state="disabled")
        self._prog_var.set("Connecting …")

        def _task():
            try:
                self._can_bus.connect(com, bps)
                self._log(f"[CAN]  Connected {com} @ {bps} bps")

                # Setup ISO-TP / UDS layer on same bus
                self._uds = UdsWorker(self._can_bus, self._log)
                uds_ok = self._uds.setup_isotp()

                # IDS worker shares same raw bus (no ISO-TP layer needed)
                self._ids = IdsWorker(self._can_bus, self._log)

                self.after(0, self._on_connected if uds_ok else self._on_connect_failed)
            except Exception as ex:
                self._log(f"[ERR]  {ex}")
                self.after(0, self._on_connect_failed)

        threading.Thread(target=_task, daemon=True).start()

    def _on_connected(self):
        self._connected = True
        self._btn_conn.config(text="Disconnect", state="normal")
        self._status_var.set("● Connected")
        self._prog_var.set("")
        self._set_buttons_state("normal")
        self._log("[GUI]  Ready — UDS / IDS / Custom panels active.")

    def _on_connect_failed(self):
        self._can_bus.disconnect()
        self._btn_conn.config(state="normal")
        self._prog_var.set("Connection failed.")

    def _disconnect(self):
        if self._uds:
            self._uds.teardown()
            self._uds = None
        self._ids = None
        self._can_bus.disconnect()
        self._connected = False
        self._btn_conn.config(text="Connect")
        self._status_var.set("● Disconnected")
        self._set_buttons_state("disabled")
        self._prog_var.set("")
        self._log("[CAN]  Disconnected.")

    # ── Button state helper ───────────────────────────────────────────────────
    def _set_buttons_state(self, state: str):
        for b in self._all_action_buttons:
            b.config(state=state)

    # ── Run helper — fire-and-forget background thread ────────────────────────
    def _run(self, fn, label: str = ""):
        if not self._connected:
            return
        self._set_buttons_state("disabled")
        if label:
            self._prog_var.set(f"Running: {label} …")

        def _task():
            try:
                fn()
            finally:
                self.after(0, self._on_task_done)

        threading.Thread(target=_task, daemon=True).start()

    def _on_task_done(self):
        self._prog_var.set("Done.")
        self._set_buttons_state("normal")

    # ── UDS callbacks ─────────────────────────────────────────────────────────
    def _do_tester_present(self):
        self._run(self._uds.tester_present, "TesterPresent")

    def _do_session(self, sid: int, name: str):
        self._run(lambda: self._uds.change_session(sid, name), f"Session {name}")

    def _do_read_did(self, did: int):
        self._run(lambda: self._uds.read_did(did), f"ReadDID 0x{did:04X}")

    def _do_read_all(self):
        self._run(self._uds.read_all_dids, "Read All DIDs")

    def _do_security(self):
        self._run(self._uds.security_access, "SecurityAccess")

    def _do_full_uds_seq(self):
        self._run(self._uds.run_full_sequence, "Full UDS Sequence")

    # ── IDS callbacks ─────────────────────────────────────────────────────────
    def _do_ids_safe(self):
        self._run(self._ids.send_safe_frame, "IDS Safe Frame")

    def _do_ids_r1(self):
        self._run(self._ids.trigger_r1_unknown_id, "IDS R1 inject")

    def _do_ids_r2(self):
        self._run(self._ids.trigger_r2_wrong_dlc, "IDS R2 inject")

    def _do_ids_r3(self):
        self._run(self._ids.trigger_r3_rapid_inject, "IDS R3 inject")

    def _do_ids_r4(self):
        self._run(self._ids.trigger_r4_burst_flood, "IDS R4 flood")

    def _do_full_ids_seq(self):
        self._run(self._ids.run_full_ids_sequence, "Full IDS Sequence")

    # ── Custom Frame callback ─────────────────────────────────────────────────
    def _do_custom_send(self):
        # Parse inputs before spawning thread
        try:
            cid      = int(self._custom_id_var.get().strip(), 16)
            dlc      = int(self._custom_dlc_var.get().strip())
            data_str = self._custom_data_var.get().strip()
            data     = bytes(int(x, 16) for x in data_str.split()) if data_str else b''
            repeat   = max(1, int(self._custom_repeat_var.get().strip()))
            interval = max(0, float(self._custom_interval_var.get().strip())) / 1000.0
        except Exception as ex:
            messagebox.showerror("Input Error", f"Invalid frame parameters:\n{ex}")
            return

        if cid > 0x7FF:
            messagebox.showerror("Input Error", "CAN ID must be ≤ 0x7FF (11-bit standard)")
            return
        if dlc > 8:
            messagebox.showerror("Input Error", "DLC must be 0–8")
            return

        def _send_task():
            self._log(f"[CUS]  Sending  ID=0x{cid:03X}  DLC={dlc}  "
                      f"data=[{' '.join(f'{b:02X}' for b in data[:dlc])}]  "
                      f"×{repeat}  interval={int(interval*1000)}ms")
            ok_count = 0
            for i in range(repeat):
                if self._can_bus.send(cid, data, dlc=dlc):
                    ok_count += 1
                if repeat > 1 and i < repeat - 1:
                    time.sleep(interval)
            self._log(f"[CUS]  ✓ Sent {ok_count}/{repeat} frame(s)  ID=0x{cid:03X}")

        self._run(_send_task, f"Custom 0x{cid:03X} ×{repeat}")

    # ── Window close ─────────────────────────────────────────────────────────
    def on_close(self):
        self._disconnect()
        self.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
# Each scenario is a list of steps executed sequentially by ScenarioRunner.
# Step dict keys:
#   label      str   – shown in log
#   cmd        int   – HART command number
#   data       bytes – command data payload
#   use_long   bool  – use long (unique-ID) address frame
#   preambles  int|None – override preamble count for this step
#   master     str|None – "primary" | "secondary"
#   delay_pre  float – sleep BEFORE sending (seconds)
#   delay_post float – sleep AFTER response received (seconds)
#   note       str   – diagnostic hint shown in log before step

SCENARIOS = {
  "1 - Cold-Start Enumeration (Rockwell style)": {
    "description": (
      "Mimics Rockwell 5094-IF4IHSXT startup sequence:\n"
      "• 20 preambles (Rockwell default)\n"
      "• Short-addr Cmd0 to discover unique ID\n"
      "• Switch to long-addr Cmd0 to confirm\n"
      "• Cmd1/Cmd2/Cmd3 with long addr\n"
      "Diagnoses: slave ignores long addr, wrong preamble count,\n"
      "           no response to addr=0 short frame."
    ),
    "steps": [
      {"label": "Cmd0 short addr (20 preambles)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 20, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Rockwell sends 20 preambles on init. If slave needs ≥5 this is fine."},
      {"label": "Cmd0 long addr (20 preambles)", "cmd": 0, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Long addr uses unique ID from previous step. Slave must accept both."},
      {"label": "Cmd1 PV (long addr)", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.2,
       "note": "Normal PV read with long addr."},
      {"label": "Cmd2 Current+% (long addr)", "cmd": 2, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.2,
       "note": "Loop current + percent of range."},
      {"label": "Cmd3 Dynamic vars (long addr)", "cmd": 3, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.2,
       "note": "All dynamic variables."},
      {"label": "Cmd48 Ext Status (long addr)", "cmd": 48, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.2,
       "note": "Extended device status - PLCs poll this on startup."},
    ]
  },

  "2 - Preamble Count Sweep (3/5/8/16/20)": {
    "description": (
      "Sends Cmd0 with increasing preamble counts.\n"
      "Slave must respond to ≥5 preambles (HART spec minimum).\n"
      "3 preambles: slave MAY ignore (below spec).\n"
      "Rockwell uses 20, Emerson Trex uses 5.\n"
      "Diagnoses: slave preamble detector threshold."
    ),
    "steps": [
      {"label": "Cmd0 - 3 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 3, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "3 preambles - BELOW spec minimum. Slave may legitimately ignore."},
      {"label": "Cmd0 - 5 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "5 preambles - HART spec minimum. Slave MUST respond."},
      {"label": "Cmd0 - 8 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 8, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "8 preambles - common default."},
      {"label": "Cmd0 - 16 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 16, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "16 preambles - conservative industrial default."},
      {"label": "Cmd0 - 20 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 20, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "20 preambles - Rockwell 5094 default. Slave must handle."},
    ]
  },

  "3 - Secondary Master Probe (Emerson Trex)": {
    "description": (
      "Emerson Trex communicates as secondary master (delimiter 0x01/0x81).\n"
      "Sends Cmd0 first as primary then as secondary.\n"
      "Slave must respond to both.\n"
      "Diagnoses: slave rejects secondary master delimiter,\n"
      "           or hardcodes primary-only check."
    ),
    "steps": [
      {"label": "Cmd0 - primary master", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Baseline: primary master delimiter 0x02."},
      {"label": "Cmd0 - secondary master", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Secondary master delimiter 0x01. Slave must also respond."},
      {"label": "Cmd1 PV - secondary master", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Normal read via secondary master."},
      {"label": "Cmd3 Dynamic - secondary master", "cmd": 3, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "All vars via secondary master."},
    ]
  },

  "4 - Multi-Drop Address Sweep (addr 0–15)": {
    "description": (
      "Polls Cmd0 at every short address 0–15.\n"
      "Standard multi-drop enumeration - PLCs do this on bus startup.\n"
      "Diagnoses: slave responds on wrong address,\n"
      "           does not respond on its configured address,\n"
      "           or responds to broadcast address 0 only."
    ),
    "steps": [
      {"label": f"Cmd0 - addr {addr}", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "_addr": addr,
       "delay_pre": 0.2, "delay_post": 0.4,
       "note": f"Poll address {addr}. Only the device at this address should respond."}
      for addr in range(16)
    ]
  },

  "5 - Short/Long Address Interleave": {
    "description": (
      "Alternates between short-addr and long-addr frames.\n"
      "Some slaves mis-handle address mode switches mid-session.\n"
      "Diagnoses: slave confuses address mode, loses sync,\n"
      "           or fails to echo correct delimiter in response."
    ),
    "steps": [
      {"label": "Cmd0 short addr", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "Short address frame. Response delimiter should be 0x06."},
      {"label": "Cmd0 long addr", "cmd": 0, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "Long address frame. Response delimiter should be 0x86."},
      {"label": "Cmd1 short addr", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Back to short addr."},
      {"label": "Cmd1 long addr", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Long addr again."},
      {"label": "Cmd3 short addr", "cmd": 3, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Short addr, all variables."},
      {"label": "Cmd3 long addr", "cmd": 3, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Long addr, all variables."},
    ]
  },

  "6 - Rapid-Fire Stress (gap/timing)": {
    "description": (
      "Sends Cmd1 repeatedly with short inter-frame gaps.\n"
      "Tests slave's ability to handle back-to-back requests.\n"
      "1200 baud = ~8.33ms/byte. HART spec requires ≥2 char times gap.\n"
      "Diagnoses: slave buffer overrun, missed frames,\n"
      "           wrong 'overrun error' bit in status byte."
    ),
    "steps": [
      {"label": f"Cmd1 rapid #{i+1}", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.05, "delay_post": 0.0,
       "note": "Minimal inter-frame gap (50ms). Tests slave receiver reset speed."}
      for i in range(10)
    ]
  },

  "7 - Bad Checksum Injection × 3 then Valid": {
    "description": (
      "Sends 3 frames with corrupted checksums, then a valid Cmd0.\n"
      "Slave must:\n"
      "  • Silently discard or NAK bad-CheckSum frames\n"
      "  • Recover and respond to the valid frame\n"
      "Diagnoses: slave locks up after bad frame, never recovers,\n"
      "           or responds to bad-CheckSum frame (wrong behavior)."
    ),
    "steps": [
      {"label": "BAD CheckSum frame #1", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Checksum XOR'd with 0xFF - slave must discard.",
       "_corrupt_cs": True, "expect_no_response": True},
      {"label": "BAD CheckSum frame #2", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Second bad-CS frame.",
       "_corrupt_cs": True, "expect_no_response": True},
      {"label": "BAD CheckSum frame #3", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Third bad-CS frame.",
       "_corrupt_cs": True, "expect_no_response": True},
      {"label": "VALID Cmd0 - recovery check", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.8, "delay_post": 0.5,
       "note": "Valid frame after 3 bad ones. Slave MUST respond - tests recovery."},
    ]
  },

  "8 - HART Revision Probe (Cmd0 → check revision byte)": {
    "description": (
      "Sends Cmd0 and inspects HART revision field.\n"
      "HART 5 = 0x05, HART 6 = 0x06, HART 7 = 0x07.\n"
      "Rockwell 5094 requires HART 6 or 7.\n"
      "Emerson Trex supports HART 5/6/7.\n"
      "Diagnoses: slave reports HART 5 to a HART 7 master\n"
      "           causing master to abort session."
    ),
    "steps": [
      {"label": "Cmd0 - check HART revision", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Check byte[3] of response payload: 5=HARTv5, 6=HARTv6, 7=HARTv7."},
      {"label": "Cmd0 long - cross-check", "cmd": 0, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Long addr repeat - confirm same revision reported."},
      {"label": "Cmd48 - extended status", "cmd": 48, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.4,
       "note": "HART 7 devices must implement Cmd48. HART 5 may return cmd-not-implemented (64)."},
    ]
  },

  "9 - Full PLC Session (Rockwell + Emerson combined)": {
    "description": (
      "Full realistic session combining both device patterns:\n"
      "Phase 1 (Rockwell): 20 preambles, primary master, long addr\n"
      "Phase 2 (Emerson):  5 preambles, secondary master, short+long addr\n"
      "Phase 3: rapid polling as primary\n"
      "Comprehensive end-to-end compatibility check."
    ),
    "steps": [
      # Phase 1: Rockwell
      {"label": "[Rockwell] Cmd0 short (20 pre)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 20, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Rockwell phase: discover device."},
      {"label": "[Rockwell] Cmd0 long (20 pre)", "cmd": 0, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.4,
       "note": "Rockwell phase: confirm long addr."},
      {"label": "[Rockwell] Cmd1 PV", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Rockwell phase: read PV."},
      {"label": "[Rockwell] Cmd2 Current", "cmd": 2, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Rockwell phase: read loop current."},
      {"label": "[Rockwell] Cmd48 Ext Status", "cmd": 48, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.5,
       "note": "Rockwell phase: extended status."},
      # Phase 2: Emerson Trex
      {"label": "[Emerson] Cmd0 short (5 pre, secondary)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.6, "delay_post": 0.4,
       "note": "Emerson Trex phase: secondary master, 5 preambles."},
      {"label": "[Emerson] Cmd13 Tag/Descriptor", "cmd": 13, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Emerson reads tag/descriptor."},
      {"label": "[Emerson] Cmd14 PV Info", "cmd": 14, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Emerson reads PV transducer info."},
      # Phase 3: rapid primary poll
      {"label": "[Poll] Cmd1 #1", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.2, "delay_post": 0.1,
       "note": "Rapid poll phase."},
      {"label": "[Poll] Cmd1 #2", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.2, "delay_post": 0.1,
       "note": "Rapid poll phase."},
      {"label": "[Poll] Cmd1 #3", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.2, "delay_post": 0.1,
       "note": "Rapid poll phase."},
    ]
  },

  "10 - Identity Read (Cmd12/13/14)": {
    "description": (
      "Reads device identity information in the order PLCs use during commissioning:\n"
      "• Cmd0  – identify device, learn unique ID\n"
      "• Cmd12 – read message string (32-char packed ASCII)\n"
      "• Cmd13 – read tag, descriptor, date\n"
      "• Cmd14 – read PV transducer info (units, upper/lower limits, min span)\n"
      "Diagnoses: slave omits optional identity commands, returns wrong packed-ASCII,\n"
      "           or rejects Cmd14 on non-pressure devices."
    ),
    "steps": [
      {"label": "Cmd0 - identify device", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Baseline identification. Unique ID learned for long-addr steps."},
      {"label": "Cmd12 - read message", "cmd": 12, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Read 32-char packed-ASCII message field. Must return 24 bytes payload."},
      {"label": "Cmd13 - tag/descriptor/date", "cmd": 13, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Read tag (8 char), descriptor (16 char), date (3 bytes). Must return 21 bytes."},
      {"label": "Cmd14 - PV transducer info", "cmd": 14, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "PV units, upper/lower sensor limits, min span. Must return ≥14 bytes."},
      {"label": "Cmd14 - long addr repeat", "cmd": 14, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Same read via long addr – confirm consistent response."},
    ]
  },

  "11 - Write-Protect Probe (Cmd17/18/19)": {
    "description": (
      "Attempts configuration writes to verify write-protect behavior:\n"
      "• Cmd17 – write message (should return 0x07 if write-protected)\n"
      "• Cmd18 – write tag/descriptor/date\n"
      "• Cmd19 – write final assembly number\n"
      "In write-protect mode, slave MUST return response code 0x07.\n"
      "Diagnoses: slave silently ignores writes, corrupts config,\n"
      "           or fails to set write-protect bit in status byte."
    ),
    "steps": [
      {"label": "Cmd0 - baseline status", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Read status before write attempts. Note 'Analog output fixed' bit."},
      {"label": "Cmd17 - write message (24 bytes)", "cmd": 17,
       "data": bytes([0x20] * 24),  # packed-ASCII spaces
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.4,
       "note": "Write blank message. If write-protected: expect RC=0x07. If not: message overwritten."},
      {"label": "Cmd18 - write tag/descriptor/date", "cmd": 18,
       "data": bytes([0x20] * 21),  # spaces tag + descriptor + date
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.4,
       "note": "Write blank tag and descriptor. Expect RC=0x07 if write-protected."},
      {"label": "Cmd19 - write final assembly number", "cmd": 19,
       "data": bytes([0x00, 0x00, 0x00]),
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.4,
       "note": "Write assembly number 0. Expect RC=0x07 if write-protected."},
      {"label": "Cmd0 - verify no config change", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "'Configuration changed' bit should NOT be set if write-protect worked."},
    ]
  },

  "12 - Configuration Changed Recovery": {
    "description": (
      "Simulates the PLC cold-start / config-changed handling sequence:\n"
      "• Cmd0 – check for 'Cold start' (0x20) and 'Config changed' (0x40) bits\n"
      "• Cmd48 – read extended status to capture full snapshot\n"
      "• Cmd0 × 3 – re-poll until status bits clear (PLCs retry up to N times)\n"
      "Diagnoses: slave never clears Cold start bit, Config changed sticky after Cmd0,\n"
      "           or 'More status available' bit loops indefinitely."
    ),
    "steps": [
      {"label": "Cmd0 - initial (check cold-start/config bits)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "ST2 byte: 0x20=ColdStart, 0x40=ConfigChanged, 0x10=MoreStatusAvail."},
      {"label": "Cmd48 - extended status snapshot", "cmd": 48, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Capture full extended status. Must send after 'More status available' bit set."},
      {"label": "Cmd0 - retry #1 (expect bits clearing)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Cold-start and config-changed bits should clear after first valid Cmd0 exchange."},
      {"label": "Cmd0 - retry #2", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Second retry - bits must be clear by now on a conformant device."},
      {"label": "Cmd1 - PV after recovery", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Confirm normal operation restored after status-bit recovery sequence."},
      {"label": "Cmd3 - all vars after recovery", "cmd": 3, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Confirm all dynamic variables readable post-recovery."},
    ]
  },

  "13 - Device Variable Commands (Cmd9/33)": {
    "description": (
      "HART 7 device variable commands:\n"
      "• Cmd9  – read device variables with status (up to 4 variable codes)\n"
      "• Cmd33 – read device variables (HART 5/6 compatible subset)\n"
      "• Cmd3  – fallback all-vars read for comparison\n"
      "Diagnoses: HART 7 slave does not implement Cmd9 (returns RC=64),\n"
      "           variable status byte always 0x00 (no status support),\n"
      "           or returns wrong variable count."
    ),
    "steps": [
      {"label": "Cmd0 - check HART revision", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Byte[3] of payload: 7 = HART7 required for Cmd9. 5/6 = expect RC=64 on Cmd9."},
      # Cmd9: request 4 device variable codes (0=PV,1=SV,2=TV,3=QV)
      {"label": "Cmd9 - device vars with status (PV/SV/TV/QV)", "cmd": 9,
       "data": bytes([0x00, 0x01, 0x02, 0x03]),
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "HART7 Cmd9: request variable codes 0,1,2,3. Each var returns 8 bytes + status."},
      {"label": "Cmd9 - just PV (code 0)", "cmd": 9,
       "data": bytes([0x00, 0xFF, 0xFF, 0xFF]),
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "Cmd9 with only PV requested (0xFF = not used). Minimal variable read."},
      {"label": "Cmd33 - read device variables", "cmd": 33,
       "data": bytes([0x00]),  # slot 0
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "Cmd33: HART 5/6/7 compatible device variable read."},
      {"label": "Cmd3 - all dynamic vars (baseline)", "cmd": 3, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Cmd3 as baseline - compare values against Cmd9/33 results."},
    ]
  },

  "14 - Broadcast / Global Address (addr 63)": {
    "description": (
      "Tests slave behavior on broadcast address (short addr 63, 0x3F):\n"
      "• HART spec: address 63 is global broadcast - slaves must NOT respond\n"
      "• Slave responding to addr 63 causes bus collisions in multi-drop\n"
      "• Followed by valid addressed Cmd0 to confirm slave is still alive\n"
      "Diagnoses: slave incorrectly responds to broadcast,\n"
      "           slave locks up after seeing broadcast address."
    ),
    "steps": [
      {"label": "Cmd0 - normal addr (baseline)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Baseline: normal addressed Cmd0. Slave must respond."},
      {"label": "Cmd0 - broadcast addr 63 (expect NO response)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "_addr": 63,
       "expect_no_response": True,
       "delay_pre": 0.5, "delay_post": 0.6,
       "note": "Broadcast addr 0x3F. Conformant slave MUST NOT respond. Timeout = pass."},
      {"label": "Cmd0 - normal addr (recovery check)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Slave must still respond normally after seeing broadcast."},
      {"label": "Cmd1 - PV after broadcast", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Confirm normal data flow after broadcast test."},
    ]
  },

  "15 - Power-On Cold Start Validation": {
    "description": (
      "Validates if slave handles cold-start flags:\n"
      "• Sends baseline short-addr Cmd0\n"
      "• Checks if 'Configuration changed' / 'Cold start' bit is set\n"
      "• Sends second Cmd0 to see if bits clear properly."
    ),
    "steps": [
      {"label": "Initial Cmd0 Discovery", "cmd": 0, "data": b"", "use_long": False, "preambles": 5, "master": "primary", "delay_pre": 0.5, "delay_post": 0.3, "note": "Check device flags byte (st2) for Cold Start bit."},
      {"label": "Verify Flag Clearing Cmd0", "cmd": 0, "data": b"", "use_long": False, "preambles": 5, "master": "primary", "delay_pre": 0.2, "delay_post": 0.2, "note": "Flag must clear on second transmission according to spec."}
    ]
  },

  "16 - Invalid Command Exception Handling": {
    "description": (
      "Injects unimplemented command code to monitor slave's response logic:\n"
      "• Sends invalid Cmd 99\n"
      "• Expects response code 64 ('Command not implemented')\n"
      "• Verifies slave doesn't freeze up under invalid inputs."
    ),
    "steps": [
      {"label": "Send Unimplemented Cmd 99", "cmd": 99, "data": b"0000", "use_long": False, "preambles": 5, "master": "primary", "delay_pre": 0.3, "delay_post": 0.3, "note": "Slave must respond with error code 64, not time out."}
    ]
  },
}

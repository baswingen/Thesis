# Porti7 TRIG Input – Wiring and Troubleshooting

This note summarizes how the TMSi Porti7 TRIG port works, how it appears in the TMSi Python interface, and what to check when **no trigger signal is recognised** (flat TRIG at 0).

---

## 1. Interface findings (TMSi Python + TMSi docs)

### TMSi trigger inputs in general
- **TMSi** (Artinis) docs describe trigger inputs for **SAGA**, **APEX**, and **SPIRE** (not Porti7 by name).
- Trigger inputs are **digital**: they receive **event trigger cables** or digital sensors (e.g. “DIGI” on SAGA).
- The **USB TTL module** (BBTK) is used to *send* triggers *from* a PC *to* SAGA/APEX via a trigger cable. So **TTL is used for trigger signalling** in the TMSi ecosystem.
- SAGA’s “DIGI” is described as an input for an “event trigger cable or digital serial sensors”.
- **Allowed input voltages** are not given in the public sync page; TMSi refers to the **user manual and Knowledge Base** for SAGA/APEX. Porti7 is a **legacy** device; its TRIG specs are not in the Python repo.

### TMSi Python interface
- **SAGA**: `set_device_triggers(True)` must be called to enable external triggers. There is a **REVERSE mask** for the trigger channel (inverted logic in software).
- **Porti7 (legacy)**: In `tmsi-python-interface`, the **Legacy** device (Porti7, REFA) has **no** `set_device_triggers()`; the TRIG input is always part of the signal format. The TRIG appears as a channel named **STATUS** or **Dig** (index 36 on a 38‑channel Porti7). Channel names come from the SDK; `fix_channel_name()` in this project maps index 36 → `"STATUS"`.
- So: **TTL is used for triggers**; the open question for Porti7 is **voltage level and input circuit** (e.g. 3.3 V vs 5 V, and whether the input is high‑Z or 50 Ω).

---

## 2. Recommended wiring (from this project)

- **Porti7 TRIG connector**: LEMO FFA.00 (coax).
  - **Centre pin**: trigger signal (from STM32).
  - **Shield**: ground (GND).
- **STM32 → Porti7 TRIG**:
  - **STM32F401 PA8** → series resistor → **Porti7 TRIG (LEMO centre)**.
  - **STM32F401 GND** → **Porti7 TRIG (LEMO shield)**.
- **Series resistor**: In this repo, **330 Ω** is used; TMSi-style recommendation is “330 Ω..1 kΩ” for the centre pin. The resistor limits current and protects both sides.

So the wiring you have (PA8 → 330 Ω → LEMO centre, GND → LEMO shield) is **correct in principle**. If the TRIG channel stays flat at 0, the problem is likely **signal level, cable/connector, or wrong connector**, not “TTL not usable” (TTL is the intended type of trigger).

---

## 3. Why “no trig signal recognised” (flat at 0)

Possible causes:

1. **Voltage level**
   - STM32 GPIO is **3.3 V** (HIGH). Some older TMSi inputs may expect **5 V TTL** (e.g. ViH min 2.4 V or 3.0 V). 3.3 V is above classic TTL “high” (2.0 V) but below some 5 V CMOS thresholds.
   - **Try**: Use a **3.3 V → 5 V level shifter** (e.g. 74LVC245 or similar) so the Porti7 TRIG sees a clear 5 V HIGH. Keep GND common.

2. **Series resistor**
   - If the Porti7 TRIG input has a low input impedance (e.g. 50 Ω), 330 Ω could form a divider and reduce the voltage at the Porti7 pin.
   - **Try**: Temporarily use a **smaller resistor** (e.g. 100 Ω) or **no resistor** only for a quick test (then restore 330 Ω or 1 kΩ for safety).

3. **Wrong connector / cable**
   - Confirm the LEMO cable is plugged into the **TRIG** (digital trigger) port on the Porti7, not e.g. SYNC OUT or another port. Check the Porti7 front panel labelling.

4. **Cable or contact**
   - Verify continuity: LEMO centre ↔ PA8 (through resistor), LEMO shield ↔ GND. Check that the LEMO plug is fully seated.

5. **Software channel**
   - This project finds the TRIG channel by name: first `"TRIG"`, then `"DIG"`, then `"STATUS"`, or index 36. If the device reports a different name, the code may still pick it up via STATUS/index. Flat 0 with correct channel selection usually means the **hardware** is not seeing a voltage swing.

---

## 4. What to do next

1. **Hardware check**
   - With STM32 running and PRBS toggling PA8: measure **voltage at the LEMO plug** (centre vs shield): you should see it switch between ~0 V and ~3.3 V (or 5 V if using a level shifter).
   - If possible, check the **Porti7 user manual** or **TMSi support** for: “TRIG input”, “digital input”, “allowed voltage”, “input impedance”. Porti7 may be documented under “REFA” or “legacy” in TMSi’s Knowledge Base.

2. **Try 5 V trigger**
   - If you have a 3.3 V → 5 V level shifter: PA8 → level shifter → LEMO centre (and common GND). Then re-run your PRBS sync test.

3. **Try lower series R (temporary test only)**
   - Replace 330 Ω with 100 Ω or short the resistor once, then run the same test. If TRIG starts toggling, the level was marginal; then use 330 Ω or 1 kΩ again and consider the level shifter for a permanent fix.

4. **Confirm TRIG channel in software**
   - In your diagnostics, confirm which channel index and name are used for TRIG (e.g. index 36, “STATUS”). If the TRIG channel is correct and still flat, the issue is on the hardware side.

---

## 5. Short summary

- **TTL is the right type of signal** for TMSi trigger inputs; the USB TTL module and SAGA/APEX docs confirm that.
- Porti7 TRIG is a **digital trigger input** on the LEMO TRIG port; it appears as STATUS/Dig in the legacy Python interface.
- Your wiring (PA8 → 330 Ω → LEMO centre, GND → LEMO shield) is **correct**. If the Porti7 still shows no trigger, the most likely causes are **3.3 V vs 5 V level**, **input impedance + resistor**, or **connector/cable**. Trying a **5 V level shifter** and/or a **smaller series resistor** (for testing) is the next practical step; for definitive specs, refer to the **Porti7/REFA user manual** or **TMSi support**.

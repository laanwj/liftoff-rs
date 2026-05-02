# Flight Controller ↔ Autopilot Interface Investigation

## Summary

There is **no single universal standard** for flight controller ↔ autopilot communication. The ecosystem is fragmented across different layers:

- **RC → Flight Controller**: CRSF/ELRS dominates
- **GCS → Autopilot**: MAVLink dominates
- **Autopilot → Motors**: Various (DShot, PWM, STMLink)
- **Autopilot ↔ FC**: Often proprietary or custom

**CRSF is a solid choice** for the "autopilot from ground" simulation use case - it's simple, low-latency, and matches the real-world RC→FC path.

---

## Protocols Investigated

### 1. CRSF (Crossfire Serial Protocol)

**What it is:** Binary frame-based protocol for RC devices, originally by TBS for Crossfire radio system, now widely adopted by ELRS.

**Characteristics:**
- Low latency, bidirectional UART (~400kbps)
- 64-byte max frame size with CRC8
- Device addressing and routing
- Rich telemetry frame types (GPS, battery, attitude, RPM, etc.)
- Open specification maintained by TBS

**Use case:** RC receiver ↔ flight controller communication

**Pros:**
- ✅ Simple, well-documented
- ✅ Low overhead, deterministic timing
- ✅ Mature ecosystem (ELRS, Betaflight, INAV support)
- ✅ Matches real RC→FC path

**Cons:**
- ❌ Designed for RC, not general autopilot communication
- ❌ Limited to 64-byte frames

**Verdict:** **Best fit for this project** - simulates real RC→FC communication path.

---

### 2. MAVLink

**What it is:** Standard protocol for GCS ↔ autopilot communication, maintained by Dronecode Foundation.

**Characteristics:**
- Variable-length messages (up to 281 bytes for MAVLink2)
- Rich message set for navigation, telemetry, commands
- System/component addressing
- Multiple transports (UART, UDP, TCP, etc.)

**Use case:** Ground control station ↔ autopilot computer

**Pros:**
- ✅ De facto standard for GCS communication
- ✅ Extensive message set for high-level commands
- ✅ Well-supported by ArduPilot, PX4, etc.

**Cons:**
- ❌ High overhead for low-level control
- ❌ Complex protocol with significant latency
- ❌ Not designed for real-time control loops

**Verdict:** **Not suitable** for FC↔autopilot control, but useful for GCS integration (already used in this project).

---

### 3. STMLink

**What it is:** Simple binary protocol used by ArduPilot for autopilot ↔ ESC/motor driver communication.

**Characteristics:**
- Lightweight UART protocol
- Designed for motor telemetry and control
- ArduPilot-specific

**Use case:** Autopilot ↔ motor drivers

**Pros:**
- ✅ Very lightweight
- ✅ Bidirectional telemetry

**Cons:**
- ❌ ArduPilot ecosystem only
- ❌ Not widely adopted
- ❌ Limited documentation

**Verdict:** **Niche use** - only relevant if targeting ArduPilot with custom ESCs.

---

### 4. DShot

**What it is:** Digital protocol for flight controller → ESC communication, replacing analog PWM.

**Characteristics:**
- One-way digital PWM replacement (with optional telemetry return)
- Very fast update rates (4-32kHz vs ~500Hz for PWM)
- 16-32 bit frames with CRC
- Pulse-width encoded on single wire per motor

**Use case:** Flight controller → motor speed controllers

**Pros:**
- ✅ Extremely fast motor response
- ✅ Bidirectional telemetry (RPM, temperature, etc.)
- ✅ Widely adopted in racing FPV drones

**Cons:**
- ❌ Motor control only, not general autopilot communication
- ❌ One wire per motor (no shared bus)

**Verdict:** **Different layer** - for motor control, not autopilot↔FC communication.

---

### 5. Cyphal (formerly UAVCAN)

**What it is:** Open standard for real-time intravehicular distributed computing and communication.

**Characteristics:**
- Pub/sub + RPC (request/response)
- Multiple transports (CAN, CAN FD, Ethernet/UDP, Serial)
- DSDL (Data Structure Description Language) for type definitions
- Stateless, peer-to-peer architecture
- Designed for safety-critical systems (DO-178C, IEC 61508)

**Use case:** Full vehicle internal network (sensors, actuators, computers)

**Pros:**
- ✅ Professional-grade, robust protocol
- ✅ Strong typing with semantic versioning
- ✅ Deterministic, real-time capable
- ✅ Adopted by ArduPilot, commercial UAV systems, spacecraft

**Cons:**
- ❌ High complexity (formal spec, DSDL compilation)
- ❌ Overkill for hobby/simulation use
- ❌ Limited language bindings compared to general-purpose systems
- ❌ Requires CAN hardware for full benefits

**Verdict:** **Overkill for this project** - better suited for production embedded systems with functional safety requirements.

---

## Comparison Table

| Protocol | Layer | Direction | Complexity | Best For |
|----------|-------|-----------|------------|----------|
| **CRSF** | RC→FC | Bidirectional | Low | Radio control, simulation |
| **MAVLink** | GCS↔AP | Bidirectional | Medium | Ground station commands |
| **DShot** | FC→ESC | Unidirectional (+telem) | Low | Motor control |
| **STMLink** | AP→ESC | Bidirectional | Low | ArduPilot motor telemetry |
| **Cyphal** | Vehicle bus | Pub/Sub + RPC | High | Production avionics |

---

## Architecture Context

In this project, **Zenoh** handles inter-process communication between services (sim, autopilot, dashboard, GPSD). This is similar to what Cyphal does, but:

| | Zenoh | Cyphal |
|---|---|---|
| **Transport** | TCP, UDP, WebSocket | CAN, CAN FD, UDP, Serial |
| **Target** | IoT, edge, microservices | Avionics, safety-critical embedded |
| **Complexity** | Medium | High (formal spec, DSDL) |
| **Safety cert** | No | Designed for DO-178C/IEC 61508 |
| **Language bindings** | Rust, C, Python, Java, etc. | C, C++, Python (limited) |

**Why Zenoh makes sense:**
- ✅ Easier to develop with (better tooling, documentation)
- ✅ Works over network transparently (UDP multicast discovery)
- ✅ No need for CAN hardware or DSDL compilation
- ✅ Simulation + real hardware use same protocol
- ✅ Already integrated and working

**When Cyphal would make sense:**
- Production embedded system with CAN bus
- Need for functional safety certification
- Hard real-time determinism requirements

---

## Conclusion

**Stick with CRSF + Zenoh.** This combination provides:

1. **CRSF** for realistic RC→FC simulation with low latency
2. **Zenoh** for flexible service-to-service communication
3. **MAVLink** (already integrated) for GCS compatibility

This architecture is pragmatic, well-supported, and matches the project's goals. Cyphal would only be justified for a production autopilot targeting certified aircraft with CAN bus hardware.

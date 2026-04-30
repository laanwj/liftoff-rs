-- Uncrashed FPV Drone Sim → file IPC telemetry bridge.
--
-- UE4SS Lua mod. On every game tick, locates the player's drone pawn
-- (BP_FPVDrone_Pawn_C), reads its state via UE4SS's UObject reflection,
-- packs a `UCFV` packet, and overwrites the IPC file at PATH with the
-- single latest packet (no append, no rotation — only the current
-- snapshot ever matters). The Rust-side receiver (`uncrashed-input`)
-- polls the same path and republishes on Zenoh.
--
-- Wire format: see ../README.md. Single source of truth across mod and
-- receiver; do NOT change one side without the other.
--
-- Why file IPC and not UDP: avoids bundling LuaSocket. UE4SS bundles
-- only Lua 5.4's stdlib, which has io.* but no networking primitives.
-- Wine maps the Linux root filesystem to drive Z:, so a Win-side
-- io.open("Z:\\tmp\\…") writes to the host's /tmp/…, which the receiver
-- on Linux reads natively.

local PATH = "Z:\\tmp\\uncrashed-telemetry.bin"

-- ─── configuration ────────────────────────────────────────────────────────
local CONFIG = {
    -- Class names confirmed via Phase 1 pak recon (see uncrashed.md).
    drone_class = "BP_FPVDrone_Pawn_C",
    controller_class = "BP_MyController_C",
    -- Log every Nth packet for sanity-checking; 0 disables.
    log_every = 600,
    -- LoopAsync interval, ms. 16 ms ≈ 60 Hz; close to UE4 frame rate.
    tick_ms = 16,
}

-- ─── UCFV wire format constants ───────────────────────────────────────────
local UCFV_TAG     = "UCFV"
local UCFV_VERSION = 2
local FLAG_ON_GROUND   = 0x0001
local FLAG_ARMED       = 0x0002
local FLAG_HAS_BATTERY = 0x0004
local FLAG_HAS_DAMAGE  = 0x0008
local FLAG_CRASHED     = 0x0010
local UCFV_MAX_PROPS = 8
local UCFV_HEADER_SIZE = 108
-- Every emitted packet is padded to UCFV_PACKET_SIZE bytes so the file
-- size is constant and the receiver only reads a known length.
local UCFV_PACKET_SIZE = UCFV_HEADER_SIZE + 4 * UCFV_MAX_PROPS

-- ─── runtime state ────────────────────────────────────────────────────────
local epoch_ms = 0          -- monotonic baseline (os.clock-based)
local seq = 0
local cached_pawn = nil
local cached_pc = nil
local out_file = nil        -- long-lived write handle, see init()

-- ─── helpers ──────────────────────────────────────────────────────────────
local function log(fmt, ...)
    -- UE4SS's print() does NOT append a newline; do it ourselves so the
    -- log stays readable.
    print("[uncrashed-telemetry] " .. string.format(fmt, ...) .. "\n")
end

-- Monotonic μs since mod load. os.clock returns process CPU time in
-- seconds (Lua 5.4 stdlib). Good enough for sequencing — for actual
-- wall-clock timestamps the receiver layers them on at recv time.
local function now_us()
    return math.floor(os.clock() * 1e6) - epoch_ms
end

-- UE4SS's IsValid is the safe way to check a UObject hasn't been GC'd.
local function valid(obj)
    return obj and obj:IsValid()
end

-- Resolve drone pawn lazily; re-resolve whenever the cache goes stale (map
-- reload, drone respawn, replay enter/exit). FindFirstOf scans the live
-- UObject pool, which is comparatively expensive — keep this off the hot path.
local function find_drone()
    if valid(cached_pawn) then return cached_pawn end
    cached_pawn = FindFirstOf(CONFIG.drone_class)
    if valid(cached_pawn) then
        log("acquired drone: %s", cached_pawn:GetFullName())
    end
    return cached_pawn
end

local function find_controller()
    if valid(cached_pc) then return cached_pc end
    cached_pc = FindFirstOf(CONFIG.controller_class)
    return cached_pc
end

-- Read a property and return a Lua number, falling back to 0 on miss.
-- pcall'd so a renamed-by-patch field can't tear down the entire mod.
local function read_float(obj, name)
    local ok, val = pcall(function() return obj[name] end)
    if ok and type(val) == "number" then return val end
    return 0.0
end
local function read_bool(obj, name)
    local ok, val = pcall(function() return obj[name] end)
    if ok and type(val) == "boolean" then return val end
    return false
end

-- Pack the UCFV header. Layout matches uncrashed-input/src/wire.rs exactly.
-- Lua 5.4 string.pack format string:
--   <  little-endian
--   c4 fixed 4-byte string
--   I2 / I4 / I8  unsigned int
--   i4 signed int32
--   f  f32
--   B  u8
local HEADER_FMT = "<c4I2I2I8I4 fff ffff fff fff ffff fff i4 B B BB"
-- Sanity: c4(4)+I2(2)+I2(2)+I8(8)+I4(4) = 20 header
--       + fff(12)              position
--       + ffff(16)             attitude
--       + fff(12)              velocity
--       + fff(12)              gyro
--       + ffff(16)             inputs
--       + fff(12) + i4(4)      battery raw [vpc, current_amps, used_mah] + capacity_mah
--       + B(1) + B(1) + BB(2)  cell_count + prop_count + pad
--       = 108 bytes total

local function build_packet(snap)
    local hdr = string.pack(HEADER_FMT,
        UCFV_TAG, UCFV_VERSION, snap.flags, snap.timestamp_us, snap.sequence,
        snap.position[1], snap.position[2], snap.position[3],
        snap.attitude_quat[1], snap.attitude_quat[2], snap.attitude_quat[3], snap.attitude_quat[4],
        snap.velocity[1], snap.velocity[2], snap.velocity[3],
        snap.gyro[1], snap.gyro[2], snap.gyro[3],
        snap.inputs[1], snap.inputs[2], snap.inputs[3], snap.inputs[4],
        snap.voltage_per_cell, snap.current_amps, snap.charge_used_mah,
        snap.capacity_mah,
        snap.cell_count, snap.prop_count, 0, 0)

    -- Emit UCFV_MAX_PROPS damage slots. Slots beyond snap.prop_count
    -- carry whatever's in snap.damage (typically 0) — the receiver only
    -- exposes [0, prop_count) and only when FLAG_HAS_DAMAGE is set, so
    -- out-of-range slots can't surface.
    local parts = {hdr}
    for i = 1, UCFV_MAX_PROPS do
        parts[#parts + 1] = string.pack("<f", snap.damage[i] or 0.0)
    end
    return table.concat(parts)
end

-- ─── per-tick sample ──────────────────────────────────────────────────────
local function sample()
    local snap = {
        flags = 0,
        timestamp_us = now_us(),
        sequence = seq,
        position = {0, 0, 0},
        attitude_quat = {0, 0, 0, 1},
        velocity = {0, 0, 0},
        gyro = {0, 0, 0},
        inputs = {0, 0, 0, 0},
        voltage_per_cell = 0.0,
        current_amps = 0.0,
        charge_used_mah = 0.0,
        capacity_mah = 0,
        cell_count = 0,
        prop_count = 0,
        damage = {},
    }
    seq = (seq + 1) % 0x100000000

    local pawn = find_drone()
    if not valid(pawn) then return snap end

    -- Position / rotation / velocity from the actor transform + physics.
    -- All wrapped in pcall so a single missing field doesn't kill the tick.
    local ok_loc, err_loc = pcall(function()
        local loc = pawn:K2_GetActorLocation()         -- FVector cm
        snap.position = {loc.X, loc.Y, loc.Z}
    end)
    if not ok_loc then log("location read failed: %s", tostring(err_loc)) end

    -- FRotator::Quaternion() is a plain C++ method, not a UFunction, so
    -- UE4SS Lua can't call it. Convert here with UE's own formula
    -- (from UnrealMath.cpp): degrees → half-angle → quaternion.
    local ok_rot, err_rot = pcall(function()
        local rot = pawn:K2_GetActorRotation()
        local hp = math.rad(rot.Pitch) * 0.5
        local hy = math.rad(rot.Yaw)   * 0.5
        local hr = math.rad(rot.Roll)  * 0.5
        local sp, cp = math.sin(hp), math.cos(hp)
        local sy, cy = math.sin(hy), math.cos(hy)
        local sr, cr = math.sin(hr), math.cos(hr)
        snap.attitude_quat = {
             cr * sp * sy - sr * cp * cy,
            -cr * sp * cy - sr * cp * sy,
             cr * cp * sy - sr * sp * cy,
             cr * cp * cy + sr * sp * sy,
        }
    end)
    if not ok_rot then log("rotation read failed: %s", tostring(err_rot)) end

    -- Physics body is the `drone` USkeletalMeshComponent (lowercase,
    -- per the CXX header dump). The capital-D `Drone` doesn't exist as
    -- a property and trying to access it crashes UE4SS at the C++ level.
    --
    -- Read linear velocity via the cached `ComponentVelocity` UPROPERTY
    -- on USceneComponent rather than via the UPrimitiveComponent
    -- UFunction `GetPhysicsLinearVelocity(FName)` — the latter takes a
    -- required (default) FName argument that UE4SS Lua doesn't reliably
    -- supply, so the call resolves to a function pointer and gets
    -- thrown back as an error.
    local ok_vel, err_vel = pcall(function()
        local body = pawn.drone
        if not valid(body) then return end
        local vel = body.ComponentVelocity
        if vel then snap.velocity = {vel.X, vel.Y, vel.Z} end
    end)
    if not ok_vel then log("velocity read failed: %s", tostring(err_vel)) end

    -- Angular velocity intentionally unread. UPrimitiveComponent's
    -- `GetPhysicsAngularVelocityInDegrees(FName)` UFunction takes a
    -- required FName argument that UE4SS Lua doesn't reliably supply,
    -- and there's no cached UPROPERTY equivalent. Gyro stays at zero
    -- in the UCFV packet.

    -- Stick inputs from the player controller (raw, pre-curve).
    local pc = find_controller()
    if valid(pc) then
        snap.inputs[1] = read_float(pc, "ThrottleAxis_Raw")
        snap.inputs[2] = read_float(pc, "YawAxis_Raw")
        snap.inputs[3] = read_float(pc, "PitchAxis_Raw")
        snap.inputs[4] = read_float(pc, "RollAxis_Raw")
    end

    -- Damage: PropsDamage TArray<float> on the pawn. Raw passthrough —
    -- the receiver handles units/ordering/inversion to match the
    -- dashboard's display convention.
    pcall(function()
        local dmg = pawn.PropsDamage
        if dmg and dmg.GetArrayNum then
            local n = dmg:GetArrayNum()
            if n > 0 and n <= UCFV_MAX_PROPS then
                snap.prop_count = n
                for i = 1, n do
                    -- LuaArrays in UE4SS are 0-indexed natively but the
                    -- :get() helper accepts 1-indexed Lua-style; double-check
                    -- after first run and adjust if every prop reads as the
                    -- same value.
                    snap.damage[i] = dmg[i] or 0.0
                end
                snap.flags = snap.flags | FLAG_HAS_DAMAGE
            end
        end
    end)

    -- Armed bit. The actual UPROPERTY name is "Armed?" (literal `?`) —
    -- visible in the CXX header dump. Lua bracket access in read_bool
    -- handles the special character fine.
    if read_bool(pawn, "Armed?") then
        snap.flags = snap.flags | FLAG_ARMED
    end

    -- FLAG_CRASHED is intentionally unset. The pawn's `damaged` boolean
    -- flips on any non-zero PropsDamage (including minor wear from a
    -- spawn-time touchdown), which is too noisy for a discrete crash
    -- status. Per-prop damage values still flow through the CRSF damage
    -- frame, so the dashboard's rotor colour-coding reflects actual
    -- wear; the discrete CRASHED bit will return when we identify a
    -- cleaner signal (e.g. HealthComponent.Health falling below a
    -- threshold, or the DestroyTarget event firing).

    -- Battery state lives on the player controller as an FS_BatteryState
    -- struct (see CXX dump of BP_MyController.hpp), gated by the
    -- SimulateBattery bool.
    --
    -- User-defined struct fields in UE4 carry a per-pin hash suffix that
    -- the BP compiler bakes in (`cellcount_26_3DF635…`). UE4SS Lua's
    -- bare-name lookup falls back to a generic getter-function when it
    -- can't find the clean prefix, which then explodes on use; using
    -- the full hashed names sidesteps the fallback. These hashes are
    -- copied verbatim from the CXX header dump of `S_BatteryState.hpp`
    -- and are stable for this game build.
    local F_CELLCOUNT  = "cellcount_26_3DF6358D40B3935835C031AB87D1C46C"
    local F_VPC        = "currentvoltagepercellv_28_CE5AFC654C729FFD7FE77BA12484916A"
    local F_CAPACITY   = "Capacitymah_29_24C81CFF4DA54C51AA0F50BC1F9CFD3D"
    local F_REMAINING  = "Currentmah_23_FAA75A3B448832DDCB372ABA4787AECE"
    -- Raw passthrough: emit FS_BatteryState fields exactly as Uncrashed
    -- exposes them. The receiver builds the LFBT-style BatteryPacket
    -- and does all the math (pack voltage, percent remaining, Ah scale).
    local F_CURRENT = "CurrentAmpdrainA_30_5BDC35E347F4521EF29FA58365AB1369"
    pcall(function()
        local pc = find_controller()
        if not valid(pc) then return end
        if not read_bool(pc, "SimulateBattery") then return end
        local bs = pc.S_BatteryState
        if not bs then return end
        snap.cell_count        = bs[F_CELLCOUNT] or 0
        snap.voltage_per_cell  = bs[F_VPC] or 0.0
        snap.current_amps      = bs[F_CURRENT] or 0.0
        snap.charge_used_mah   = bs[F_REMAINING] or 0.0
        snap.capacity_mah      = bs[F_CAPACITY] or 0
        if snap.cell_count > 0 and snap.voltage_per_cell > 0 then
            snap.flags = snap.flags | FLAG_HAS_BATTERY
        end
    end)

    return snap
end

-- ─── main loop ────────────────────────────────────────────────────────────
local function on_tick()
    if not out_file then return end

    local snap = sample()
    local pkt = build_packet(snap)

    -- File was truncated to 0 bytes by io.open("wb") in init(). Every tick
    -- thereafter rewrites the same UCFV_PACKET_SIZE region from offset 0.
    -- Because the packet size is fixed, no truncation is ever needed
    -- mid-flight, and a single UCFV_PACKET_SIZE write() is well below the
    -- kernel's atomicity threshold — the receiver effectively never sees
    -- a torn frame.
    out_file:seek("set", 0)
    out_file:write(pkt)
    -- No flush() needed: setvbuf("no") in init() makes every write
    -- a direct write() syscall.

    if CONFIG.log_every > 0 and (seq % CONFIG.log_every) == 0 then
        log("seq=%d flags=0x%04x pos=(%.1f, %.1f, %.1f) prop_count=%d",
            snap.sequence, snap.flags,
            snap.position[1], snap.position[2], snap.position[3],
            snap.prop_count)
    end
end

-- ─── init ─────────────────────────────────────────────────────────────────
local function init()
    epoch_ms = math.floor(os.clock() * 1e6)

    -- Open once, keep the handle for the lifetime of the mod. "wb" truncates
    -- here (clearing any stale content from a previous mod load); subsequent
    -- writes seek to 0 and overwrite in place.
    local f, err = io.open(PATH, "wb")
    if not f then
        log("FATAL: cannot open %s for write: %s", PATH, tostring(err))
        return
    end
    -- Disable Lua's default 4 KB write buffer — without this, the receiver
    -- would only see updates every ~30 ticks when the buffer fills.
    f:setvbuf("no")
    out_file = f

    log("initialised; writing UCFV → %s every %d ms", PATH, CONFIG.tick_ms)
end

init()

-- UE4SS hook: NotifyOnNewObject fires when a fresh BP_FPVDrone_Pawn_C
-- spawns (e.g. respawn after crash), invalidating the cache.
NotifyOnNewObject("/Script/Engine.Pawn", function(new_obj)
    if new_obj:GetClass():GetFName():ToString():find(CONFIG.drone_class, 1, true) then
        cached_pawn = new_obj
        log("re-acquired drone (NotifyOnNewObject): %s", new_obj:GetFullName())
    end
end)

LoopAsync(CONFIG.tick_ms, function()
    local ok, err = pcall(on_tick)
    if not ok then log("tick error: %s", tostring(err)) end
    return false   -- false = keep looping
end)

using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using BepInEx;
using BepInEx.Configuration;
using HarmonyLib;
using UnityEngine;

namespace LiftoffSimstateBridge
{
    [BepInPlugin(PluginGuid, PluginName, PluginVersion)]
    public class Plugin : BaseUnityPlugin
    {
        public const string PluginGuid = "com.liftoffrs.simstatebridge";
        public const string PluginName = "Liftoff Simstate Bridge";
        public const string PluginVersion = "0.3.0";

        // Two packet kinds share the same UDP port. Both start with a 4-byte ASCII
        // tag so the consumer can dispatch on it. See README.md for the byte-level
        // layout of each.
        private static readonly byte[] DamageMagicTag  = { (byte)'L', (byte)'F', (byte)'D', (byte)'M' };
        private static readonly byte[] BatteryMagicTag = { (byte)'L', (byte)'F', (byte)'B', (byte)'T' };
        private const ushort WireVersion = 1;

        // Damage packet limits.
        private const int MaxProps = 8;
        private const int DamageHeaderSize = 20;
        // Re-emit current damage at this interval even when nothing changed,
        // so the consumer can detect plugin presence and recover from packet loss.
        private const float DamageHeartbeatSeconds = 0.1f;

        // Battery packet is fixed size (no variable arrays); send at fixed cadence.
        private const int BatteryPacketSize = 40;
        private const float BatterySendInterval = 0.1f; // 10 Hz

        // === BatteryDrainer reflection ===
        // The drainer class and its members are obfuscated in Assembly-CSharp.dll
        // (the type and method identifiers are 47-char strings of pure ' and " ASCII
        // bytes, which can't be written as C# identifiers). We resolve them at
        // startup via reflection: the drainer Type comes from the FieldType of
        // DroneHUDModule.AmpTotal.batteryDrainer (a clean, named field), and each
        // getter method is looked up by its raw byte string, encoded here as hex.
        // Mapping was determined by inspecting the IL of the corresponding HUD
        // module Tick() methods — see README.md "Caveats" for the recovery
        // procedure when a Liftoff update breaks this.
        private const string DrainerVoltageNameHex          = "2227222227222722222222222227272727272727222222272727222722272222272727222722222727222227272722";
        private const string DrainerVoltagePerCellNameHex   = "2227222222222227272222272222222222272227222727222227222227272722272227272727222222272222222727";
        private const string DrainerCurrentAmpsNameHex      = "2722272722222727222722272227272227272222272727272222222722222727222722222227222722272722272727";
        private const string DrainerAhDrawnNameHex          = "2722222727222227222227272222222727272222272722222722222727272722222222222222222727272727272722";
        private const string DrainerPercentageNameHex       = "2722222222272727272222222727272722222222222722222222272222272222272227272722222227222222272722";

        private ConfigEntry<string> _cfgAddress;
        private ConfigEntry<int> _cfgPort;

        private UdpClient _udp;
        private IPEndPoint _endpoint;

        // FlightManager.CurrentDrone returns a class whose identifier is obfuscated
        // — we hold the reference as object and downcast to Component for the Unity
        // helpers we need (.gameObject, .transform, GetComponentsInChildren<T>).
        private FlightManager _flightManager;
        private object _cachedDrone;

        // Damage state.
        private Propeller[] _cachedPropellers = Array.Empty<Propeller>();
        private float[] _lastDamageValues = Array.Empty<float>();
        private float _lastDamageSendTime;
        private readonly byte[] _damageBuffer = new byte[DamageHeaderSize + MaxProps * 4];
        // First-emit / state-change diagnostic logging. Throttled so it can't spam.
        private bool _loggedFirstEmit;
        private bool _loggedFlightManagerFound;
        private bool _loggedDroneFound;

        // Battery state.
        private Type _drainerType;
        private MethodInfo _voltageGetter;
        private MethodInfo _voltagePerCellGetter;
        private MethodInfo _currentAmpsGetter;
        private MethodInfo _ahDrawnGetter;
        private MethodInfo _percentageGetter;
        // Cached per-drone: the drainer Component on the current drone, plus the
        // cell count from BatteryPart (a clean named API on the drone build).
        private Component _cachedDrainer;
        private byte _cachedCellCount;
        private float _lastBatterySendTime;
        private readonly byte[] _batteryBuffer = new byte[BatteryPacketSize];
        // Single-element array reused for reflection invocations to avoid per-call alloc.
        private readonly object[] _emptyArgs = Array.Empty<object>();

        // Stopwatch gives a stable monotonic ms clock for the timestamp field.
        private static readonly Stopwatch _wallClock = Stopwatch.StartNew();

        private void Awake()
        {
            _cfgAddress = Config.Bind("Network", "TargetAddress", "127.0.0.1",
                "Destination IP address for the damage + battery UDP stream.");
            _cfgPort = Config.Bind("Network", "TargetPort", 9020,
                "Destination UDP port. Pick a value not used by Liftoff's own telemetry (which defaults to 9001-9003). Both packet types share this port.");

            try
            {
                _udp = new UdpClient();
                _endpoint = new IPEndPoint(IPAddress.Parse(_cfgAddress.Value), _cfgPort.Value);
                Logger.LogInfo($"Liftoff Simstate Bridge ready: emitting to {_cfgAddress.Value}:{_cfgPort.Value}");
            }
            catch (Exception ex)
            {
                Logger.LogError($"Failed to set up UDP socket: {ex.Message}");
                _udp = null;
            }

            ResolveBatteryReflection();

            // Liftoff ships with the CodeStage Anti-Cheat Toolkit (ACTk), which
            // detects unauthorized assemblies and destroys the plugin's
            // GameObject within a frame of Chainloader completing. The user is
            // OK with the resulting "competitive features disabled" warning;
            // we just need our tick to keep running after the plugin is
            // marked destroyed.
            //
            // Workaround: hook FlightManager.Update via Harmony. The C# Plugin
            // instance survives via the static _instance reference (Harmony
            // captures it). FlightManager.Update fires every frame the FM is
            // alive — i.e. every frame in flight, which is when we want
            // telemetry. ACTk would have to patch its own game's IL to
            // unhook us, which it doesn't.
            _instance = this;
            _staticLogger = Logger;
            try
            {
                var harmony = new Harmony(PluginGuid);
                harmony.PatchAll(typeof(FlightManagerUpdatePatch));
                Logger.LogInfo("Harmony patch installed on FlightManager.Update — telemetry will start when you enter a flight.");
            }
            catch (Exception ex)
            {
                Logger.LogError($"Harmony patch failed: {ex}");
            }
        }

        // Static handles so the Harmony patch can reach into the (possibly
        // already-destroyed) Plugin instance. The MonoBehaviour wrapper can
        // be marked destroyed by Unity, but the C# object itself lives on as
        // long as a managed reference is held.
        private static Plugin _instance;
        private static BepInEx.Logging.ManualLogSource _staticLogger;

        [HarmonyPatch(typeof(FlightManager), "Update")]
        private static class FlightManagerUpdatePatch
        {
            [HarmonyPostfix]
            static void Postfix()
            {
                try { _instance?.Tick(); }
                catch (Exception ex) { _staticLogger?.LogError($"Tick threw: {ex}"); }
            }
        }

        private void ResolveBatteryReflection()
        {
            try
            {
                var asm = typeof(Propeller).Assembly; // Assembly-CSharp
                var ampTotalType = asm.GetType("DroneHUDModule.AmpTotal");
                if (ampTotalType == null)
                {
                    Logger.LogWarning("DroneHUDModule.AmpTotal not found; battery telemetry disabled.");
                    return;
                }
                var drainerField = ampTotalType.GetField("batteryDrainer",
                    BindingFlags.NonPublic | BindingFlags.Instance);
                if (drainerField == null)
                {
                    Logger.LogWarning("AmpTotal.batteryDrainer field not found; battery telemetry disabled.");
                    return;
                }
                _drainerType = drainerField.FieldType;

                const BindingFlags bf = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
                _voltageGetter        = _drainerType.GetMethod(HexToAscii(DrainerVoltageNameHex),        bf, null, Type.EmptyTypes, null);
                _voltagePerCellGetter = _drainerType.GetMethod(HexToAscii(DrainerVoltagePerCellNameHex), bf, null, Type.EmptyTypes, null);
                _currentAmpsGetter    = _drainerType.GetMethod(HexToAscii(DrainerCurrentAmpsNameHex),    bf, null, Type.EmptyTypes, null);
                _ahDrawnGetter        = _drainerType.GetMethod(HexToAscii(DrainerAhDrawnNameHex),        bf, null, Type.EmptyTypes, null);
                _percentageGetter     = _drainerType.GetMethod(HexToAscii(DrainerPercentageNameHex),     bf, null, Type.EmptyTypes, null);

                int found = (_voltageGetter        != null ? 1 : 0)
                          + (_voltagePerCellGetter != null ? 1 : 0)
                          + (_currentAmpsGetter    != null ? 1 : 0)
                          + (_ahDrawnGetter        != null ? 1 : 0)
                          + (_percentageGetter     != null ? 1 : 0);
                if (found < 5)
                {
                    Logger.LogWarning($"Battery getter resolution: only {found}/5 drainer methods found. Battery telemetry will be partial.");
                }
                else
                {
                    Logger.LogInfo("Battery telemetry: all 5 drainer getters resolved by reflection.");
                }
            }
            catch (Exception ex)
            {
                Logger.LogWarning($"Battery reflection setup failed: {ex.Message}. Battery telemetry disabled.");
                _drainerType = null;
            }
        }

        private void OnDestroy()
        {
            // Note: ACTk destroys this MonoBehaviour shortly after Awake. The
            // C# instance survives via the static _instance field, so don't
            // tear down _udp here — Tick still needs it.
        }

        private bool _loggedFirstTick;

        private void Tick()
        {
            if (!_loggedFirstTick)
            {
                _staticLogger?.LogInfo($"First Tick from FlightManager.Update postfix (t={Time.unscaledTime:F2}).");
                _loggedFirstTick = true;
            }

            if (_udp == null) return;

            // We're invoked from FlightManager.Update via Harmony, so by
            // definition FlightManager is alive — find it once and cache.
            if (_flightManager == null)
            {
                _flightManager = UnityEngine.Object.FindObjectOfType<FlightManager>();
                if (_flightManager != null && !_loggedFlightManagerFound)
                {
                    _staticLogger?.LogInfo("FlightManager discovered.");
                    _loggedFlightManagerFound = true;
                }
            }

            // CurrentDrone changes whenever the player respawns or switches drones.
            object droneObj = _flightManager != null ? _flightManager.CurrentDrone : null;
            if (!ReferenceEquals(droneObj, _cachedDrone))
            {
                RebuildDroneCaches(droneObj);
                if (droneObj != null && !_loggedDroneFound)
                {
                    _staticLogger?.LogInfo($"Player drone cached — {_cachedPropellers.Length} propeller(s), drainer={(_cachedDrainer != null ? "found" : "missing")}, cells={_cachedCellCount}.");
                    _loggedDroneFound = true;
                }
            }

            EmitDamageIfNeeded();
            EmitBatteryIfDue();
        }

        private void RebuildDroneCaches(object droneObj)
        {
            _cachedDrone = droneObj;
            _cachedPropellers = Array.Empty<Propeller>();
            _lastDamageValues = Array.Empty<float>();
            _cachedDrainer = null;
            _cachedCellCount = 0;

            if (droneObj == null) return;

            // The obfuscated drone class doesn't necessarily inherit from
            // Component — earlier symptom was `as Component` returning null
            // even though CurrentDrone was non-null. Probe several extraction
            // paths and log what we find on first encounter so we have ground
            // truth.
            var droneType = droneObj.GetType();
            GameObject droneGo = ResolveDroneGameObject(droneObj, droneType);

            if (!_loggedDroneTypeProbe)
            {
                _staticLogger?.LogInfo($"Drone object type: {droneType.FullName}");
                _staticLogger?.LogInfo($"  Base: {droneType.BaseType?.FullName}");
                if (droneGo != null)
                {
                    _staticLogger?.LogInfo($"  Resolved GameObject '{droneGo.name}', children={droneGo.transform.childCount}");
                }
                else
                {
                    _staticLogger?.LogWarning("  Could not resolve a GameObject from the drone — listing instance fields/properties for diagnosis:");
                    foreach (var f in droneType.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance))
                        _staticLogger?.LogInfo($"    field {f.FieldType.Name} {f.Name}");
                    foreach (var p in droneType.GetProperties(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance))
                        _staticLogger?.LogInfo($"    prop  {p.PropertyType.Name} {p.Name}");
                }
                _loggedDroneTypeProbe = true;
            }

            if (droneGo == null) return;

            var props = droneGo.GetComponentsInChildren<Propeller>();
            if (props.Length > MaxProps)
            {
                _staticLogger?.LogWarning($"Drone has {props.Length} propellers; truncating to {MaxProps}.");
                Array.Resize(ref props, MaxProps);
            }
            _cachedPropellers = props;
            _lastDamageValues = new float[props.Length];
            for (int i = 0; i < _lastDamageValues.Length; i++) _lastDamageValues[i] = float.NaN;

            if (_drainerType != null)
                _cachedDrainer = droneGo.GetComponentInChildren(_drainerType) as Component;
            var batteryPart = droneGo.GetComponentInChildren<BatteryPart>();
            if (batteryPart != null)
            {
                int n = batteryPart.NrOfCells;
                _cachedCellCount = (n >= 0 && n <= 255) ? (byte)n : (byte)0;
            }
        }

        private bool _loggedDroneTypeProbe;

        private static GameObject ResolveDroneGameObject(object droneObj, Type droneType)
        {
            // Direct paths first.
            if (droneObj is GameObject directGo) return directGo;
            if (droneObj is Component directComp) return directComp.gameObject;

            // Common Unity convention: a `gameObject` property (Component
            // exposes one). Pull via reflection in case the type extends
            // something unusual.
            var goProp = droneType.GetProperty("gameObject",
                BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (goProp != null && goProp.PropertyType == typeof(GameObject))
            {
                if (goProp.GetValue(droneObj) is GameObject g1) return g1;
            }

            // Last resort: scan instance fields for any GameObject reference,
            // or a Component (then take its .gameObject). This finds a
            // gameObject hidden behind a wrapper class — the drone's
            // `droneRigidbody` is one such Component reference we already know
            // exists from earlier metadata inspection.
            const BindingFlags bf = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
            foreach (var f in droneType.GetFields(bf))
            {
                if (typeof(GameObject).IsAssignableFrom(f.FieldType))
                {
                    if (f.GetValue(droneObj) is GameObject g2) return g2;
                }
                else if (typeof(Component).IsAssignableFrom(f.FieldType))
                {
                    if (f.GetValue(droneObj) is Component c) return c.gameObject;
                }
            }
            return null;
        }

        private void EmitDamageIfNeeded()
        {
            bool changed = false;
            for (int i = 0; i < _cachedPropellers.Length; i++)
            {
                var p = _cachedPropellers[i];
                // Propeller can be destroyed mid-frame; treat as a change so the
                // consumer sees a fresh packet with the current prop count.
                if (p == null) { changed = true; break; }
                if (p.DamageState != _lastDamageValues[i]) { changed = true; break; }
            }

            bool heartbeatDue = (Time.unscaledTime - _lastDamageSendTime) >= DamageHeartbeatSeconds;
            if (!changed && !heartbeatDue) return;

            EmitDamagePacket();
        }

        private void EmitDamagePacket()
        {
            int propCount = _cachedPropellers.Length;

            ushort flags = 0;
            if (_flightManager != null)
            {
                if (_flightManager.IsKilled)  flags |= 0x0001;
                if (_flightManager.IsCrashed) flags |= 0x0002;
            }
            if (_cachedDrone == null)         flags |= 0x0004;

            int offset = 0;
            Buffer.BlockCopy(DamageMagicTag, 0, _damageBuffer, offset, 4); offset += 4;
            WriteU16LE(_damageBuffer, offset, WireVersion); offset += 2;
            WriteU16LE(_damageBuffer, offset, flags);       offset += 2;
            WriteU64LE(_damageBuffer, offset, (ulong)_wallClock.ElapsedMilliseconds); offset += 8;
            _damageBuffer[offset++] = (byte)propCount;
            _damageBuffer[offset++] = 0;
            _damageBuffer[offset++] = 0;
            _damageBuffer[offset++] = 0;

            for (int i = 0; i < propCount; i++)
            {
                var p = _cachedPropellers[i];
                float v = (p != null) ? p.DamageState : 0f;
                WriteF32LE(_damageBuffer, offset, v); offset += 4;
                _lastDamageValues[i] = v;
            }

            try
            {
                _udp.Send(_damageBuffer, offset, _endpoint);
                _lastDamageSendTime = Time.unscaledTime;
                if (!_loggedFirstEmit)
                {
                    _staticLogger?.LogInfo($"First UDP packet sent ({offset} bytes) to {_endpoint}.");
                    _loggedFirstEmit = true;
                }
            }
            catch (Exception ex)
            {
                _staticLogger?.LogWarning($"UDP send failed (damage): {ex.Message}");
            }
        }

        private void EmitBatteryIfDue()
        {
            if (_drainerType == null) return;
            if ((Time.unscaledTime - _lastBatterySendTime) < BatterySendInterval) return;
            EmitBatteryPacket();
        }

        private void EmitBatteryPacket()
        {
            ushort flags = 0;
            if (_cachedDrainer == null) flags |= 0x0001;  // NO_DRAINER
            if (_cachedDrone == null)   flags |= 0x0002;  // NO_DRONE

            float voltage = 0f, voltagePerCell = 0f, amps = 0f, ahDrawn = 0f, percentage = 0f;
            if (_cachedDrainer != null)
            {
                try
                {
                    if (_voltageGetter        != null) voltage        = (float)_voltageGetter       .Invoke(_cachedDrainer, _emptyArgs);
                    if (_voltagePerCellGetter != null) voltagePerCell = (float)_voltagePerCellGetter.Invoke(_cachedDrainer, _emptyArgs);
                    if (_currentAmpsGetter    != null) amps           = (float)_currentAmpsGetter   .Invoke(_cachedDrainer, _emptyArgs);
                    if (_ahDrawnGetter        != null) ahDrawn        = (float)_ahDrawnGetter       .Invoke(_cachedDrainer, _emptyArgs);
                    if (_percentageGetter     != null) percentage     = (float)_percentageGetter    .Invoke(_cachedDrainer, _emptyArgs);
                }
                catch (Exception ex)
                {
                    _staticLogger?.LogWarning($"Battery getter invoke failed: {ex.Message}");
                }
            }

            int offset = 0;
            Buffer.BlockCopy(BatteryMagicTag, 0, _batteryBuffer, offset, 4); offset += 4;
            WriteU16LE(_batteryBuffer, offset, WireVersion); offset += 2;
            WriteU16LE(_batteryBuffer, offset, flags);       offset += 2;
            WriteU64LE(_batteryBuffer, offset, (ulong)_wallClock.ElapsedMilliseconds); offset += 8;
            _batteryBuffer[offset++] = _cachedCellCount;
            _batteryBuffer[offset++] = 0;
            _batteryBuffer[offset++] = 0;
            _batteryBuffer[offset++] = 0;
            WriteF32LE(_batteryBuffer, offset, voltage);        offset += 4;
            WriteF32LE(_batteryBuffer, offset, voltagePerCell); offset += 4;
            WriteF32LE(_batteryBuffer, offset, amps);           offset += 4;
            WriteF32LE(_batteryBuffer, offset, ahDrawn);        offset += 4;
            WriteF32LE(_batteryBuffer, offset, percentage);     offset += 4;

            try
            {
                _udp.Send(_batteryBuffer, BatteryPacketSize, _endpoint);
                _lastBatterySendTime = Time.unscaledTime;
            }
            catch (Exception ex)
            {
                _staticLogger?.LogWarning($"UDP send failed (battery): {ex.Message}");
            }
        }

        // Liftoff only ships for little-endian x86_64 platforms, so we serialize
        // little-endian directly without runtime endianness checks.
        private static void WriteU16LE(byte[] b, int off, ushort v)
        {
            b[off]     = (byte)(v & 0xFF);
            b[off + 1] = (byte)((v >> 8) & 0xFF);
        }

        private static void WriteU64LE(byte[] b, int off, ulong v)
        {
            for (int i = 0; i < 8; i++) b[off + i] = (byte)((v >> (8 * i)) & 0xFF);
        }

        private static void WriteF32LE(byte[] b, int off, float v)
        {
            // BitConverter.GetBytes allocates a 4-byte array per call; acceptable at
            // these rates (a few KB/s of short-lived garbage).
            byte[] bytes = BitConverter.GetBytes(v);
            Buffer.BlockCopy(bytes, 0, b, off, 4);
        }

        private static string HexToAscii(string hex)
        {
            var bytes = new byte[hex.Length / 2];
            for (int i = 0; i < bytes.Length; i++)
                bytes[i] = Convert.ToByte(hex.Substring(i * 2, 2), 16);
            return System.Text.Encoding.ASCII.GetString(bytes);
        }
    }
}

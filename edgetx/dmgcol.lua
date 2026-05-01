-- dmgcol.lua -- EdgeTX telemetry script for Liftoff rotor damage
--                Color LCD version (Radiomaster TX16S, Jumper T-Pro,
--                Horus X10/X12, etc.)
--
-- For B&W / 128x64 radios (Pocket, Zorro, Boxer, etc.) use dmgbw.lua.
--
-- Decodes custom CRSF frame type 0x42 sent by liftoff-rs and registers
-- per-rotor health sensors (Hp1..Hp8) that are accessible from EdgeTX
-- telemetry screens, logical switches, voice alerts, and other LUA
-- scripts via getValue("Hp1"), etc.
--
-- Install: copy to /SCRIPTS/TELEMETRY/ on the SD card, then assign to
-- a telemetry screen in Model Setup -> Display. EdgeTX requires the
-- filename (without .lua) to be 6 characters or less.
--
-- Wire format (extended-header broadcast, type 0x42):
--   [dest:1] [origin:1] [flags:1] [n_rotors:1] [health_1:2] ... [health_n:2]
--
-- Flags: bit 0 = killed, bit 1 = crashed, bit 2 = no drone
-- Health: uint16 big-endian, 0-10000 => 0.00% - 100.00%

local FRAME_TYPE = 0x42

-- Unit 13 = UNIT_PERCENT in EdgeTX
local UNIT_PERCENT = 13

-- Cached state for the display
local health = {}
local flags = 0
local lastUpdate = 0

local function init()
end

local function processFrame()
  local cmd, data = crossfireTelemetryPop()
  while cmd ~= nil do
    if cmd == FRAME_TYPE and data ~= nil then
      -- data[1]=dest, data[2]=origin (already stripped by EdgeTX in some
      -- builds; adjust offsets if your firmware includes them).
      -- The LUA queue receives bytes starting after the type byte, so:
      --   data[1] = dest
      --   data[2] = origin
      --   data[3] = flags
      --   data[4] = n_rotors
      --   data[5..] = health values (2 bytes each, big-endian)
      local fOff = 3  -- flags offset
      local nOff = 4  -- n_rotors offset

      if #data >= nOff then
        flags = data[fOff]
        local n = data[nOff]
        health = {}
        for i = 1, math.min(n, 8) do
          local off = nOff + 1 + (i - 1) * 2
          if off + 1 <= #data then
            local h = data[off] * 256 + data[off + 1]
            health[i] = h
            -- Register/update the telemetry sensor so it appears in
            -- EdgeTX screens and is available via getValue("Hp<i>").
            setTelemetryValue(0x4200, i - 1, 0, h, UNIT_PERCENT, 2, "Hp" .. i)
          end
        end
        lastUpdate = getTime()
      end
    end
    cmd, data = crossfireTelemetryPop()
  end
end

local function background()
  processFrame()
end

local function flagStr()
  if bit32.band(flags, 0x04) ~= 0 then return "NO DRONE" end
  if bit32.band(flags, 0x01) ~= 0 then return "KILLED" end
  if bit32.band(flags, 0x02) ~= 0 then return "CRASHED" end
  return ""
end

local function healthColor(h)
  -- h is 0-10000 (0.00% - 100.00%)
  if h >= 8000 then return 0x07E0 end   -- green  (LCD_COLOR_GREEN)
  if h >= 4000 then return 0xFFE0 end   -- yellow (LCD_COLOR_YELLOW)
  return 0xF800                          -- red    (LCD_COLOR_RED)
end

local function run(event)
  processFrame()
  lcd.clear()

  local stale = (getTime() - lastUpdate) > 300  -- 3 seconds

  lcd.drawText(1, 1, "Rotor Health", MIDSIZE)

  local fs = flagStr()
  if fs ~= "" then
    lcd.drawText(1, 25, fs, BOLD + BLINK)
  end

  if #health == 0 then
    lcd.drawText(1, 50, stale and "No data" or "Waiting...", 0)
    return
  end

  local y = 45
  for i = 1, #health do
    local pct = health[i] / 100
    local label = string.format("Rotor %d: %6.2f%%", i, pct)
    lcd.drawText(1, y, label, 0)
    -- Simple bar (width proportional to health, max 100px)
    local barW = math.floor(health[i] / 100)
    lcd.drawFilledRectangle(130, y + 1, barW, 12, healthColor(health[i]))
    lcd.drawRectangle(130, y + 1, 100, 12, 0)
    y = y + 18
  end

  if stale then
    lcd.drawText(1, y + 5, "(stale)", 0)
  end
end

return { init = init, background = background, run = run }

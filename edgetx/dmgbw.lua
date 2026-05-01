-- dmgbw.lua -- EdgeTX telemetry script for Liftoff rotor damage
--               B&W / 128x64 version (Radiomaster Pocket, Zorro,
--               Boxer, TX12, Taranis QX7, etc.)
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
      -- The LUA queue receives bytes starting after the type byte:
      --   data[1] = dest
      --   data[2] = origin
      --   data[3] = flags
      --   data[4] = n_rotors
      --   data[5..] = health values (2 bytes each, big-endian)
      local fOff = 3
      local nOff = 4

      if #data >= nOff then
        flags = data[fOff]
        local n = data[nOff]
        health = {}
        for i = 1, math.min(n, 8) do
          local off = nOff + 1 + (i - 1) * 2
          if off + 1 <= #data then
            local h = data[off] * 256 + data[off + 1]
            health[i] = h
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

-- 128x64 layout
--   Row 0:  Title (MIDSIZE = 8x12 font)
--   Row 13: Status flag or blank
--   Row 22+: 4 rotor rows, 10px each (fits 4 rotors comfortably)
--            For >4 rotors we shrink to 8px rows with SMLSIZE.
local function run(event)
  processFrame()
  lcd.clear()

  local stale = (getTime() - lastUpdate) > 300  -- 3 seconds

  lcd.drawText(1, 0, "Rotor HP", MIDSIZE)

  -- Stale indicator top-right
  if stale and lastUpdate > 0 then
    lcd.drawText(128, 0, "!", SMLSIZE + BOLD + BLINK)
  end

  local fs = flagStr()
  if fs ~= "" then
    lcd.drawText(1, 13, fs, BOLD + BLINK)
  end

  if #health == 0 then
    lcd.drawText(1, 30, stale and "No data" or "Waiting...", 0)
    return
  end

  -- Dynamic row sizing: <=4 rotors get comfortable 10px rows,
  -- >4 use compact 8px rows with SMLSIZE font.
  local compact = #health > 4
  local rowH = compact and 8 or 10
  local font = compact and SMLSIZE or 0
  local barMaxW = 42  -- max bar width in pixels
  local barX = 85     -- bar start X
  local y = 22

  for i = 1, #health do
    local pct = health[i] / 100
    -- Label: "1: 99.9%" or "1:100.0%"
    local label
    if pct >= 100.0 then
      label = string.format("%d:100.0", i)
    else
      label = string.format("%d:%5.1f", i, pct)
    end
    lcd.drawText(1, y, label, font)
    lcd.drawText(lcd.getLastRightPos(), y, "%", font)

    -- Health bar: outline + filled portion
    local barW = math.floor(health[i] * barMaxW / 10000)
    local barH = rowH - 2
    lcd.drawRectangle(barX, y, barMaxW, barH)
    if barW > 0 then
      lcd.drawFilledRectangle(barX, y, barW, barH)
    end

    -- Damage marker: invert the bar when health drops below 40%
    -- so it visually stands out on the 1-bit display.
    if health[i] < 4000 and barW > 0 then
      lcd.drawFilledRectangle(barX, y, barW, barH, INVERS)
    end

    y = y + rowH
  end
end

return { init = init, background = background, run = run }

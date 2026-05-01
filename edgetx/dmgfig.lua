-- dmgfig.lua -- EdgeTX telemetry script for Liftoff rotor damage
--                Quadcopter X-frame diagram version (B&W 128x64)
--
-- Decodes custom CRSF frame type 0x42 sent by liftoff-rs and shows a
-- mini drone outline with each rotor's health percentage drawn next
-- to it. Rotor squares fill from the bottom up proportional to
-- remaining health and are replaced with a bold X when a rotor is
-- destroyed (0%). For non-quad rotor counts use dmgbw.lua instead.
--
-- Also registers Hp1..Hp4 telemetry sensors via setTelemetryValue() so
-- they're available in screens, logical switches, voice alerts, and
-- other LUA scripts via getValue("Hp1"), etc.
--
-- Rotor index -> position mapping comes from liftoff-simstate-bridge,
-- which emits damage in Propeller.MotorRPM order (LF, RF, LB, RB):
--   Hp1 = LF (left front)
--   Hp2 = RF (right front)
--   Hp3 = LB (left back)
--   Hp4 = RB (right back)
--
-- Install: copy to /SCRIPTS/TELEMETRY/ on the SD card, then assign to
-- a telemetry screen in Model Setup -> Display. EdgeTX requires the
-- filename (without .lua) to be 6 characters or less.

local FRAME_TYPE = 0x42
local UNIT_PERCENT = 13

-- Cached state
local health = {}
local flags = 0
local lastUpdate = 0

-- Drone diagram geometry (128x64). Top of screen = front of the drone,
-- so LF/RF are top-left/top-right and LB/RB are bottom-left/bottom-right.
local BODY_CX, BODY_CY = 64, 36
local ROTOR_SIZE = 11  -- rotor square is 7x7 px (gives some fill resolution)
local ROTOR_HALF = 5  -- (ROTOR_SIZE - 1) / 2

-- Order matches the wire format: index 1 = LF, 2 = RF, 3 = LB, 4 = RB.
-- Labels sit laterally next to their rotor: left rotors get a
-- right-aligned label ending just before the rotor, right rotors get
-- a left-aligned label starting just after.
local ROTORS = {
  -- Hp1: LF (left front)
  { cx = 53, cy = 27, lblX = 45, lblY = 23, rightAlign = true  },
  -- Hp2: RF (right front)
  { cx = 75, cy = 27, lblX = 83, lblY = 23, rightAlign = false },
  -- Hp3: LB (left back)
  { cx = 53, cy = 45, lblX = 45, lblY = 43, rightAlign = true  },
  -- Hp4: RB (right back)
  { cx = 75, cy = 45, lblX = 83, lblY = 43, rightAlign = false },
}

local function init()
end

local function processFrame()
  local cmd, data = crossfireTelemetryPop()
  while cmd ~= nil do
    if cmd == FRAME_TYPE and data ~= nil then
      -- LUA queue strips the type byte, so:
      --   data[1] = dest, data[2] = origin
      --   data[3] = flags, data[4] = n_rotors
      --   data[5..] = health values (u16 big-endian, 0..10000)
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

-- Format health (0..10000) as a single-decimal percentage. Variable
-- width (4-6 chars), but the RIGHT alignment flag handles the left
-- column and the right column is naturally left-flush.
local function fmtPct(h)
  return string.format("%.1f%%", h / 100)
end

-- Draw a single rotor at center (cx, cy) with health h (0..10000).
-- Healthy rotor: outlined square filling from bottom up. Dead: bold X.
local function drawRotor(cx, cy, h)
  if h <= 0 then
    -- Bold X spanning 9x9 (slightly larger than the rotor box) so it
    -- visually overrides the arm line drawn earlier.
    lcd.drawLine(cx - 4, cy - 4, cx + 4, cy + 4, SOLID, FORCE)
    lcd.drawLine(cx + 4, cy - 4, cx - 4, cy + 4, SOLID, FORCE)
    -- Add an inner cross stroke for thickness on radios that render
    -- single-pixel lines very thinly.
    lcd.drawLine(cx - 4, cy - 3, cx + 3, cy + 4, SOLID, FORCE)
    lcd.drawLine(cx + 4, cy - 3, cx - 3, cy + 4, SOLID, FORCE)
    return
  end
  local x, y = cx - ROTOR_HALF, cy - ROTOR_HALF
  lcd.drawRectangle(x, y, ROTOR_SIZE, ROTOR_SIZE, FORCE)
  -- Clear inside (so that arm is overwritten)
  lcd.drawFilledRectangle(x + 1, y + 1, ROTOR_SIZE - 2, ROTOR_SIZE - 2, ERASE)
  -- Fill height in pixels, proportional to health (0..10000 -> 0..ROTOR_SIZE).
  local fill = math.floor(h * ROTOR_SIZE / 10000)
  if fill > 0 then
    lcd.drawFilledRectangle(x, y + ROTOR_SIZE - fill, ROTOR_SIZE, fill, FORCE)
  end
end

local function drawDrone()
  -- Arms: only draw to alive rotors so dead ones look detached.
  for i, r in ipairs(ROTORS) do
    if (health[i] or 0) > 0 then
      lcd.drawLine(BODY_CX, BODY_CY, r.cx, r.cy, SOLID, FORCE)
    end
  end
  -- Body: small filled square in the centre.
  lcd.drawFilledRectangle(BODY_CX - 2, BODY_CY - 2, 5, 5)
  -- Rotors on top of arms so the fill is clean.
  for i, r in ipairs(ROTORS) do
    drawRotor(r.cx, r.cy, health[i] or 0)
  end
end

local function drawLabels()
  for i, r in ipairs(ROTORS) do
    local txt = fmtPct(health[i] or 0)
    local lblFlags = SMLSIZE
    if r.rightAlign then lblFlags = lblFlags + RIGHT end
    lcd.drawText(r.lblX, r.lblY, txt, lblFlags)
  end
end

local function run(event)
  processFrame()
  lcd.clear()

  local stale = (getTime() - lastUpdate) > 300  -- 3 seconds

  lcd.drawText(1, 0, "Damage indicator", MIDSIZE)
  if stale and lastUpdate > 0 then
    lcd.drawText(122, 1, "!", SMLSIZE + BOLD + BLINK)
  end

  if #health == 0 then
    lcd.drawText(34, 30, stale and "No data" or "Waiting...", 0)
    return
  end

  if #health ~= 4 then
    lcd.drawText(0, 22, "Diagram view needs", SMLSIZE)
    lcd.drawText(0, 30, ("4 rotors, got %d."):format(#health), SMLSIZE)
    lcd.drawText(0, 40, "Use dmgbw.lua instead.", SMLSIZE)
    return
  end

  drawLabels()
  drawDrone()

  local fs = flagStr()
  if fs ~= "" then
    -- Approx 6 px per char for default font; centre horizontally.
    local w = string.len(fs) * 6
    local x = math.floor((128 - w) / 2)
    lcd.drawText(x, 56, fs, BOLD + BLINK)
  end
end

return { init = init, background = background, run = run }

local ffi = require'ffi'
local lib = {}
local has_mmap, mmap = pcall(require, 'mmap')

local tinsert = require'table'.insert

local nmea_keys = {
  'Timestamp',
  'Validity', -- A-ok, V-invalid
  'Latitude',
  'North_South',
  'Longitude',
  'East_West',
  'Speed', -- knots
  'Course',
  'Datestamp',
  'Variation',
  'EW',
}
local nmea_match_str = {}
for _, name in ipairs(nmea_keys) do
  tinsert(nmea_match_str, "([^,]*)")
end
nmea_match_str = '^$GPRMC'..table.concat(nmea_match_str, ',')

-- Add the checksum
tinsert(nmea_keys, "Checksum")
nmea_match_str = nmea_match_str..'*(.+)$'
--print(nmea_match_str)

ffi.cdef[[
typedef struct laser_return {
  uint16_t distance; // 0.2 cm increments
  uint8_t intensity; // 0 is no return up to 65 meters
} __attribute__((packed)) laser_return;

typedef struct velodyne_block {
  uint8_t flag[2];
  uint16_t azimuth; // Units of 1/100 of a degree
  laser_return returns[32];
} __attribute__((packed)) velodyne_block;

typedef struct velodyne_data {
  velodyne_block blocks[12];
  uint32_t gps_timestamp;
  uint8_t return_mode;
  uint8_t model;
} __attribute__((packed)) velodyne_data;

typedef struct velodyne_position {
  uint8_t unused[198];
  uint32_t gps_timestamp;
  uint32_t blank;
  uint8_t nmea[72];
  uint8_t pad[234];
} __attribute__((packed)) velodyne_position;
]]

-- Raw packet, less the 42 byte UDP header
local VELO_DATA_SZ = 1248 - 42
local VELO_POSITION_SZ = 554 - 42

local function parse_blocks(blocks)
  local lower, upper = {}, {}
  for i=0,11 do
    local block = blocks[i]
    if block.flag[1]==0xEE then
      local distances, intensities = {}, {}
      for i=0, 31 do
        tinsert(distances, block.returns[i].distance)
        tinsert(distances, block.returns[i].intensity)
      end
      tinsert(upper, {
        distances = distances,
        intensities = intensities,
        azimuth = block.azimuth
      })
    elseif block.flag[1]==0xDD then
      local distances, intensities = {}, {}
      for i=0, 31 do
        tinsert(distances, block.returns[i].distance)
        tinsert(distances, block.returns[i].intensity)
      end
      tinsert(lower, {
        distances = distances,
        intensities = intensities,
        azimuth = block.azimuth
      })
    end
  end
  return upper, lower
end

local function parse_data(str)
  if #str~=VELO_DATA_SZ then return false, "Bad packet length" end
  local ptr_pkt_data = ffi.cast('velodyne_data*', str)
  local blocks = ptr_pkt_data.blocks
  local upper, lower = parse_blocks(blocks)
  return {
    upper = upper,
    lower = lower,
    t_us = ptr_pkt_data.gps_timestamp,
    mode = ptr_pkt_data.return_mode,
    model = ptr_pkt_data.model
  }
end
lib.parse_data = parse_data

local function parse_position(str)
  if #str~=VELO_POSITION_SZ then return false, "Bad packet length" end
  local ptr_pkt_pos = ffi.cast('velodyne_position*', str)
  local obj = {t_us = ptr_pkt_pos.gps_timestamp}
  local nmea = ffi.string(ptr_pkt_pos.nmea, 72)
  --obj.nmea = nmea
--  print("NMEA", nmea)
----[[
  for i,v in ipairs{nmea:match(nmea_match_str)} do
    local key, val = nmea_keys[i]
    if key=='Latitude' or key=='Longitude' then
      local dot = v:find'%.'
      local d, m = v:match"([^%.]+)%.([^%.]+)"
      d, m = tonumber(d), tonumber(m)
      val = m and (d + m / 60) or false
    elseif key=='Checksum' then
      val = v -- Not number
    else
      val = tonumber(v) or v
    end
    obj[key] = val
  end
--]]
  return obj
end
lib.parse_position = parse_position

-- Just give the struct
function lib.update(data_str)
  while true do
    local obj = data_str and parse_data(data_str)
    data_str = coroutine.yield(obj)
  end
end

lib.POSITION_PORT = 8308
lib.DATA_PORT = 2368

return lib

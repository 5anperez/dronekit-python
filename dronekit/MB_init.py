


# 1. fetchAttributes.py

#!/usr/bin/env python
"""
Test script to verify the string representations of refactored DroneKit classes.
Tests all the __str__ methods we updated in the refactor.
"""

from dronekit import connect
import time

# Connect to SITL
print("Connecting to vehicle on: 127.0.0.1:14550")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

print("\n" + "="*50)
print("Testing Class String Representations")
print("="*50)

# Test Attitude
print("\n1. Attitude:")
print(f"   {vehicle.attitude}")

# Test LocationGlobal
print("\n2. Location Global Frame:")
if vehicle.location.global_frame:
    print(f"   {vehicle.location.global_frame}")
else:
    print("   Location not yet available")

# Test LocationGlobalRelative
print("\n3. Location Global Relative Frame:")
if vehicle.location.global_relative_frame:
    print(f"   {vehicle.location.global_relative_frame}")
else:
    print("   Location not yet available")

# Test LocationLocal
print("\n4. Location Local Frame:")
if vehicle.location.local_frame:
    print(f"   {vehicle.location.local_frame}")
else:
    print("   Location not yet available")

# Test GPSInfo
print("\n5. GPS Info:")
print(f"   {vehicle.gps_0}")

# Test Wind
print("\n6. Wind:")
if vehicle.wind:
    print(f"   {vehicle.wind}")
else:
    print("   Wind data not available")

# Test Battery
print("\n7. Battery:")
if vehicle.battery:
    print(f"   {vehicle.battery}")
else:
    print("   Battery data not available")

# Test Rangefinder
print("\n8. Rangefinder:")
print(f"   {vehicle.rangefinder}")

# Test Version
print("\n9. Version:")
print(f"   {vehicle.version}")

# Test VehicleMode
print("\n10. Vehicle Mode:")
print(f"    {vehicle.mode}")

# Test SystemStatus
print("\n11. System Status:")
if vehicle.system_status:
    print(f"    {vehicle.system_status}")
else:
    print("    System status not available")

# Test Gimbal
print("\n12. Gimbal:")
print(f"    {vehicle.gimbal}")

# Test Capabilities
print("\n13. Capabilities:")
if vehicle.capabilities:
    print(f"    mission_float: {vehicle.capabilities.mission_float}")
    print(f"    mission_int: {vehicle.capabilities.mission_int}")
    print(f"    command_int: {vehicle.capabilities.command_int}")
    print(f"    param_float: {vehicle.capabilities.param_float}")
    print(f"    terrain: {vehicle.capabilities.terrain}")
else:
    print("    Capabilities not available")

# Test Parameters (sample a few)
print("\n14. Parameters (sample):")
params_to_check = ['THR_MIN', 'ARMING_CHECK', 'WPNAV_SPEED']
for param in params_to_check:
    try:
        value = vehicle.parameters.get(param)
        if value is not None:
            print(f"    {param}: {value}")
    except:
        print(f"    {param}: Not available")

print("\n" + "="*50)
print("String representation tests completed!")
print("="*50)

# Close vehicle object
vehicle.close()
print("\nConnection closed.")
























# 2. fetchChannelOverrides.py


#!/usr/bin/env python
"""
Test script to verify RC channels and channel overrides functionality.
"""

from dronekit import connect
import time

# Connect to SITL
print("Connecting to vehicle on: 127.0.0.1:14550")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

print("\n" + "="*50)
print("Testing RC Channels and Overrides")
print("="*50)

# Test reading RC channels
print("\n1. Reading RC Channels:")
print(f"   Channel count: {vehicle.channels.count}")
for i in range(1, min(9, vehicle.channels.count + 1)):
    try:
        value = vehicle.channels[str(i)]
        print(f"   Channel {i}: {value}")
    except KeyError:
        print(f"   Channel {i}: Not available")

# Test channel overrides
print("\n2. Testing Channel Overrides:")

# Clear any existing overrides
print("   Clearing all overrides...")
vehicle.channels.overrides = {}
time.sleep(1)

# Set some overrides
print("   Setting channel overrides...")
vehicle.channels.overrides = {'1': 1500, '2': 1600, '3': 1100}
time.sleep(1)

# Read back overrides
print("   Current overrides:")
try:
    for ch in ['1', '2', '3']:
        if ch in vehicle.channels.overrides:
            print(f"   Channel {ch} override: {vehicle.channels.overrides[ch]}")
except KeyError as e:
    print(f"   Error reading override: {e}")

# Test individual override setting
print("\n3. Testing individual override operations:")
print("   Setting channel 4 override to 1700...")
vehicle.channels.overrides['4'] = 1700
time.sleep(1)

print("   Current overrides after individual set:")
for ch in ['1', '2', '3', '4']:
    try:
        if ch in vehicle.channels.overrides:
            print(f"   Channel {ch}: {vehicle.channels.overrides[ch]}")
    except KeyError:
        pass

# Test clearing individual override
print("\n4. Testing override clearing:")
print("   Clearing channel 2 override...")
vehicle.channels.overrides['2'] = None
time.sleep(1)

print("   Overrides after clearing channel 2:")
for ch in ['1', '2', '3', '4']:
    try:
        if ch in vehicle.channels.overrides:
            print(f"   Channel {ch}: {vehicle.channels.overrides[ch]}")
    except KeyError:
        pass

# Test del operation
print("\n5. Testing del operation on channel 3:")
try:
    del vehicle.channels.overrides['3']
    time.sleep(1)
    print("   Successfully deleted channel 3 override")
except Exception as e:
    print(f"   Error deleting override: {e}")

print("   Final overrides:")
for ch in ['1', '2', '3', '4']:
    try:
        if ch in vehicle.channels.overrides:
            print(f"   Channel {ch}: {vehicle.channels.overrides[ch]}")
    except KeyError:
        pass

# Clear all overrides at the end
print("\n6. Clearing all overrides...")
vehicle.channels.overrides = {}
print("   All overrides cleared.")

print("\n" + "="*50)
print("Channel override tests completed!")
print("="*50)

# Close vehicle object
vehicle.close()
print("\nConnection closed.")







































# 3. fetchVehicleState.py


#!/usr/bin/env python
"""
Test script to verify vehicle state changes and perform a cross/plus sign maneuver.
The copter will:
1. Take off to 5m
2. Perform a cross pattern
3. RTL and land
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

def arm_and_takeoff(vehicle, target_altitude):
    """Arms vehicle and fly to target altitude."""
    print(f"Basic pre-arm checks...")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height
    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.1f}m")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_velocity(vehicle, vx, vy, vz, duration):
    """
    Send velocity command to vehicle.
    vx: Velocity in North direction (m/s)
    vy: Velocity in East direction (m/s)  
    vz: Velocity in Down direction (m/s, negative for up)
    duration: Time to maintain velocity (seconds)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)
    
    # Send command to vehicle
    for _ in range(int(duration * 10)):  # Send at 10Hz
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

# Connect to SITL
print("Connecting to vehicle on: 127.0.0.1:14550")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

print("\n" + "="*50)
print("Vehicle State Test with Cross Maneuver")
print("="*50)

try:
    # Initial state
    print(f"\nInitial state:")
    print(f"  Mode: {vehicle.mode}")
    print(f"  Armed: {vehicle.armed}")
    print(f"  Location: {vehicle.location.global_relative_frame}")
    
    # Arm and takeoff
    arm_and_takeoff(vehicle, 5)
    
    print("\nStarting cross maneuver...")
    print("Hovering for 3 seconds...")
    time.sleep(3)
    
    # Cross maneuver parameters
    velocity = 2.0  # m/s
    full_duration = 4.0  # seconds for full leg
    half_duration = full_duration / 2.0
    
    # Save starting position
    start_loc = vehicle.location.global_relative_frame
    print(f"Starting position: Lat={start_loc.lat:.6f}, Lon={start_loc.lon:.6f}")
    
    # Cross pattern:
    # 1. North (forward) - full duration
    print("\n1. Moving North (forward)...")
    send_velocity(vehicle, velocity, 0, 0, full_duration)
    
    # 2. South (backward) - half duration to center
    print("2. Moving South (backward) to center...")
    send_velocity(vehicle, -velocity, 0, 0, half_duration)
    
    # 3. East (right) - half duration
    print("3. Moving East (right)...")
    send_velocity(vehicle, 0, velocity, 0, half_duration)
    
    # 4. West (left) - full duration
    print("4. Moving West (left) through center...")
    send_velocity(vehicle, 0, -velocity, 0, full_duration)
    
    # 5. East (right) - half duration back to center
    print("5. Moving East (right) to center...")
    send_velocity(vehicle, 0, velocity, 0, half_duration)
    
    # 6. South (backward) - half duration
    print("6. Moving South (backward)...")
    send_velocity(vehicle, -velocity, 0, 0, half_duration)
    
    # 7. North (forward) - half duration back to start
    print("7. Moving North (forward) to starting position...")
    send_velocity(vehicle, velocity, 0, 0, half_duration)
    
    print("\nCross maneuver complete!")
    end_loc = vehicle.location.global_relative_frame
    print(f"End position: Lat={end_loc.lat:.6f}, Lon={end_loc.lon:.6f}")
    
    # Calculate position error
    lat_error = abs(start_loc.lat - end_loc.lat)
    lon_error = abs(start_loc.lon - end_loc.lon)
    print(f"Position error: Lat={lat_error:.6f}, Lon={lon_error:.6f}")
    
    print("\nHovering for 2 seconds...")
    time.sleep(2)
    
    # RTL and land
    print("\nReturning to Launch (RTL)...")
    vehicle.mode = VehicleMode("RTL")
    
    # Monitor landing
    while vehicle.armed:
        print(f"  Altitude: {vehicle.location.global_relative_frame.alt:.1f}m")
        time.sleep(1)
    
    print("\nLanded successfully!")
    print(f"Final mode: {vehicle.mode}")
    print(f"Armed: {vehicle.armed}")

except KeyboardInterrupt:
    print("\nUser interrupt - switching to RTL...")
    vehicle.mode = VehicleMode("RTL")

except Exception as e:
    print(f"\nError occurred: {e}")
    print("Attempting RTL...")
    try:
        vehicle.mode = VehicleMode("RTL")
    except:
        pass

finally:
    print("\n" + "="*50)
    print("Vehicle state test completed!")
    print("="*50)
    
    # Close vehicle object
    vehicle.close()
    print("\nConnection closed.")














































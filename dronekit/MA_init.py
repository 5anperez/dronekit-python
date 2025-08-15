#!/usr/bin/env python3
"""
Simple mission script for SITL copter using modernized DroneKit API.
Connects to vehicle, takes off to 5m, then flies to specified coordinates.
"""

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Connection parameters
SITL_ADDRESS = "127.0.0.1:14550"

# Mission parameters
TAKEOFF_ALTITUDE = 5.0  # meters
TARGET_ALTITUDE = 10.0  # meters

# TODO: Replace these with your actual target coordinates
TARGET_LAT = -35.363261  # Placeholder latitude
TARGET_LON = 149.165230  # Placeholder longitude





# Add distance monitoring
def get_distance_metres(loc1, loc2):
    """Calculate ground distance between two LocationGlobal objects."""
    from math import radians, sin, cos, sqrt, atan2
    
    R = 6371000  # Earth's radius in meters
    lat1, lon1 = radians(loc1.lat), radians(loc1.lon)
    lat2, lon2 = radians(loc2.lat), radians(loc2.lon)
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    
    return R * c

# Monitor until close to target
while True:
    current_location = vehicle.location.global_relative_frame
    distance = get_distance_metres(current_location, target_location)
    print(f"Distance to target: {distance:.1f}m")
    
    if distance < 2:  # Within 2 meters
        print("Reached target location!")
        break
        
    time.sleep(1)






def main():
    print("Connecting to vehicle on: %s" % SITL_ADDRESS)
    
    # Connect to the Vehicle
    vehicle = connect(SITL_ADDRESS, wait_ready=True, timeout=30)
    
    print("Vehicle connected!")
    print(" GPS: %s" % vehicle.gps_0)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name)
    
    # Wait for vehicle to be armable
    print("\nWaiting for vehicle to become armable...")
    vehicle.wait_for_armable(timeout=60)
    print("Vehicle is now armable")
    
    # Change to GUIDED mode
    print("\nChanging to GUIDED mode...")
    vehicle.wait_for_mode("GUIDED", timeout=30)
    print("Vehicle is now in GUIDED mode")
    
    # Arm the vehicle
    print("\nArming motors...")
    vehicle.arm(wait=True, timeout=30)
    print("Vehicle armed!")
    
    # Take off to target altitude
    print("\nTaking off to %s meters..." % TAKEOFF_ALTITUDE)
    vehicle.simple_takeoff(TAKEOFF_ALTITUDE)
    
    # Wait until the vehicle reaches a safe height
    vehicle.wait_for_alt(TAKEOFF_ALTITUDE, epsilon=0.5, timeout=30)
    print("Reached target altitude of %s meters" % TAKEOFF_ALTITUDE)
    
    # Hover for a moment
    print("\nHovering for 5 seconds...")
    time.sleep(5)
    
    # Create target location
    target_location = LocationGlobalRelative(
        lat=TARGET_LAT,
        lon=TARGET_LON, 
        alt=TARGET_ALTITUDE
    )
    
    print("\nFlying to target location:")
    print(" Target: lat=%s, lon=%s, alt=%s" % (TARGET_LAT, TARGET_LON, TARGET_ALTITUDE))
    
    # Fly to the target location
    vehicle.simple_goto(target_location, groundspeed=5)  # 5 m/s ground speed
    
    # Monitor progress (optional - you can add more sophisticated monitoring here)
    print("\nFlying to waypoint... (monitoring for 30 seconds)")
    for i in range(30):
        current_location = vehicle.location.global_relative_frame
        print(" Current Location: lat=%s, lon=%s, alt=%s" % 
              (current_location.lat, current_location.lon, current_location.alt))
        time.sleep(1)
    
    print("\nMission segment complete!")
    print("Vehicle will continue flying to target or await further commands.")
    
    # Keep the script running to maintain connection
    print("\nScript will keep running. Press Ctrl+C to exit and close connection.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nClosing vehicle connection...")
        vehicle.close()
        print("Connection closed.")










if __name__ == "__main__":
    main()









































































































# # 1. fetchAttributes.py

# #!/usr/bin/env python
# """
# Test script to verify the string representation methods of refactored classes.
# Tests all the __str__ methods we updated in the refactor.
# """

# from dronekit import connect
# import time

# def test_attribute_strings(vehicle):
#     """Test string representations of various vehicle attributes."""
    
#     print("="*60)
#     print("TESTING ATTRIBUTE STRING REPRESENTATIONS")
#     print("="*60)
    
#     # Test Attitude
#     print("\n1. Testing Attitude:")
#     print(f"   {vehicle.attitude}")
    
#     # Test LocationGlobal
#     print("\n2. Testing Location Global:")
#     print(f"   {vehicle.location.global_frame}")
    
#     # Test LocationGlobalRelative
#     print("\n3. Testing Location Global Relative:")
#     print(f"   {vehicle.location.global_relative_frame}")
    
#     # Test LocationLocal
#     print("\n4. Testing Location Local:")
#     print(f"   {vehicle.location.local_frame}")
    
#     # Test GPSInfo
#     print("\n5. Testing GPS Info:")
#     print(f"   {vehicle.gps_0}")
    
#     # Test Battery
#     print("\n6. Testing Battery:")
#     print(f"   {vehicle.battery}")
    
#     # Test Rangefinder
#     print("\n7. Testing Rangefinder:")
#     print(f"   {vehicle.rangefinder}")
    
#     # Test Version
#     print("\n8. Testing Version:")
#     print(f"   {vehicle.version}")
    
#     # Test VehicleMode
#     print("\n9. Testing Vehicle Mode:")
#     print(f"   {vehicle.mode}")
    
#     # Test SystemStatus
#     print("\n10. Testing System Status:")
#     print(f"   {vehicle.system_status}")
    
#     # Test Wind (may be None if not available)
#     print("\n11. Testing Wind:")
#     print(f"   {vehicle.wind}")
    
#     # Test Gimbal
#     print("\n12. Testing Gimbal:")
#     print(f"   {vehicle.gimbal}")
    
#     # Test Capabilities
#     print("\n13. Testing Capabilities:")
#     if vehicle.capabilities:
#         print(f"   Mission Float: {vehicle.capabilities.mission_float}")
#         print(f"   Param Float: {vehicle.capabilities.param_float}")
#         print(f"   Mission Int: {vehicle.capabilities.mission_int}")
#         print(f"   Command Int: {vehicle.capabilities.command_int}")
#         print(f"   Terrain: {vehicle.capabilities.terrain}")
#     else:
#         print("   Capabilities not available")
    
#     print("\n" + "="*60)
#     print("STRING REPRESENTATION TEST COMPLETED")
#     print("="*60)






# def main():
#     # Connect to SITL
#     print("Connecting to vehicle on: 127.0.0.1:14550")
#     vehicle = connect('127.0.0.1:14550', wait_ready=True)
    
#     try:
#         # Wait a bit for all attributes to populate
#         print("Waiting for attributes to populate...")
#         time.sleep(2)
        
#         # Run the string representation tests
#         test_attribute_strings(vehicle)
        
#     except Exception as e:
#         print(f"Error during testing: {e}")
#         import traceback
#         traceback.print_exc()
    
#     finally:
#         # Close vehicle connection
#         print("\nClosing vehicle connection...")
#         vehicle.close()




# if __name__ == '__main__':
#     main()











# # 2. fetchChannelOverrides.py

# #!/usr/bin/env python
# """
# Test script to verify ChannelsOverride and Channels classes work correctly.
# Tests reading RC channels and setting/clearing overrides.
# """

# from dronekit import connect
# import time

# def test_channel_operations(vehicle):
#     """Test channel reading and override operations."""
    
#     print("="*60)
#     print("TESTING RC CHANNELS AND OVERRIDES")
#     print("="*60)
    
#     # Test 1: Read current RC channel values
#     print("\n1. Current RC Channel Values:")
#     print(f"   Channel count: {vehicle.channels.count}")
#     for i in range(1, min(9, vehicle.channels.count + 1)):
#         print(f"   Channel {i}: {vehicle.channels[str(i)]}")
    

#     # Test 2: Test channel overrides dictionary operations
#     print("\n2. Testing Channel Override Operations:")
    
#     # Clear any existing overrides
#     print("   Clearing all overrides...")
#     vehicle.channels.overrides = {}
#     time.sleep(0.5)
    
#     # Set some overrides using dictionary syntax
#     print("   Setting overrides using dictionary syntax...")
#     vehicle.channels.overrides = {'1': 1500, '2': 1600, '3': 1100}
#     time.sleep(0.5)
#     print(f"   Current overrides: {dict(vehicle.channels.overrides)}")
    
#     # Test 3: Individual override operations
#     print("\n3. Testing Individual Override Operations:")
    
#     # Set individual override
#     print("   Setting channel 4 override to 1700...")
#     vehicle.channels.overrides['4'] = 1700
#     time.sleep(0.5)
#     print(f"   Channel 4 override: {vehicle.channels.overrides.get('4', 'Not set')}")
    
#     # Clear individual override
#     print("   Clearing channel 2 override...")
#     vehicle.channels.overrides['2'] = None
#     time.sleep(0.5)
#     print(f"   Current overrides: {dict(vehicle.channels.overrides)}")
    
#     # Test 4: Delete operation
#     print("\n4. Testing Delete Operation:")
#     print("   Deleting channel 3 override using del...")
#     if '3' in vehicle.channels.overrides:
#         del vehicle.channels.overrides['3']
#     time.sleep(0.5)
#     print(f"   Current overrides: {dict(vehicle.channels.overrides)}")
    
#     # Test 5: Edge cases
#     print("\n5. Testing Edge Cases:")
    
#     # Try invalid channel
#     try:
#         print("   Attempting to set invalid channel (9)...")
#         vehicle.channels.overrides['9'] = 1500
#     except KeyError as e:
#         print(f"   Expected error caught: {e}")
    
#     # Test 6: Clear all overrides
#     print("\n6. Clearing All Overrides:")
#     print("   Setting all overrides to empty dictionary...")
#     vehicle.channels.overrides = {}
#     time.sleep(0.5)
#     print(f"   Final overrides: {dict(vehicle.channels.overrides)}")
    
#     print("\n" + "="*60)
#     print("CHANNEL OVERRIDE TEST COMPLETED")
#     print("="*60)

# def main():
#     # Connect to SITL
#     print("Connecting to vehicle on: 127.0.0.1:14550")
#     vehicle = connect('127.0.0.1:14550', wait_ready=True)
    
#     try:
#         # Wait for initialization
#         print("Waiting for initialization...")
#         time.sleep(2)
        
#         # Run channel tests
#         test_channel_operations(vehicle)
        
#     except Exception as e:
#         print(f"Error during testing: {e}")
#         import traceback
#         traceback.print_exc()
    
#     finally:
#         # Make sure to clear any overrides before closing
#         print("\nCleaning up - clearing all overrides...")
#         vehicle.channels.overrides = {}
        
#         # Close vehicle connection
#         print("Closing vehicle connection...")
#         vehicle.close()

# if __name__ == '__main__':
#     main()



































# # 3. fetchVehicleState.py


# #!/usr/bin/env python
# """
# Test script to verify vehicle state changes and command execution.
# Performs takeoff, cross/plus sign maneuver, and RTL.
# """

# from dronekit import connect, VehicleMode, LocationGlobalRelative
# import time
# import math

# def arm_and_takeoff(vehicle, target_altitude):
#     """Arms vehicle and fly to target_altitude."""
    
#     print(f"Basic pre-arm checks")
#     # Don't try to arm until autopilot is ready
#     while not vehicle.is_armable:
#         print(" Waiting for vehicle to initialise...")
#         time.sleep(1)

#     print("Arming motors")
#     # Copter should arm in GUIDED mode
#     vehicle.mode = VehicleMode("GUIDED")
#     vehicle.armed = True

#     # Confirm vehicle armed before attempting to take off
#     while not vehicle.armed:
#         print(" Waiting for arming...")
#         time.sleep(1)

#     print("Taking off!")
#     vehicle.simple_takeoff(target_altitude)

#     # Wait until the vehicle reaches a safe height
#     while True:
#         print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
#         # Break and return from function just below target altitude.
#         if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
#             print("Reached target altitude")
#             break
#         time.sleep(1)

# def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
#     """
#     Move vehicle in direction based on specified velocity vectors.
#     velocity_x: Velocity in meters/second (positive is forward, negative backward)
#     velocity_y: Velocity in meters/second (positive is right, negative left)
#     velocity_z: Velocity in meters/second (positive is down, negative up)
#     """
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#         0,       # time_boot_ms (not used)
#         0, 0,    # target system, target component
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
#         0b0000111111000111,  # type_mask (only speeds enabled)
#         0, 0, 0,  # x, y, z positions (not used)
#         velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
#         0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
#         0, 0)     # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

#     # Send command to vehicle
#     for _ in range(0, int(duration * 10)):  # Send at 10Hz for duration
#         vehicle.send_mavlink(msg)
#         time.sleep(0.1)

# def perform_cross_maneuver(vehicle, velocity=2.0, leg_duration=3.0):
#     """
#     Perform a cross/plus sign maneuver.
#     The vehicle moves in a cross pattern and returns to starting position.
#     """
#     print("\n" + "="*60)
#     print("PERFORMING CROSS MANEUVER")
#     print("="*60)
    
#     # Record starting position
#     start_location = vehicle.location.global_relative_frame
#     print(f"Starting position: Lat={start_location.lat}, Lon={start_location.lon}, Alt={start_location.alt}")
    
#     # Cross maneuver sequence:
#     # 1. Forward (full duration)
#     print("\n1. Moving forward...")
#     send_ned_velocity(vehicle, velocity, 0, 0, leg_duration)
    
#     # 2. Backward to center (half duration)
#     print("2. Moving backward to center...")
#     send_ned_velocity(vehicle, -velocity, 0, 0, leg_duration/2)
    
#     # 3. Right (half duration)
#     print("3. Moving right...")
#     send_ned_velocity(vehicle, 0, velocity, 0, leg_duration/2)
    
#     # 4. Left to center (half duration)
#     print("4. Moving left to center...")
#     send_ned_velocity(vehicle, 0, -velocity, 0, leg_duration/2)
    
#     # 5. Backward (half duration)
#     print("5. Moving backward...")
#     send_ned_velocity(vehicle, -velocity, 0, 0, leg_duration/2)
    
#     # 6. Forward to center (half duration)
#     print("6. Moving forward to center...")
#     send_ned_velocity(vehicle, velocity, 0, 0, leg_duration/2)
    
#     # 7. Left (half duration)
#     print("7. Moving left...")
#     send_ned_velocity(vehicle, 0, -velocity, 0, leg_duration/2)
    
#     # 8. Right to complete cross (full duration back to start)
#     print("8. Moving right to complete cross...")
#     send_ned_velocity(vehicle, 0, velocity, 0, leg_duration)
    
#     # Stop any residual movement
#     print("\nStopping movement...")
#     send_ned_velocity(vehicle, 0, 0, 0, 1)
    
#     # Report final position
#     end_location = vehicle.location.global_relative_frame
#     print(f"\nEnding position: Lat={end_location.lat}, Lon={end_location.lon}, Alt={end_location.alt}")
    
#     print("\nCross maneuver completed!")

# def test_vehicle_state(vehicle):
#     """Test various vehicle state queries during the mission."""
    
#     print("\n" + "="*60)
#     print("VEHICLE STATE INFORMATION")
#     print("="*60)
    
#     print(f"\nMode: {vehicle.mode}")
#     print(f"Armed: {vehicle.armed}")
#     print(f"System Status: {vehicle.system_status}")
#     print(f"Heading: {vehicle.heading}°")
#     print(f"Groundspeed: {vehicle.groundspeed} m/s")
#     print(f"Airspeed: {vehicle.airspeed} m/s")
#     print(f"GPS: {vehicle.gps_0}")
#     print(f"Battery: {vehicle.battery}")
#     print(f"EKF OK: {vehicle.ekf_ok}")
#     print(f"Last Heartbeat: {vehicle.last_heartbeat}")
#     print(f"Is Armable: {vehicle.is_armable}")
    
#     # Test velocity
#     print(f"\nVelocity [vx, vy, vz]: {vehicle.velocity}")
    
#     # Test all location frames
#     print(f"\nLocation (Global): {vehicle.location.global_frame}")
#     print(f"Location (Relative): {vehicle.location.global_relative_frame}")
#     print(f"Location (Local): {vehicle.location.local_frame}")

# def main():
#     # Import mavutil for velocity commands
#     global mavutil
#     from pymavlink import mavutil
    
#     # Connect to SITL
#     print("Connecting to vehicle on: 127.0.0.1:14550")
#     vehicle = connect('127.0.0.1:14550', wait_ready=True)
    
#     try:
#         # Initial state check
#         print("\nInitial vehicle state:")
#         test_vehicle_state(vehicle)
        
#         # Arm and takeoff to 5 meters
#         print("\n" + "="*60)
#         print("ARMING AND TAKING OFF")
#         print("="*60)
#         arm_and_takeoff(vehicle, 5)
        
#         # Check state after takeoff
#         print("\nVehicle state after takeoff:")
#         test_vehicle_state(vehicle)
        
#         # Perform the cross maneuver
#         time.sleep(2)  # Stabilize
#         perform_cross_maneuver(vehicle, velocity=2.0, leg_duration=3.0)
        
#         # Final state check
#         print("\nVehicle state after maneuver:")
#         test_vehicle_state(vehicle)
        
#         # Return to Launch
#         print("\n" + "="*60)
#         print("RETURNING TO LAUNCH (RTL)")
#         print("="*60)
#         print("Setting RTL mode...")
#         vehicle.mode = VehicleMode("RTL")
        
#         # Monitor RTL
#         while vehicle.armed:
#             print(f"Altitude: {vehicle.location.global_relative_frame.alt}, Mode: {vehicle.mode}")
#             time.sleep(2)
            
#         print("\nLanded and disarmed!")
        
#         # Final state
#         print("\nFinal vehicle state:")
#         test_vehicle_state(vehicle)
        
#     except Exception as e:
#         print(f"Error during testing: {e}")
#         import traceback
#         traceback.print_exc()
        
#         # Emergency RTL
#         print("\nEmergency RTL...")
#         vehicle.mode = VehicleMode("RTL")
    
#     finally:
#         # Close vehicle connection
#         print("\nClosing vehicle connection...")
#         vehicle.close()

# if __name__ == '__main__':
#     main()






















































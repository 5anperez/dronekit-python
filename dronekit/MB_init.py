#!/usr/bin/env python3
"""
SITL Copter Control Script
Commands a copter to takeoff, hover at 5m, then fly to specified coordinates
"""

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Connection parameters
CONNECTION_STRING = '127.0.0.1:14550'

# Mission parameters
TAKEOFF_ALTITUDE = 5.0  # meters
TARGET_ALTITUDE = 10.0  # meters for waypoint

# Placeholder coordinates - replace with your actual coordinates
TARGET_LATITUDE = -35.363261  # placeholder latitude
TARGET_LONGITUDE = 149.165230  # placeholder longitude






def arm_and_takeoff(vehicle, target_altitude):
    """
    Arms vehicle and flies to target altitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {target_altitude}m!")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height before processing the goto
    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)









def main():
    """
    Main mission execution
    """
    print(f"Connecting to vehicle on: {CONNECTION_STRING}")
    
    # Connect to the Vehicle
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
    
    print(f"Vehicle version: {vehicle.version}")
    print(f"Vehicle location: {vehicle.location.global_frame}")
    
    try:
        # Arm and takeoff to 5 meters
        arm_and_takeoff(vehicle, TAKEOFF_ALTITUDE)
        
        # Hover for a moment at 5m
        print("Hovering at 5m for 5 seconds...")
        time.sleep(5)
        
        # Create target location
        target_location = LocationGlobalRelative(
            lat=TARGET_LATITUDE,
            lon=TARGET_LONGITUDE, 
            alt=TARGET_ALTITUDE
        )
        
        print(f"Flying to waypoint: lat={TARGET_LATITUDE}, lon={TARGET_LONGITUDE}, alt={TARGET_ALTITUDE}m")
        
        # Set airspeed to 5 m/s (optional)
        vehicle.airspeed = 5
        
        # Fly to the target location
        vehicle.simple_goto(target_location)
        
        # Monitor progress (basic implementation)
        while True:
            current_location = vehicle.location.global_relative_frame
            print(f" Current Location: lat={current_location.lat:.6f}, "
                  f"lon={current_location.lon:.6f}, alt={current_location.alt:.1f}m")
            
            # Calculate rough distance to target (simplified 2D distance)
            import math
            lat_diff = abs(current_location.lat - TARGET_LATITUDE)
            lon_diff = abs(current_location.lon - TARGET_LONGITUDE)
            
            # Very rough approximation - for demonstration purposes
            # In production, use proper geodesic distance calculation
            distance = math.sqrt((lat_diff * 111000)**2 + (lon_diff * 111000)**2)  # rough meters
            
            print(f" Approximate distance to target: {distance:.1f}m")
            
            # Consider target reached if within 2 meters (rough approximation)
            if distance < 2:
                print("Reached target location!")
                break
                
            time.sleep(2)
        
        # Hover at target for a moment
        print("Hovering at target location for 5 seconds...")
        time.sleep(5)
        
        print("Mission complete!")
        
    except KeyboardInterrupt:
        print("\nUser interrupt detected")
        
    except Exception as e:
        print(f"An error occurred: {e}")
        
    finally:
        # Return to launch (RTL)
        print("Setting RTL mode...")
        vehicle.mode = VehicleMode("RTL")
        
        # Wait a moment for mode change to take effect
        time.sleep(2)
        
        # Close vehicle connection
        print("Closing vehicle connection")
        vehicle.close()
        print("Connection closed")








if __name__ == '__main__':
    main()























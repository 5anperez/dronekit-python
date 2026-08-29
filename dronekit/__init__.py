"""
### This is the API Reference for the DroneKit-Python API.

The main API is the `Vehicle` class. The code snippet below shows how to use `connect` to obtain an instance of a connected vehicle:

```python
    from dronekit import connect

    # Connect to the Vehicle using "connection string" (in this case an address on network)
    vehicle = connect("127.0.0.1:14550", wait_ready=True)
```

- `Vehicle` provides access to the vehicle's state through Python attributes (e.g. `Vehicle.mode`) and settings/parameters through the `Vehicle.parameters` attribute.

- Asynchronous notification on vehicle attribute changes is available by registering listeners/observers.

- Vehicle movement is primarily controlled using the `Vehicle.armed` attribute and
then the `Vehicle.simple_takeoff` method and/or the `Vehicle.simple_goto` method. Note that the vehicle must be in the `GUIDED` flight mode to use these methods.

- Velocity-based movement and control over other vehicle features can be achieved using custom MAVLink messages wrapped in methods such as `Vehicle.send_mavlink` and/or `Vehicle.message_factory`.

- It is also possible to work with vehicle missions when in `AUTO` flight mode by using the `Vehicle.commands` attribute.

- All the logging is handled through the builtin Python `logging` module.
"""
from collections.abc import MutableMapping, Callable, Iterator
import copy
import logging
import math
import struct
import time
import monotonic
from typing import Any
from dataclasses import dataclass, field

from pymavlink import mavutil, mavwp
from pymavlink.dialects.v20 import ardupilotmega
from dronekit.util import ErrprinterHandler





"""
TODO:
- ARE YOU SURE THAT THE TYPE HINTS ARE CORRECT? E.G., FOR THE BATTERY CLASS, ARE VOLTS AND CURRENT REALLY INTS?
- CREATE AN ENUM CLASS OR MAP FOR THE ATTRIBUTE, PROPERTY, AND PARAMETER NAMES, E.G., 'RANGEFINDER' = 0, ATTITUDE = 1, ETC.
- 
"""








class APIException(Exception):
    """
    ### DroneKit related exceptions.

    ---

    Attributes:
        `message`: A string describing the exception.

    ---
    """
    pass
# APIException 



class TimeoutError(APIException):
    """
    ### Raised by operations that have timeouts.
    """
    pass
# TimeoutError



@dataclass
class Attitude:
    """
    ### Vehicle attitude information.

    An object of this type is returned by `Vehicle.attitude`.

    [Diagram showing Pitch, Roll, Yaw](http://commons.wikimedia.org/wiki/File:Yaw_Axis_Corrected.svg)

    ---

    Attributes:
    
        `pitch`: Pitch in radians 
        `yaw`: Yaw in radians
        `roll`: Roll in radians
    
    ---
    """

    # Member variables
    pitch: float
    yaw: float
    roll: float


    # Member methods
    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"pitch = {self.pitch:.4f}rads, "
                f"roll = {self.roll:.4f}rads, "
                f"yaw = {self.yaw:.4f}rads")
    # __str__
# Attitude



@dataclass
class LocationGlobal:
    """
    ### A global location object.

    The latitude and longitude are relative to the [WGS84 coordinate system](https://en.wikipedia.org/wiki/World_Geodetic_System).

    The altitude is relative to mean sea-level (MSL).

    Example:
    ```python
        # A global location object with altitude 30 metres above sea level
        location = LocationGlobal(-34.364114, 149.166022, 30)
        
        # Accesss and print a vehicle's global relative frame value
        print(vehicle.location.global_frame)
    ```

    An object of this type is owned by `Vehicle.location`. See that class for information on reading and observing location in the global frame.

    ---

    Attributes:
        `lat`: Latitude
        `lon`: Longitude  
        `alt`: Altitude in meters relative to mean sea-level (MSL)

    ---

    TODO: Location class - possibly add a vector3 representation.
    """

    # Member variables
    lat: float
    lon: float
    alt: float | None = None
    # For backward compatibility
    local_frame: Any | None = field(default=None, init=False)
    global_frame: Any | None = field(default=None, init=False)


    # Member methods
    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"lat = {self.lat}, "
                f"lon = {self.lon}, "
                f"alt = {self.alt} meters")
    # __str__
# LocationGlobal



@dataclass
class LocationGlobalRelative:
    """
    ### A global location object with altitude relative to home location.

    The latitude and longitude are relative to the [WGS84 coordinate system](https://en.wikipedia.org/wiki/World_Geodetic_System).
    
    The altitude is relative to the home position. 

    Example:
    ```python
        # A location 30 metres above the home location
        location = LocationGlobalRelative(-34.364114, 149.166022, 30)
        
        # Accesss and print a vehicle's global relative frame value
        print(vehicle.location.global_relative_frame)
    ```

    An object of this type is owned by `Vehicle.location`. See that class for information on reading and observing location in the global-relative frame.

    ---

    Attributes:
        `lat`: Latitude
        `lon`: Longitude
        `alt`: Altitude in meters (relative to the home location)

    ---

    TODO: Location class - possibly add a vector3 representation.
    """

    # Member variables
    lat: float
    lon: float
    alt: float | None = None
    # For backward compatibility
    local_frame: Any | None = field(default=None, init=False)
    global_frame: Any | None = field(default=None, init=False)


    # Member methods
    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"lat = {self.lat}, "
                f"lon = {self.lon}, "
                f"alt = {self.alt} meters")
    # __str__
# LocationGlobalRelative



@dataclass
class LocationLocal:
    """
    ### A local location object.

    The north, east and down are relative to the EKF origin. This is most likely the location where the vehicle was turned on.

    An object of this type is owned by `Vehicle.location`. See that class for information on reading and observing location in the local frame.

    ---

    Attributes:
        `north`: Position north of the EKF origin in meters
        `east`: Position east of the EKF origin in meters
        `down`: Position down from the EKF origin in meters (i.e. negative altitude in meters)

    ---
    """

    # Member variables
    north: float
    east: float
    down: float


    # Member methods
    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"north = {self.north}, "
                f"east = {self.east}, "
                f"down = {self.down}")
    # __str__


    def distance_home(self) -> float | None:
        """
        ### Distance away from home in meters.
        
        The distance will be in 3D if `down` is known, otherwise 2D.
        
        ---
        
        Returns:
            Distance in meters, or None if position is not known
            
        ---
        """
        if self.north is not None and self.east is not None:
            if self.down is not None:
                return math.sqrt((self.north**2) + (self.east**2) + (self.down**2))
            else:
                return math.sqrt((self.north**2) + (self.east**2))
        return None
    # distance_home
# LocationLocal



@dataclass
class GPSInfo:
    """
    ### Standard information about GPS.

    If there is no GPS lock the parameters are set to `None`.

    ---

    Attributes:
        `eph`: GPS horizontal dilution of position (HDOP)
        `epv`: GPS vertical dilution of position (VDOP)
        `fix_type`: 0-1: no fix, 2: 2D fix, 3: 3D fix
        `satellites_visible`: Number of satellites visible

    ---

    TODO: GPSInfo class - possibly normalize eph/epv?  report fix type as string?
    """

    # Member variables
    eph: int | None
    epv: int | None
    fix_type: int | None
    satellites_visible: int | None


    # Member methods
    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"fix_type = {self.fix_type}, "
                f"satellites_visible = {self.satellites_visible}")
    # __str__
# GPSInfo



@dataclass
class Wind:
    """
    ### Wind information.

    An object of this type is returned by `Vehicle.wind`.

    ---

    Attributes:
        `wind_direction`: Wind direction in degrees
        `wind_speed`: Wind speed in m/s
        `wind_speed_z`: Vertical wind speed in m/s

    ---
    """

    # Member variables
    wind_direction: float
    wind_speed: float
    wind_speed_z: float
    

    # Member methods
    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"direction = {self.wind_direction}°, "
                f"speed = {self.wind_speed}m/s, "
                f"vertical_speed = {self.wind_speed_z}m/s")
    # __str__
# Wind



class Battery:
    """
    ### System battery information.

    An object of this type is returned by `Vehicle.battery`.

    ---

    Attributes:
        `voltage`: Battery voltage in millivolts
        `current`: Battery current in 10 * milliamperes. `None` if the autopilot does not support current measurement
        `level`: Remaining battery energy. `None` if the autopilot cannot estimate the remaining battery

    ---
    """

    # Member variables
    def __init__(self, voltage: int, current: int, level: int) -> None:
        self.voltage: float = (voltage / 1000.0)
        self.current: float | None = None if current == -1 else (current / 100.0)
        self.level: int | None = None if level == -1 else level
    # __init__


    # Member methods
    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"voltage = {self.voltage}mV, "
                f"current = {self.current}mA, "
                f"level = {self.level}%")
    # __str__
# Battery



@dataclass
class Rangefinder:
    """
    ### Rangefinder readings.

    An object of this type is returned by `Vehicle.rangefinder`.

    ---

    Attributes:
        `distance`: Distance in meters. `None` if the vehicle doesn't have a rangefinder
        `voltage`: Voltage in volts. `None` if the vehicle doesn't have a rangefinder

    ---
    """

    # Member variables
    distance: float | None
    voltage: float | None
    

    # Member methods
    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"distance = {self.distance}m, "
                f"voltage = {self.voltage}V")
    # __str__
# Rangefinder



@dataclass
class Version:
    """
    ### Autopilot version and type.

    An object of this type is returned by `Vehicle.version`.

    The version number can be read in a human-readable format by printing the object.
    This might print something like "APM:Copter-3.3.2-rc4".

    ---

    Attributes:
        `major`: Major version number
        `minor`: Minor version number
        `patch`: Patch version number
        `release`: Release type. See [FIRMWARE_VERSION_TYPE enum](https://mavlink.io/en/messages/common.html#FIRMWARE_VERSION_TYPE_DEV)
        `raw_version`: Raw version data from autopilot
        `autopilot_type`: The autopilot type (e.g., ArduPilot, PX4)
        `vehicle_type`: The vehicle type (e.g., copter, plane, rover)

    ---
    """


    # Member variables
    raw_version: int | None
    autopilot_type: int
    vehicle_type: int
    major: int | None = field(init=False)
    minor: int | None = field(init=False)
    patch: int | None = field(init=False)
    release: int | None = field(init=False)


    # Member methods
    def __post_init__(self) -> None:
        if self.raw_version is None:
            self.major = None
            self.minor = None
            self.patch = None
            self.release = None
        else:
            self.major = self.raw_version >> 24 & 0xFF
            self.minor = self.raw_version >> 16 & 0xFF
            self.patch = self.raw_version >> 8 & 0xFF
            self.release = self.raw_version & 0xFF
    # __post_init__


    def is_stable(self) -> bool:
        """
        ### Check for a firmware stable release.
        
        ---

        Returns:
            `True` if the autopilot reports that the current firmware is an official stable release (not a pre-release or development version).

        ---
        """
        return self.release == 255
    # is_stable


    def release_version(self) -> int | None:
        """
        ### Get version within the release type.
        
        ---

        Returns:
            The version within the release type (an integer).
            For example, returns "23" for Copter-3.3rc23.

        ---
        """
        if self.release is None:
            return None
        if self.release == 255:
            return 0
        return self.release % 64
    # release_version


    def release_type(self) -> str | None:
        """
        ### Get release type description.
        
        ---

        Returns:
            Text describing the release type e.g. "alpha", "stable" etc.

        ---
        """
        if self.release is None:
            return None
        types = ["dev", "alpha", "beta", "rc"]
        return types[self.release >> 6]
    # release_type


    def __str__(self) -> str:
        items = []

        if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            items.append("APM:")
        elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            items.append("PX4")
        else:
            items.append("UnknownAutoPilot")

        if self.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
            items.append("Copter-")
        elif self.vehicle_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            items.append("Plane-")
        elif self.vehicle_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            items.append("Rover-")
        else:
            items.append(f"UnknownVehicleType{self.vehicle_type}-")

        prefix = "".join(items)

        if self.release_type() is None:
            release_type = "UnknownReleaseType"
        elif self.is_stable():
            release_type = ""
        else:
            release_type = f"-{self.release_type()}{self.release_version()}"

        return (f"{self.__class__.__name__}: "
                f"{prefix}{self.major}.{self.minor}.{self.patch}{release_type}")
    # __str__
# Version



class Capabilities:
    """
    ### Autopilot capabilities (supported message types and functionality).

    An object of this type is returned by `Vehicle.capabilities`.

    Each capability is a boolean value that indicates whether the autopilot supports the corresponding message type or functionality.

    See the enum [MAV_PROTOCOL_CAPABILITY](https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT).

    ---

    Attributes:
        `mission_float`: Autopilot supports MISSION float message type 
        `param_float`: Autopilot supports the PARAM float message type
        `mission_int`: Autopilot supports MISSION_INT scaled integer message type
        `command_int`: Autopilot supports COMMAND_INT scaled integer message type
        `param_union`: Autopilot supports the PARAM_UNION message type
        `ftp`: Autopilot supports ftp for file transfers
        `set_attitude_target`: Autopilot supports commanding attitude offboard
        `set_attitude_target_local_ned`: Autopilot supports commanding position and velocity targets in local NED frame
        `set_altitude_target_global_int`: Autopilot supports commanding position and velocity targets in global scaled integers
        `terrain`: Autopilot supports terrain protocol / data handling
        `set_actuator_target`: Autopilot supports direct actuator control
        `flight_termination`: Autopilot supports the flight termination command
        `compass_calibration`: Autopilot supports onboard compass calibration

    ---
    """

    def __init__(self, capabilities: int) -> None:
        self.mission_float: bool                    = (((capabilities >> 0) & 1) == 1)
        self.param_float: bool                      = (((capabilities >> 1) & 1) == 1)
        self.mission_int: bool                      = (((capabilities >> 2) & 1) == 1)
        self.command_int: bool                      = (((capabilities >> 3) & 1) == 1)
        self.param_union: bool                      = (((capabilities >> 4) & 1) == 1)
        self.ftp: bool                              = (((capabilities >> 5) & 1) == 1)
        self.set_attitude_target: bool              = (((capabilities >> 6) & 1) == 1)
        self.set_attitude_target_local_ned: bool    = (((capabilities >> 7) & 1) == 1)
        self.set_altitude_target_global_int: bool   = (((capabilities >> 8) & 1) == 1)
        self.terrain: bool                          = (((capabilities >> 9) & 1) == 1)
        self.set_actuator_target: bool              = (((capabilities >> 10) & 1) == 1)
        self.flight_termination: bool               = (((capabilities >> 11) & 1) == 1)
        self.compass_calibration: bool              = (((capabilities >> 12) & 1) == 1)
    # __init__
# Capabilities



@dataclass
class VehicleMode:
    """
    ### Used to get and set the current vehicle flight mode.

    The flight mode determines the behavior of the vehicle and what commands it can obey.
    The recommended flight modes for DroneKit-Python apps depend on the vehicle type:

    - Copter apps should use `AUTO` mode for normal waypoint missions and `GUIDED` mode otherwise.
    - Plane and Rover apps should use the `AUTO` mode in all cases, re-writing the mission commands if dynamic behavior is required (they support only a limited subset of commands in `GUIDED` mode).
    - Some modes like `RETURN_TO_LAUNCH` can be used on all platforms. Care should be taken when using manual modes as these may require remote control input from the user. 

    The available set of supported modes is vehicle-specific, e.g., see:
    - [Copter Modes](https://ardupilot.org/copter/docs/flight-modes.html)
    - [Plane Modes](https://ardupilot.org/plane/docs/flight-modes.html)
    - [Rover Modes](https://ardupilot.org/rover/docs/rover-control-modes.html)
    
    If an unsupported mode is set the script will raise a `KeyError` exception.

    The `Vehicle.mode` attribute can be queried for the current mode.
    The code snippet below shows how to observe changes to the mode and then read the value:

    ```python
        # Callback definition for mode observer
        def mode_callback(self, attr_name: str) -> None:
            print(f"Vehicle Mode: {self.mode}")

        # Add observer callback for the mode attribute
        vehicle.add_attribute_listener(attr_name='mode', observer=mode_callback)
    ```

    The code snippet below shows how to change the vehicle mode to AUTO:

    ```python
        # Set the vehicle into auto mode
        vehicle.mode = VehicleMode('AUTO')
    ```

    For more information on getting/setting/observing the `Vehicle.mode` (and other attributes) see the [Vehicle State and Settings Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters).

    ---

    Attributes:
        `name`: The mode name, as a string

    ---
    """
    

    # Member variables
    name: str


    # Member methods
    def __str__(self) -> str:
        return f"{self.__class__.__name__}: {self.name}"
    # __str__


    def __eq__(self, other: str) -> bool:
        return self.name == other
    # __eq__


    def __ne__(self, other: str) -> bool:
        return not self.__eq__(other=other)
    # __ne__
# VehicleMode



@dataclass
class SystemStatus:
    """
    ### Used to get and set the current system status.

    An object of this type is returned by `Vehicle.system_status`.

    ---

    Attributes:
        `state`: The system state, as a string

    ---
    """


    # Member variables
    state: str


    # Member methods
    def __str__(self) -> str:
        return f"{self.__class__.__name__}: {self.state}"
    # __str__


    def __eq__(self, other: str) -> bool:
        return self.state == other
    # __eq__


    def __ne__(self, other: str) -> bool:
        return not self.__eq__(other=other)
    # __ne__
# SystemStatus



# Since observers have the following arg list and do not return anything.
# The Vehicle class type hint is in quotes to forward declare the class.
Observer = Callable[['Vehicle', str, Any], None]

class HasObservers:
    """
    ### For objects that support attribute observation.
    
    Provides methods to add, remove, and notify listeners when attributes change.
    """


    # Member methods
    def __init__(self) -> None:
        logging.basicConfig()
        self._logger: logging.Logger = logging.getLogger(name=__name__)

        # A mapping from attr_name to a list of observers
        self._attribute_listeners: dict[str, list[Observer]] = {}
        self._attribute_cache: dict[str, Any] = {}
    # __init__


    def add_attribute_listener(self, attr_name: str, observer: Observer) -> None:
        """
        ### Add an attribute listener callback.

        - The callback function (`observer`) is invoked differently depending on the type of attribute.
        - Attributes that represent sensor values or which are used to monitor connection status are updated whenever a message is received from the vehicle. 
        - Attributes which reflect vehicle state are only updated when their values change (for example `Vehicle.system_status`, `Vehicle.armed`, and `Vehicle.mode`).

        The callback can be removed using `remove_attribute_listener`.

        ---

        📝
        #### NOTE:

            The `on_attribute` decorator performs the same operation as this method, but with a more elegant syntax. 
            
            Use `add_attribute_listener` by preference if you will need to remove the observer.

        ---

        The argument list for the callback is `observer(object, attr_name, attribute_value)`:

            `self`: the associated `Vehicle`. This may be compared to a global vehicle handle to implement vehicle-specific callback handling (if needed).
            `attr_name`: the attribute name. This can be used to infer which attribute has triggered if the same callback is used for watching several attributes.
            `attribute_value`: the attribute value (so you don't need to re-query the vehicle object).

        ---

        The example below shows how to get callbacks for (global) location changes:
        ```python
            # Callback to print the location in global frame
            def location_callback(self, attr_name, msg):
                print(f"Location (Global): {msg}")

            # Add observer for the vehicle's current location
            vehicle.add_attribute_listener(attr_name='global_frame', observer=location_callback)
        ```

        See [Vehicle State and Settings Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters) for more information.

        Args:
            `attr_name`: The name of the attribute to watch (or `*` to watch all attributes).
            `observer`: The callback to invoke when a change in the attribute is detected.

        """
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is None:  # if this is the first time the attr is being added
            listeners_for_attr = []     # create a list for it
            self._attribute_listeners[attr_name] = listeners_for_attr
        if observer not in listeners_for_attr:  # if the observer callback isnt in the list yet
            listeners_for_attr.append(observer) # add it
    # add_attribute_listener


    def remove_attribute_listener(self, attr_name: str, observer: Observer) -> None:
        """
        ### Remove an attribute listener (observer) that was previously added using `add_attribute_listener`.

        For example, the following line would remove a previously added vehicle 'global_frame'
        observer called `location_callback`:

        ```python
            vehicle.remove_attribute_listener('global_frame', location_callback)
        ```

        See [Vehicle State and Settings Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters) for more information.

        Args:
            `attr_name`: The attribute name that is to have an observer removed (or `*` to remove an 'all attribute' observer).
            `observer`: The callback function to remove.

        """
        listeners_for_attr = self._attribute_listeners.get(attr_name)
        if listeners_for_attr is not None:
            listeners_for_attr.remove(observer)
            if len(listeners_for_attr) == 0:
                del self._attribute_listeners[attr_name]
    # remove_attribute_listener


    def notify_attribute_listeners(self, attr_name: str, value: Any, cache: bool = False) -> None:
        """
        ### This method is used to update attribute observers when the named attribute is updated.

        You should call it in your message listeners after updating an attribute with information from a vehicle message.

        By default the value of `cache` is `False` and every update from the vehicle is sent to listeners (whether or not the attribute has changed). This is appropriate for attributes which represent sensor or heartbeat-type monitoring.

        Set `cache=True` to update listeners only when the value actually changes (cache the previous attribute value). This should be used where clients will only ever need to know the value when it has changed. For example, this setting has been used for notifying `mode` changes.

        See [Create Attribute in App](https://dronekit.netlify.app/examples/create_attribute) for more information.

        Args:
            `attr_name`: The name of the attribute that has been updated.
            `value`: The current value of the attribute that has been updated.
            `cache`: Set `True` to only notify observers when the attribute value changes.
        """
        # Cached values are not re-sent if they are unchanged.
        if cache:
            if self._attribute_cache.get(attr_name) == value:
                return
            # O/w, if changed, update
            self._attribute_cache[attr_name] = value

        # Notify observers.
        for fn in self._attribute_listeners.get(attr_name, []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)

        for fn in self._attribute_listeners.get('*', []):
            try:
                fn(self, attr_name, value)
            except Exception:
                self._logger.exception('Exception in attribute handler for %s' % attr_name, exc_info=True)
    # notify_attribute_listeners


    def on_attribute(self, name: str) -> Callable[[Observer], None]:
        """
        ### Decorator for attribute listeners.

        - The decorated function (`observer`) is invoked differently depending on the type of attribute.
        - Attributes that represent sensor values or which are used to monitor connection status are updated whenever a message is received from the vehicle. 
        - Attributes which reflect vehicle "state" are only updated when their values change (for example `Vehicle.system_status`, `Vehicle.armed`, and `Vehicle.mode`).

        ---
        
        📝
        #### NOTE:

            There is no way to remove an attribute listener added with this decorator. 
            
            Use `add_attribute_listener` if you need to be able to remove the attribute listener with `remove_attribute_listener`.

        ---

        The argument list for the callback is `observer(object, attr_name, msg)`:

            `self`: the associated `Vehicle`. This may be compared to a global vehicle handle to implement vehicle-specific callback handling (if needed).
            `attr_name`: the attribute name. This can be used to infer which attribute has triggered if the same callback is used for watching several attributes.
            `msg`: the attribute value (so you don't need to re-query the vehicle object).

        ---

        The code fragment below shows how you can create a listener for the attitude attribute.

        ```python
            @vehicle.on_attribute('attitude')
            def attitude_listener(self, name, msg):
                print(f'{name} attribute is: {msg}')
        ```

        See [Vehicle State and Settings Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters) for more information.

        ---

        Args:
            `name`: The name of the attribute to watch (or `*` to watch all attributes)
            `observer`: The callback to invoke when a change in the attribute is detected.

        ---
        """

        def decorator(fn: Observer) -> None:
            if isinstance(name, list):
                for n in name:
                    self.add_attribute_listener(n, fn)
            else:
                self.add_attribute_listener(name, fn)

        return decorator
    # on_attribute
# HasObservers



class ChannelsOverride(dict):
    """
    ### A dictionary class for managing Vehicle channel overrides.

    Channels can be read, written, or cleared by index or using a dictionary syntax.
    To clear a value, set it to `None` or use `del` on the item.

    An object of this type is returned by `Vehicle.channels.overrides` ≡ `Channels.overrides`.

    For more information and examples see [Channels and Channel Overrides](https://dronekit.netlify.app/examples/channel_overrides).

    ---

    Attributes:
        `_vehicle`: The associated `Vehicle`.
        `_count`: The number of channels defined in the dictionary (currently 8).
        `_active`: Whether the channel overrides are active.

    ---
    """

    def __init__(self, vehicle: 'Vehicle') -> None:
        self._vehicle = vehicle
        self._count = 8  # Fixed by MAVLink
        self._active = True
    # __init__


    def __getitem__(self, key: str) -> Any:
        return dict.__getitem__(self, str(key))
    # __getitem__


    def __setitem__(self, key: str, value: Any) -> None:
        # This is ArduPilot's channels numerical values 1-8
        if not (0 < int(key) <= self._count):
            raise KeyError('Invalid channel index %s' % key)
        if not value:
            try:
                dict.__delitem__(self, str(key))
            except:
                pass
        else:
            dict.__setitem__(self, str(key), value)
        self._send()
    # __setitem__


    def __delitem__(self, key: str) -> None:
        dict.__delitem__(self, str(key))
        self._send()
    # __delitem__


    def __len__(self) -> int:
        return self._count
    # __len__


    def _send(self) -> None:
        if self._active:
            overrides = [0] * 8
            for k, v in self.items():
                overrides[int(k) - 1] = v
            self._vehicle._master.mav.rc_channels_override_send(0, 0, *overrides)
    # _send
# ChannelsOverride



class Channels(dict):
    """
    ### A dictionary class for managing RC channel information associated with a Vehicle.

    An object of this type is accessed through the `Vehicle.channels` attribute. 
    This object also stores the current vehicle channel overrides through its `overrides` attribute.

    For more information and examples see [Channels and Channel Overrides](https://dronekit.netlify.app/examples/channel_overrides).

    ---

    Attributes:
        `count`: The number of channels defined in the dictionary.
        `overrides`: The channel overrides dictionary.

    ---
    """

    def __init__(self, vehicle: 'Vehicle', count: int) -> None:
        self._vehicle = vehicle
        self._count = count
        # Since the ChannelsOverride class is derived from the dict class,
        # this initialization creates our dictionary automatically!
        self._overrides = ChannelsOverride(vehicle)

        # Populate readback
        self._readonly = False
        for k in range(count):
            self[k + 1] = None
        self._readonly = True
    # __init__


    @property
    def count(self) -> int:
        """
        ### The number of channels defined in the dictionary (currently 8).
        """
        return self._count
    # count


    def __getitem__(self, key: str) -> Any:
        return dict.__getitem__(self, str(key))
    # __getitem__


    def __setitem__(self, key: str, value: Any) -> None:
        if self._readonly:
            raise TypeError('__setitem__ is not supported on Channels object')
        return dict.__setitem__(self, str(key), value)
    # __setitem__


    def __len__(self) -> int:
        return self._count
    # __len__


    def _update_channel(self, channel: int, value: Any) -> None:
        # If we have channels on different ports, we expand the Channels
        # object to support them.
        channel = int(channel)
        self._readonly = False
        self[channel] = value
        self._readonly = True
        self._count = max(self._count, channel)
    # _update_channel


    @property
    def overrides(self) -> ChannelsOverride:
        """
        ### Channel overrides dictionary.

        Read, set, and clear channel overrides (also known as "rc overrides") associated with a `Vehicle` (via `Vehicle.channels`). 
        
        This is an object of type `ChannelsOverride`.
        
        For more information and examples see [Channels and Channel Overrides](https://dronekit.netlify.app/examples/channel_overrides).

        Examples:

        ```python
            # Set and clear overrides using dictionary syntax (clear by setting override to none)
            vehicle.channels.overrides = {'5':None, '6':None,'3':500}

            # You can also set and clear overrides using indexing syntax
            vehicle.channels.overrides['2'] = 200
            vehicle.channels.overrides['2'] = None

            # Clear using 'del'
            del vehicle.channels.overrides['3']

            # Clear all overrides by setting an empty dictionary
            vehicle.channels.overrides = {}
        ```

        Read the channel overrides either as a dictionary or by index. 
        
        ---
        
        📝
        #### NOTE: 
         
            You'll get a `KeyError` exception if you read a channel override that has not been set.

        ---

        ```python
            # Get all channel overrides
            print(f"Channel overrides: {vehicle.channels.overrides}")
            # Print just one channel override
            print(f"Ch2 override: {vehicle.channels.overrides['2']}")
        ```
        """
        return self._overrides
    # overrides


    @overrides.setter
    def overrides(self, newch: dict[str, Any]) -> None:
        self._overrides._active = False
        self._overrides.clear()
        for k, v in newch.items():
            if v:
                self._overrides[str(k)] = v
            else:
                try:
                    del self._overrides[str(k)]
                except:
                    pass
        self._overrides._active = True
        self._overrides._send()
    # overrides.setter
# Channels



class Locations(HasObservers):
    """
    ### An object for holding location information in global, global relative, and local frames.

    Sets up listeners for the global and local position messages.

    `Vehicle` owns an object of this type. See `Vehicle.location` for information on reading and observing location in the different frames.

    The different frames are accessed through the members, which are created with this object.
    They can be read, and are observable.

    ---

    Attributes:
        `_lat`: The latitude of the location.
        `_lon`: The longitude of the location.
        `_alt`: The altitude of the location.
        `_relative_alt`: The relative altitude of the location.
        `_north`: The north position of the location.
        `_east`: The east position of the location.
        `_down`: The down position of the location.

    ---
    """

    def __init__(self, vehicle: 'Vehicle') -> None:
        super(Locations, self).__init__()

        self._lat: float | None = None
        self._lon: float | None = None
        self._alt: float | None = None
        self._relative_alt: float | None = None

        @vehicle.on_message(name='GLOBAL_POSITION_INT')
        def listener(vehicle: 'Vehicle', name: str, m: Any) -> None:
            (self._lat, self._lon) = ((m.lat / 1.0e7), (m.lon / 1.0e7))
            self._relative_alt = (m.relative_alt / 1000.0)

            self.notify_attribute_listeners(
                attr_name='global_relative_frame', 
                value=self.global_relative_frame
            )
            vehicle.notify_attribute_listeners(
                attr_name='location.global_relative_frame', 
                value=vehicle.location.global_relative_frame
            )

            if self._alt is not None or m.alt != 0:
                # Require first alt value to be non-0
                # TODO is this the proper check to do?
                self._alt = (m.alt / 1000.0)
                self.notify_attribute_listeners(
                    attr_name='global_frame', 
                    value=self.global_frame
                )
                vehicle.notify_attribute_listeners(
                    attr_name='location.global_frame', 
                    value=vehicle.location.global_frame
                )

            vehicle.notify_attribute_listeners(attr_name='location', value=vehicle.location)
        # listener


        self._north: float | None = None
        self._east: float | None = None
        self._down: float | None = None

        @vehicle.on_message(name='LOCAL_POSITION_NED')
        def listener(vehicle: 'Vehicle', name: str, m: Any) -> None:
            self._north = m.x
            self._east = m.y
            self._down = m.z
            self.notify_attribute_listeners(
                attr_name='local_frame', 
                value=self.local_frame
            )
            vehicle.notify_attribute_listeners(
                attr_name='location.local_frame', 
                value=vehicle.location.local_frame
            )
            vehicle.notify_attribute_listeners(
                attr_name='location', 
                value=vehicle.location
            )
        # listener
    # __init__


    @property
    def local_frame(self) -> LocationLocal:
        """
        ### Location in the local NED frame (a `LocationLocal` object).

        This is accessed through the `Vehicle.location` attribute:

        ```python
            print(f"Local Location: {vehicle.location.local_frame}")
        ```

        This location will not start to update until the vehicle is armed.
        """
        return LocationLocal(self._north, self._east, self._down)
    # local_frame


    @property
    def global_frame(self) -> LocationGlobal:
        """
        ### Location in the global frame (a `LocationGlobal` object).

        The latitude and longitude are relative to the [WGS84 coordinate system](https://en.wikipedia.org/wiki/World_Geodetic_System).

        The altitude is relative to mean sea-level (MSL).

        This is accessed through the `Vehicle.location` attribute, e.g.,:

        ```python
            print(f"Global Location: {vehicle.location.global_frame}")
            print(f"Sea level altitude is: {vehicle.location.global_frame.alt}")
        ```

        Its `lat` and `lon` attributes are populated shortly after GPS becomes available.
        The `alt` can take several seconds longer to populate (from the barometer).
        Listeners are not notified of changes to this attribute until it has fully populated.

        To watch for changes you can use `Vehicle.on_attribute` decorator or
        `add_attribute_listener` (decorator approach shown below):

        ```python
            @vehicle.on_attribute(attr_name='location.global_frame')
            def listener(self, attr_name: str, value: Any) -> None:
                print(f"Global: {value}")

            # Alternatively, use decorator: 
            @vehicle.location.on_attribute(attr_name='global_frame')
        ```
        """
        return LocationGlobal(self._lat, self._lon, self._alt)
    # global_frame


    @property
    def global_relative_frame(self) -> LocationGlobalRelative:
        """
        ### Location in the global frame, with altitude relative to the home location (a `LocationGlobalRelative` object).

        The latitude and longitude are relative to the [WGS84 coordinate system](https://en.wikipedia.org/wiki/World_Geodetic_System).

        The altitude is relative to the home location `Vehicle.home_location`.

        This is accessed through the `Vehicle.location` attribute, e.g.,:

        ```python
            print(f"Global Location (relative altitude): {vehicle.location.global_relative_frame}")
            print(f"Altitude relative to home_location: {vehicle.location.global_relative_frame.alt}")
        ```
        """
        return LocationGlobalRelative(self._lat, self._lon, self._relative_alt)
    # global_relative_frame
# Locations


class Vehicle(HasObservers):
    """
    ### The main vehicle API.

    Vehicle state is exposed through attributes (e.g., `heading`). All attributes can be read, and some are also settable (e.g., `mode`, `armed`, and `home_location`).

    Attributes can also be asynchronously monitored for changes by registering listener callback functions.

    Vehicle settings (i.e., parameters like `AUTOTUNE_AXES` and `PLND_ENABLED`) are read/set using the `parameters` attribute.
    Parameters can be iterated and are also individually observable.

    Vehicle movement is primarily controlled using the `armed` attribute and the `simple_takeoff` and `simple_goto` methods in `GUIDED` mode.

    It is also possible to work with vehicle missions, when in `AUTO` mode, using the `commands` attribute.

    `STATUSTEXT` log messages from the autopilot are handled through a separate logger. It is possible to configure the log level, formatting, etc. by accessing the logger, e.g.,:

    ```python
        import logging
        autopilot_logger = logging.getLogger('autopilot')
        autopilot_logger.setLevel(logging.DEBUG)
    ```

    The guide contains more detailed information on the different ways you can use the `Vehicle` class, e.g.,:

    - [Vehicle State and Parameters Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters)
    - [Copter Guided Mode Guide](https://dronekit.netlify.app/guide/copter/guided_mode)
    - [Auto Mode Missions Guide](https://dronekit.netlify.app/guide/auto_mode)

    ---

    📝
    #### NOTE:
        
        This class currently exposes just the attributes that are most commonly used by all vehicle types. If you need to add additional attributes then subclass `Vehicle` as demonstrated in the [Create Attribute in App](https://dronekit.netlify.app/examples/create_attribute) example. 
        ∴ please then [contribute](https://dronekit.netlify.app/contributing/contributions_api) your additions back to the project!

    ---
    """
    def __init__(self, handler: Any) -> None:
        super(Vehicle, self).__init__()

        # Logger for DroneKit
        self._logger = logging.getLogger(__name__)
        
        # Logger for the autopilot messages
        self._autopilot_logger = logging.getLogger('autopilot')

        # MAVLink-to-logging-module log severity mappings
        self._mavlink_statustext_severity = {
            0: logging.CRITICAL,
            1: logging.CRITICAL,
            2: logging.CRITICAL,
            3: logging.ERROR,
            4: logging.WARNING,
            5: logging.INFO,
            6: logging.INFO,
            7: logging.DEBUG
        }

        self._handler = handler
        self._master = handler.master

        # Cache all updated attributes for wait_ready.
        # By default, we presume all "commands" are loaded.
        self._ready_attrs: set[str] = {'commands'}

        # Default parameters when calling wait_ready() or wait_ready(True).
        self._default_ready_attrs = ['parameters', 'gps_0', 'armed', 'mode', 'attitude']

        @self.on_attribute('*')
        def listener(_, name: str, value: Any) -> None:
            self._ready_attrs.add(name)

        # Attaches message listeners.
        self._message_listeners: dict[str, list[Callable]] = dict()

        @handler.forward_message
        def listener(_, msg: Any) -> None:
            self.notify_message_listeners(msg.get_type(), msg)


        # Establish listeners for, and initialize, the default attributes.

        # Status attribute
        @self.on_message('STATUSTEXT')
        def statustext_listener(self, name: str, m: Any) -> None:
            # Log the STATUSTEXT on the autopilot logger, with the correct severity
            self._autopilot_logger.log(
                msg=m.text.strip(),
                level=self._mavlink_statustext_severity[m.severity]
            )


        # Wind attribute
        self._wind_direction: float | None = None
        self._wind_speed: float | None = None
        self._wind_speed_z: float | None = None

        @self.on_message('WIND')
        def listener(self, name: str, m: Any) -> None:
            """ WIND {direction : -180.0, speed : 0.0, speed_z : 0.0} """
            self._wind_direction = m.direction
            self._wind_speed = m.speed
            self._wind_speed_z = m.speed_z
            self.notify_attribute_listeners('wind', self.wind)


        # Location attribute
        self._location = Locations(self)
        self._vx: float | None = None
        self._vy: float | None = None
        self._vz: float | None = None

        @self.on_message('GLOBAL_POSITION_INT')
        def listener(self, name: str, m: Any) -> None:
            (self._vx, self._vy, self._vz) = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
            self.notify_attribute_listeners('velocity', self.velocity)


        # Attitude attribute
        self._pitch: float | None = None
        self._yaw: float | None = None
        self._roll: float | None = None
        self._pitchspeed: float | None = None
        self._yawspeed: float | None = None
        self._rollspeed: float | None = None

        @self.on_message('ATTITUDE')
        def listener(self, name: str, m: Any) -> None:
            self._pitch = m.pitch
            self._yaw = m.yaw
            self._roll = m.roll
            self._pitchspeed = m.pitchspeed
            self._yawspeed = m.yawspeed
            self._rollspeed = m.rollspeed
            self.notify_attribute_listeners('attitude', self.attitude)


        # Visual flight rules heads-up display (VFR HUD) attribute
        self._heading: int | None = None
        self._airspeed: float | None = None
        self._groundspeed: float | None = None

        @self.on_message('VFR_HUD')
        def listener(self, name: str, m: Any) -> None:
            self._heading = m.heading
            self.notify_attribute_listeners('heading', self.heading)
            self._airspeed = m.airspeed
            self.notify_attribute_listeners('airspeed', self.airspeed)
            self._groundspeed = m.groundspeed
            self.notify_attribute_listeners('groundspeed', self.groundspeed)


        # NOTE: HERE WE SEE HOW THE OBSERVERS ARE UTILIZED. AN OBSERVER CALLED LISTENER IS CREATED AFTER A RANGEFINDER MESSAGE COMES IN, AND IT JUST UPDATES THE RANGEFINDER MEMBER COMPONENTS. NOTICE HOW THE ADD_ATTRIBUTE_LISTENERS WAS NEVER POPULATED OR CALLED, ONLY NOTIFY IS CALLED. ∴ THIS IS SETUP AS AN INITIALIZER, SO IT SETS OUR VALUES AND MEMBERS ONCE, AND THATS IT. NOW, IF YOU WANT IT TO KEEP REPORTING THE RANGEFINDER VALUES, YOU HAVE TO CREATE AND ADD YOUR OWN!!
        
        # NOTE: ARE THESE PRIVATE VARIABLES STORED OR THROWN AWAY? IF THE FORMER, THEN DONT WE ALWAYS KEEP TWO COPIES OF EVERYTHING, SINCE WE HAVE THE ONES HERE, AND THEN THE ONES IN THE CORRESPONDING CLASS?? 
         
        # Rangefinder attribute
        self._rngfnd_distance: float | None = None
        self._rngfnd_voltage: float | None = None

        @self.on_message('RANGEFINDER')
        def listener(self, name: str, m: Any) -> None:
            self._rngfnd_distance = m.distance
            self._rngfnd_voltage = m.voltage
            # NOTE: NOTIFY THE RANGEFINDER CLASS OF OUR INIT READ.
            self.notify_attribute_listeners('rangefinder', self.rangefinder)


        # Mount (e.g., cam gimbal or antenna tracker) attribute
        self._mount_pitch: float | None = None
        self._mount_yaw: float | None = None
        self._mount_roll: float | None = None

        @self.on_message('MOUNT_STATUS')
        def listener(self, name: str, m: Any) -> None:
            self._mount_pitch = m.pointing_a / 100.0
            self._mount_roll = m.pointing_b / 100.0
            self._mount_yaw = m.pointing_c / 100.0
            self.notify_attribute_listeners('mount', self.mount_status)


        # AutoPilot version attribute
        self._capabilities: int | None = None
        self._raw_version: int | None = None
        self._autopilot_version_msg_count: int = 0

        @self.on_message('AUTOPILOT_VERSION')
        def listener(vehicle: 'Vehicle', name: str, m: Any) -> None:
            self._capabilities = m.capabilities
            self._raw_version = m.flight_sw_version
            self._autopilot_version_msg_count += 1
            if self._capabilities != 0 or self._autopilot_version_msg_count > 5:
                # ArduPilot <3.4 fails to send capabilities correctly
                # straight after boot, and even older versions send
                # this back as always-0.
                vehicle.remove_message_listener('HEARTBEAT', self.send_capabilities_request)
            self.notify_attribute_listeners('autopilot_version', self._raw_version)


        # Gimbal attribute
        self._gimbal = Gimbal(self)


        # RC channels attribute
        # All keys are strings.
        self._channels = Channels(self, 8)

        @self.on_message(['RC_CHANNELS_RAW', 'RC_CHANNELS'])
        def listener(self, name: str, m: Any) -> None:
            def set_rc(chnum: int, v: Any) -> None:
                '''Private utility for handling rc channel messages'''
                # use port to allow ch nums greater than 8
                port = 0 if name == "RC_CHANNELS" else m.port
                self._channels._update_channel(str(port * 8 + chnum), v)

            for i in range(1, (18 if name == "RC_CHANNELS" else 8) + 1):
                set_rc(chnum=i, v=getattr(m, f"chan{i}_raw"))

            self.notify_attribute_listeners('channels', self.channels)


        # System status attribute
        self._voltage: int | None = None
        self._current: int | None = None
        self._level: int | None = None

        @self.on_message('SYS_STATUS')
        def listener(self, name: str, m: Any) -> None:
            self._voltage = m.voltage_battery
            self._current = m.current_battery
            self._level = m.battery_remaining
            self.notify_attribute_listeners('battery', self.battery)


        # GPS attribute
        # TODO: support multiple GPSs per vehicle - possibly by using componentId
        self._eph: int | None = None
        self._epv: int | None = None
        self._satellites_visible: int | None = None
        self._fix_type: int | None = None 

        @self.on_message('GPS_RAW_INT')
        def listener(self, name: str, m: Any) -> None:
            self._eph = m.eph
            self._epv = m.epv
            self._satellites_visible = m.satellites_visible
            self._fix_type = m.fix_type
            self.notify_attribute_listeners('gps_0', self.gps_0)


        # Current Single Waypoint/Mission attribute
        self._current_waypoint: int = 0

        @self.on_message(['WAYPOINT_CURRENT', 'MISSION_CURRENT'])
        def listener(self, name: str, m: Any) -> None:
            self._current_waypoint = m.seq


        # EKF attribute
        self._ekf_poshorizabs: bool = False
        self._ekf_constposmode: bool = False
        self._ekf_predposhorizabs: bool = False

        @self.on_message('EKF_STATUS_REPORT')
        def listener(self, name: str, m: Any) -> None:
            # boolean: EKF's horizontal position (absolute) estimate is good
            self._ekf_poshorizabs = (m.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0
            # boolean: EKF is in constant position mode and does not know it's absolute or relative position
            self._ekf_constposmode = (m.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0
            # boolean: EKF's predicted horizontal position (absolute) estimate is good
            self._ekf_predposhorizabs = (m.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0
            self.notify_attribute_listeners('ekf_ok', self.ekf_ok, cache=True)


        # Heartbeat attribute
        self._flightmode: str = 'AUTO'
        self._armed: bool = False
        self._system_status: int | None = None
        self._autopilot_type: int | None = None  # PX4, ArduPilot, etc.
        self._vehicle_type: int | None = None  # quadcopter, plane, etc.

        @self.on_message('HEARTBEAT')
        def listener(self, name: str, m: Any) -> None:
            # ignore groundstations
            if m.type == mavutil.mavlink.MAV_TYPE_GCS or (not self._handler.master.probably_vehicle_heartbeat(m)):
                return
            self._armed = (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.notify_attribute_listeners('armed', self.armed, cache=True)

            self._autopilot_type = m.autopilot
            self._vehicle_type = m.type
            if self._is_mode_available(m.custom_mode, m.base_mode) is False:
                raise APIException(f"mode ({m.custom_mode}, {m.base_mode}) not available on mavlink definition")
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self._flightmode = mavutil.interpret_px4_mode(m.base_mode, m.custom_mode)
            else:
                self._flightmode = self._mode_mapping_bynumber[m.custom_mode]
            self.notify_attribute_listeners('mode', self.mode, cache=True)

            self._system_status = m.system_status
            self.notify_attribute_listeners('system_status', self.system_status, cache=True)



        # Waypoint/Mission attribute
        self._home_location: LocationGlobal | None = None
        self._wploader = mavwp.MAVWPLoader()
        self._wp_loaded: bool = True
        self._wp_uploaded: list[bool] | None = None
        self._wpts_dirty: bool = False
        self._commands = CommandSequence(self)

        @self.on_message(['WAYPOINT_COUNT', 'MISSION_COUNT'])
        def listener(self, name: str, msg: Any) -> None:
            if not self._wp_loaded:
                self._wploader.clear()
                self._wploader.expected_count = msg.count
                self._master.waypoint_request_send(0)

        @self.on_message(['HOME_POSITION'])
        def listener(self, name: str, msg: Any) -> None:
            self._home_location = LocationGlobal(
                lat=(msg.latitude / 1.0e7), 
                lon=(msg.longitude / 1.0e7), 
                alt=(msg.altitude / 1000.0)
            )
            self.notify_attribute_listeners('home_location', self.home_location, cache=True)

        @self.on_message(['WAYPOINT', 'MISSION_ITEM'])
        def listener(self, name: str, msg: Any) -> None:
            if not self._wp_loaded:
                if msg.seq == 0:
                    if not (msg.x == 0 and msg.y == 0 and msg.z == 0):
                        self._home_location = LocationGlobal(msg.x, msg.y, msg.z)

                if msg.seq > self._wploader.count():
                    # Unexpected waypoint
                    pass
                elif msg.seq < self._wploader.count():
                    # Waypoint duplicate
                    pass
                else:
                    self._wploader.add(msg)

                    if msg.seq + 1 < self._wploader.expected_count:
                        self._master.waypoint_request_send(msg.seq + 1)
                    else:
                        self._wp_loaded = True
                        self.notify_attribute_listeners('commands', self.commands)

        # Waypoint send to master
        @self.on_message(['WAYPOINT_REQUEST', 'MISSION_REQUEST'])
        def listener(self, name: str, msg: Any) -> None:
            if self._wp_uploaded is not None:
                wp = self._wploader.wp(msg.seq)
                handler.fix_targets(wp)
                self._master.mav.send(wp)
                self._wp_uploaded[msg.seq] = True

        # TODO: Waypoint loop listeners



        # Parameters attribute
        start_duration = 0.2
        repeat_duration = 1

        self._params_count: int = -1
        self._params_set: list[Any | None] = []
        self._params_loaded: bool = False
        self._params_start: bool = False
        self._params_map: dict[str, float] = {}
        self._params_last: float = monotonic.monotonic()  # Last new param.
        self._params_duration: float = start_duration
        self._parameters = Parameters(self)

        @handler.forward_loop
        def listener(_) -> None:
            # Check the time duration for last "new" params exceeds watchdog.
            if not self._params_start:
                return

            if not self._params_loaded and all(x is not None for x in self._params_set):
                self._params_loaded = True
                self.notify_attribute_listeners('parameters', self.parameters)

            if not self._params_loaded and ((monotonic.monotonic() - self._params_last) > self._params_duration):
                c = 0
                for i, v in enumerate(self._params_set):
                    if v is None:
                        self._master.mav.param_request_read_send(0, 0, b'', i)
                        c += 1
                        if c > 50:
                            break
                self._params_duration = repeat_duration
                self._params_last = monotonic.monotonic()

        @self.on_message(['PARAM_VALUE'])
        def listener(self, name: str, msg: Any) -> None:
            # If we discover a new param count, assume we
            # are receiving a new param set.
            if self._params_count != msg.param_count:
                self._params_loaded = False
                self._params_start = True
                self._params_count = msg.param_count
                self._params_set = [None] * msg.param_count

            # Attempt to set the params. We throw an error
            # if the index is out of range of the count or
            # we lack a param_id.
            try:
                if msg.param_index < msg.param_count and msg:
                    if self._params_set[msg.param_index] is None:
                        self._params_last = monotonic.monotonic()
                        self._params_duration = start_duration
                    self._params_set[msg.param_index] = msg

                self._params_map[msg.param_id] = msg.param_value
                self._parameters.notify_attribute_listeners(
                    msg.param_id, 
                    msg.param_value,
                    cache=True
                )
            except:
                import traceback
                traceback.print_exc()



        # Heartbeats.

        self._heartbeat_started: bool = False
        self._heartbeat_lastsent: float = 0
        self._heartbeat_lastreceived: float = 0
        self._heartbeat_timeout: bool = False

        self._heartbeat_warning: int = 5
        self._heartbeat_error: int = 30
        self._heartbeat_system: int | None = None

        @handler.forward_loop
        def listener(_) -> None:
            # Send 1 heartbeat per second
            if monotonic.monotonic() - self._heartbeat_lastsent > 1:
                self._master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
                    0, 0, 0
                )
                self._heartbeat_lastsent = monotonic.monotonic()

            # Timeouts.
            if self._heartbeat_started:
                if self._heartbeat_error and monotonic.monotonic() - self._heartbeat_lastreceived > self._heartbeat_error > 0:
                    raise APIException(f'No heartbeat in {self._heartbeat_error} seconds, aborting.')
                elif monotonic.monotonic() - self._heartbeat_lastreceived > self._heartbeat_warning:
                    if self._heartbeat_timeout is False:
                        self._logger.warning(f'Link timeout, no heartbeat in last {self._heartbeat_warning} seconds')
                        self._heartbeat_timeout = True

        @self.on_message(['HEARTBEAT'])
        def listener(self, name: str, msg: Any) -> None:
            # ignore groundstations
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS or (not self._handler.master.probably_vehicle_heartbeat(msg)):
                return
            self._heartbeat_system = msg.get_srcSystem()
            self._heartbeat_lastreceived = monotonic.monotonic()
            if self._heartbeat_timeout:
                self._logger.info('...link restored.')
            self._heartbeat_timeout = False

        self._last_heartbeat: float | None = None

        @handler.forward_loop
        def listener(_) -> None:
            if self._heartbeat_lastreceived:
                self._last_heartbeat = monotonic.monotonic() - self._heartbeat_lastreceived
                self.notify_attribute_listeners('last_heartbeat', self.last_heartbeat)
    # __init__


    @property
    def last_heartbeat(self) -> float | None:
        """
        ### Time since last MAVLink heartbeat was received (in seconds).

        The attribute can be used to monitor link activity and implement script-specific timeout handling.

        For example, to pause the script if no heartbeat is received for more than 1 second you might implement
        the following observer, and use `pause_script` in a program loop to wait until the link is recovered:

        ```python
            # For stopping the script and waiting for the link to be restored.
            pause_script=False
            @vehicle.on_attribute('last_heartbeat')
            def listener(self, attr_name, value):
                global pause_script
                if value > 1 and not pause_script:
                    print("Pausing script due to bad link")
                    pause_script=True;
                if value < 1 and pause_script:
                    pause_script=False;
                    print("Un-pausing script")
        ```

        The observer will be called at the period of the messaging loop (about every 0.01 seconds). Testing
        on SITL indicates that `last_heartbeat` averages about .5 seconds, but will rarely exceed 1.5 seconds
        when connected. Whether heartbeat monitoring can be useful will very much depend on the application.

        ---

        📝
        #### NOTE:

            If you just want to change the heartbeat timeout you can modify the `heartbeat_timeout`
            parameter passed to the `connect()` function.

        ---
        """
        return self._last_heartbeat
    # last_heartbeat


    def on_message(self, name: str | list[str]) -> Callable[[Callable], None]:
        """
        ### Decorator for message listener callback functions.

        ---

        💡
        #### TIP:

            This is the most elegant way to define message listener callback functions.
            Use `add_message_listener` if, and only if, you need to be able to
            remove the listener (e.g., `remove_message_listener`) later.

        ---

        A decorated message listener function is called with three arguments every time the
        specified message is received:

        * `self` - the current vehicle.
        * `name` - the name of the message that was intercepted.
        * `message` - the actual message ([a pymavlink class](https://www.samba.org/tridge/UAV/pymavlink/apidocs/mavlink.MAVLink_message.html)).

        For example, in the fragment below `my_method` will be called for every heartbeat message:

        ```python
            @vehicle.on_message('HEARTBEAT')
            def my_method(self, name, msg):
                pass
        ```

        See [MAVLink Messages](https://dronekit.netlify.app/guide/mavlink_messages) for more information.

        ---

        Args:
            `name`: The name of the message to be intercepted by the decorated listener function (or `*` to get all messages).

        ---
        """

        def decorator(fn: Callable) -> None:
            if isinstance(name, list):
                for n in name:
                    self.add_message_listener(n, fn)
            else:
                self.add_message_listener(name, fn)

        return decorator
    # on_message


    def add_message_listener(self, name: str, fn: Callable) -> None:
        """
        ### Adds a message listener function that will be called every time the specified message is received.

        ---

        💡
        #### TIP:

            We recommend you use `on_message` instead of this method as it has a more elegant syntax.
            This method is only preferred if you need to be able to
            remove the listener (e.g., `remove_message_listener`).

        ---

        The callback function must have three arguments:

        * `self` - the current vehicle.
        * `name` - the name of the message that was intercepted.
        * `message` - the actual message ([a pymavlink class](https://www.samba.org/tridge/UAV/pymavlink/apidocs/mavlink.MAVLink_message.html)).

        For example, in the fragment below `my_method` will be called for every heartbeat message:

        ```python
            #Callback method for new messages
            def my_method(self, name, msg):
                pass

            vehicle.add_message_listener('HEARTBEAT', my_method)
        ```

        See [MAVLink Messages](https://dronekit.netlify.app/guide/mavlink_messages) for more information.

        ---

        Args:
            `name`: The name of the message to be intercepted by the listener function (or `*` to get all messages).
            `fn`: The listener function that will be called if a message is received.

        ---
        """
        name = str(name)
        if name not in self._message_listeners:
            self._message_listeners[name] = []
        if fn not in self._message_listeners[name]:
            self._message_listeners[name].append(fn)
    # add_message_listener


    def remove_message_listener(self, name: str, fn: Callable) -> None:
        """
        ### Removes a message listener (that was previously added using `add_message_listener`).

        See [MAVLink Messages](https://dronekit.netlify.app/guide/mavlink_messages) for more information.

        ---

        Args:
            `name`: The name of the message for which the listener is to be removed (or `*` to remove an 'all messages' observer).
            `fn`: The listener callback function to remove.

        ---
        """
        name = str(name)
        if name in self._message_listeners:
            if fn in self._message_listeners[name]:
                self._message_listeners[name].remove(fn)
                if len(self._message_listeners[name]) == 0:
                    del self._message_listeners[name]
    # remove_message_listener


    def notify_message_listeners(self, name: str, msg: Any) -> None:
        """
        ### Notify all registered message listeners.

        ---

        Args:
            `name`: The name of the message type
            `msg`: The message object to pass to listeners

        ---
        """
        for fn in self._message_listeners.get(name, []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception(f'Exception in message handler for {msg.get_type()}', exc_info=True)

        for fn in self._message_listeners.get('*', []):
            try:
                fn(self, name, msg)
            except Exception:
                self._logger.exception(f'Exception in message handler for {msg.get_type()}', exc_info=True)
    # notify_message_listeners


    def close(self) -> None:
        """
        ### Close the connection to the vehicle.
        """
        return self._handler.close()
    # close


    def flush(self) -> None:
        """
        ### Call `flush()` after adding or clearing mission commands.

        After the return from `flush()` any writes are guaranteed to have completed (or thrown an
        exception) and future reads will see their effects.

        ---

        ⚠️
        #### WARNING:

            This method is deprecated. It has been replaced by
            `Vehicle.commands.upload()`.

        ---
        """
        return self.commands.upload()
    # flush


    #
    # Private sugar methods
    #

    @property
    def _mode_mapping(self) -> dict[str, int]:
        return self._master.mode_mapping()
    # _mode_mapping


    @property
    def _mode_mapping_bynumber(self) -> dict[int, str]:
        return mavutil.mode_mapping_bynumber(self._vehicle_type)
    # _mode_mapping_bynumber


    def _is_mode_available(self, custommode_code: int, basemode_code: int = 0) -> bool:
        try:
            if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                mode = mavutil.interpret_px4_mode(basemode_code, custommode_code)
                return mode in self._mode_mapping
            return custommode_code in self._mode_mapping_bynumber
        except:
            return False
    # _is_mode_available


    #
    # Operations to support the standard API.
    #

    @property
    def mode(self) -> VehicleMode | None:
        """
        ### Used to get and set the current flight mode.
        
        You can specify the value as a `VehicleMode`, like this:

        ```python
            vehicle.mode = VehicleMode('LOITER')
        ```

        Or as a simple string:

        ```python
            vehicle.mode = 'LOITER'
        ```

        If you are targeting ArduPilot you can also specify the flight mode
        using a numeric value (this will not work with PX4 autopilots):

        ```python
            # set mode to LOITER
            vehicle.mode = 5
        ```
        """
        if not self._flightmode:
            return None
        return VehicleMode(self._flightmode)
    # mode


    @mode.setter
    def mode(self, v: VehicleMode | str | int) -> None:
        if isinstance(v, str):
            v = VehicleMode(name=v)

        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            self._master.set_mode(v.name)
        elif isinstance(v, int):
            self._master.set_mode(v)
        else:
            self._master.set_mode(self._mode_mapping[v.name])
    # mode.setter


    @property
    def location(self) -> Locations:
        """
        ### The vehicle location in global, global relative and local frames (`Locations`).

        The different frames are accessed through its members:

        * `global_frame` (`LocationGlobal`)
        * `global_relative_frame` (`LocationGlobalRelative`)
        * `local_frame` (`LocationLocal`)

        For example, to print the location in each frame for a `vehicle`:

        ```python
            # Print location information for `vehicle` in all frames (default printer)
            print(f"Global Location: {vehicle.location.global_frame}")
            print(f"Global Location (relative altitude): {vehicle.location.global_relative_frame}")
            print(f"Local Location: {vehicle.location.local_frame}")    #NED

            # Print altitudes in the different frames (see class definitions for other available information)
            print(f"Altitude (global frame): {vehicle.location.global_frame.alt}")
            print(f"Altitude (global relative frame): {vehicle.location.global_relative_frame.alt}")
            print(f"Altitude (NED frame): {vehicle.location.local_frame.down}")
        ```

        ---

        📝
        #### NOTE:

            All the location "values" (e.g. `global_frame.lat`) are initially
            created with value `None`. The `global_frame`, `global_relative_frame`
            latitude and longitude values are populated shortly after initialisation but
            `global_frame.alt` may take a few seconds longer to be updated.
            The `local_frame` does not populate until the vehicle is armed.

        ---

        The attribute and its members are observable. To watch for changes in all frames using a listener
        created using a decorator (you can also define a listener and explicitly add it).

        ```python
            @vehicle.on_attribute('location')
            def listener(self, attr_name: str, value: Any) -> None:
                # `self`: `Vehicle` object that has been updated.
                # `attr_name`: name of the observed attribute - 'location'
                # `value` is the updated attribute value (a `Locations`). This can be queried for the frame information
                print(f" Global: {value.global_frame}")
                print(f" GlobalRelative: {value.global_relative_frame}")
                print(f" Local: {value.local_frame}")
        ```

        To watch for changes in just one attribute (in this case `global_frame`):

        ```python
            @vehicle.on_attribute(attr_name='location.global_frame')
            def listener(self, attr_name: str, value: Any) -> None:
            # `self`: `Locations` object that has been updated.
            # `attr_name`: name of the observed attribute - 'global_frame'
            # `value` is the updated attribute value.
            print(f" Global: {value}")

            # Or watch using decorator: 
            @vehicle.location.on_attribute(attr_name='global_frame')
        ```
        """
        return self._location
    # location


    @property
    def wind(self) -> Wind | None:
        """
        ### Current wind status (`Wind`)
        """
        if (self._wind_direction is None or self._wind_speed is None or self._wind_speed_z is None):
            return None
        
        return Wind(
            wind_direction=self._wind_direction, 
            wind_speed=self._wind_speed, 
            wind_speed_z=self._wind_speed_z
        )
    # wind


    @property
    def battery(self) -> Battery | None:
        """
        ### Current system battery status (`Battery`).
        """
        if self._voltage is None or self._current is None or self._level is None:
            return None
        return Battery(self._voltage, self._current, self._level)
    # battery


    @property
    def rangefinder(self) -> Rangefinder:
        """
        ### Rangefinder distance and voltage values (`Rangefinder`).
        """
        # Send the Vehicle class private members to the Rangefinder class
        return Rangefinder(self._rngfnd_distance, self._rngfnd_voltage)
    # rangefinder


    @property
    def velocity(self) -> list[float | None]:
        """
        ### Current velocity as a three element list `[ vx, vy, vz ]` (in meter/sec).
        """
        return [self._vx, self._vy, self._vz]
    # velocity


    @property
    def version(self) -> Version:
        """
        ### The autopilot version and type in a `Version` object.

        .. versionadded:: 2.0.3
        """
        return Version(self._raw_version, self._autopilot_type, self._vehicle_type)
    # version


    @property
    def capabilities(self) -> Capabilities | None:
        """
        ### The autopilot capabilities in a `Capabilities` object.

        .. versionadded:: 2.0.3
        """
        if self._capabilities is None:
            return None
        return Capabilities(self._capabilities)
    # capabilities


    @property
    def attitude(self) -> Attitude | None:
        """
        ### Current vehicle attitude - pitch, yaw, roll (`Attitude`).
        """
        if self._pitch is None or self._yaw is None or self._roll is None:
            return None
        return Attitude(self._pitch, self._yaw, self._roll)
    # attitude


    @property
    def gps_0(self) -> GPSInfo:
        """
        ### GPS position information (`GPSInfo`).
        """
        return GPSInfo(self._eph, self._epv, self._fix_type, self._satellites_visible)
    # gps_0


    @property
    def armed(self) -> bool:
        """
        ### Used to get and set the `armed` state of the vehicle (`boolean`).

        The code below shows how to read the state, and to arm/disarm the vehicle:

        ```python
            # Print the armed state for the vehicle
            print("Armed: %s" % vehicle.armed)

            # Disarm the vehicle
            vehicle.armed = False

            # Arm the vehicle
            vehicle.armed = True
        ```
        """
        return self._armed
    # armed


    @armed.setter
    def armed(self, value: bool) -> None:
        if bool(value) != self._armed:
            if value:
                self._master.arducopter_arm()
            else:
                self._master.arducopter_disarm()
    # armed.setter


    @property
    def is_armable(self) -> bool:
        """
        ### Returns `True` if the vehicle is ready to arm, false otherwise (`boolean`).

        This attribute wraps a number of pre-arm checks, ensuring that the vehicle has booted, has a good GPS fix, and that the EKF pre-arm is complete.
        """
        # check that mode is not INITIALSING
        # check that we have a GPS fix
        # check that EKF pre-arm is complete
        return self.mode != 'INITIALISING' and (self.gps_0.fix_type is not None and self.gps_0.fix_type > 1) and self._ekf_predposhorizabs
    # is_armable


    @property
    def system_status(self) -> SystemStatus | None:
        """
        ### System status (`SystemStatus`).

        The status has a `state` property with one of the following values:

        * `UNINIT`: Uninitialized system, state is unknown.
        * `BOOT`: System is booting up.
        * `CALIBRATING`: System is calibrating and not flight-ready.
        * `STANDBY`: System is grounded and on standby. It can be launched any time.
        * `ACTIVE`: System is active and might be already airborne. Motors are engaged.
        * `CRITICAL`: System is in a non-normal flight mode. It can however still navigate.
        * `EMERGENCY`: System is in a non-normal flight mode. It lost control over parts
          or over the whole airframe. It is in mayday and going down.
        * `POWEROFF`: System just initialized its power-down sequence, will shut down now.
        """
        return {
            0: SystemStatus('UNINIT'),
            1: SystemStatus('BOOT'),
            2: SystemStatus('CALIBRATING'),
            3: SystemStatus('STANDBY'),
            4: SystemStatus('ACTIVE'),
            5: SystemStatus('CRITICAL'),
            6: SystemStatus('EMERGENCY'),
            7: SystemStatus('POWEROFF'),
        }.get(self._system_status, None)
    # system_status


    @property
    def heading(self) -> int | None:
        """
        ### Current heading in degrees ([0°, 360°]) | North ≡ 0° (`int`).
        """
        return self._heading
    # heading


    @property
    def groundspeed(self) -> float | None:
        """
        ### Current groundspeed in meters/second (`float`).

        This attribute is settable. The set value is the default target groundspeed
        when moving the vehicle using `simple_goto` (or other position-based
        movement commands).
        """
        return self._groundspeed
    # groundspeed


    @groundspeed.setter
    def groundspeed(self, speed: float) -> None:
        speed_type = 1  # groundspeed
        msg = self.message_factory.command_long_encode(
            0, 0,                                       # target system, target component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,    # command
            0,                                          # confirmation
            speed_type,                                 # param 1
            speed,                                      # m/s
            -1, 0, 0, 0, 0                              # param 3 - 7
        )

        # send command to vehicle
        self.send_mavlink(msg)
    # groundspeed.setter


    @property
    def airspeed(self) -> float | None:
        """
        ### Current airspeed in meters/second (`float`).

        This attribute is settable. The set value is the default target airspeed
        when moving the vehicle using `simple_goto` (or other position-based
        movement commands).
        """
        return self._airspeed
    # airspeed


    @airspeed.setter
    def airspeed(self, speed: float) -> None:
        speed_type = 0  # airspeed
        msg = self.message_factory.command_long_encode(
            0, 0,                                       # target system, target component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,    # command
            0,                                          # confirmation
            speed_type,                                 # param 1
            speed,                                      # m/s
            -1, 0, 0, 0, 0                              # param 3 - 7
        )

        # send command to vehicle
        self.send_mavlink(msg)
    # airspeed.setter


    @property
    def gimbal(self) -> 'Gimbal':
        """
        ### For controlling, viewing and observing gimbal status.

        .. versionadded:: 2.0.1
        """
        return self._gimbal
    # gimbal


    @property
    def mount_status(self) -> list[float | None]:
        """
        ### Current status of the camera mount (gimbal) as a three element list: `[ pitch, yaw, roll ]`.

        ---

        ⚠️
        #### WARNING:
        
            This method is deprecated. It has been replaced by `gimbal`.

        ---

        The values in the list are set to `None` if no mount is configured.
        """
        return [self._mount_pitch, self._mount_yaw, self._mount_roll]
    # mount_status


    @property
    def ekf_ok(self) -> bool:
        """
        ### `True` if the EKF status is considered acceptable, `False` otherwise (`boolean`).
        """
        # legacy check for dronekit-python for solo
        # use same check that ArduCopter::system.pde::position_ok() is using
        if self.armed:
            return self._ekf_poshorizabs and not self._ekf_constposmode
        else:
            return self._ekf_poshorizabs or self._ekf_predposhorizabs
    # ekf_ok


    @property
    def channels(self) -> Channels:
        """
        ### The RC channel values from the RC Transmitter (`Channels`).

        The attribute can also be used to set and read RC Override (channel override) values
        via `Vehicle.channels.override`.

        For more information and examples see `example_channel_overrides`.

        To read the channels from the RC transmitter:

        ```python
            # Get all channel values from RC transmitter
            print("Channel values from RC Tx:", vehicle.channels)

            # Access channels individually
            print("Read channels individually:")
            print(" Ch1: %s" % vehicle.channels['1'])
            print(" Ch2: %s" % vehicle.channels['2'])
        ```
        """
        return self._channels
    # channels


    @property
    def home_location(self) -> LocationGlobal | None:
        """
        ### The current home location (`LocationGlobal`).

        To get the attribute you must first download the `Vehicle.commands`.
        The attribute has a value of `None` until `Vehicle.commands` has been downloaded
        AND the autopilot has set an initial home location (typically where the vehicle first gets GPS lock).

        ```python
            #Connect to a vehicle object (for example, on com14)
            vehicle = connect('com14', wait_ready=True)

            # Download the vehicle waypoints (commands). Wait until download is complete.
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()

            # Get the home location
            home = vehicle.home_location
        ```

        The `home_location` is not observable.

        The attribute can be written (in the same way as any other attribute) after it has successfully
        been populated from the vehicle. The value sent to the vehicle is cached in the attribute
        (and can potentially get out of date if you don't re-download `Vehicle.commands`):

        ---

        ⚠️
        #### WARNING:

            Setting the value will fail silently if the specified location is more than 50km from the EKF origin.

        ---
        """
        return copy.copy(self._home_location)
    # home_location


    @home_location.setter
    def home_location(self, pos: LocationGlobal) -> None:
        """
        ### Sets the home location (`LocationGlobal`).

        The value cannot be set until it has successfully been read from the vehicle. After being set the value is cached in the home_location attribute and does not have to be re-read.

        ---

        📝
        #### NOTE:

            Setting the value will fail silently if the specified location is more than 50km from the EKF origin.

        ---
        """

        if not isinstance(pos, LocationGlobal):
            raise ValueError('Expecting home_location to be set to a LocationGlobal.')

        # Set cached home location.
        self._home_location = copy.copy(pos)

        # create the MAVLink update message.
        msg = self.message_factory.command_long_encode(
            0, 0,                                   # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,    # command
            0,                                      # confirmation
            0,                                      # param 1: 1 ≡ current position, 0 ≡ entered values.
            0, 0, 0,                                # params 2-4
            pos.lat, pos.lon, pos.alt               # params 5-7
        )

        # send command to vehicle
        self.send_mavlink(msg)
    # home_location.setter


    @property
    def commands(self) -> 'CommandSequence':
        """
        ### Gets the editable waypoints/current mission for this vehicle (`CommandSequence`).

        This can be used to get, create, and modify a mission.

        ---

        Returns:
            A `CommandSequence` containing the waypoints for this vehicle.

        ---
        """
        return self._commands
    # commands


    @property
    def parameters(self) -> 'Parameters':
        """
        ### The (editable) parameters for this vehicle (`Parameters`).
        """
        return self._parameters
    # parameters


    def wait_for(
        self, 
        condition: Callable[[], bool], 
        timeout: float | None = None, 
        interval: float = 0.1, 
        errmsg: str | None = None
    ) -> None:
        """
        ### Wait for a condition to be True.

        Wait for condition, a callable, to return True.  If timeout is
        nonzero, raise a TimeoutError(errmsg) if the condition is not
        True after timeout seconds.  Check the condition every
        interval seconds.

        ---

        Args:
            `condition`: A callable that returns True when the condition is met
            `timeout`: Maximum time to wait in seconds. None means wait forever
            `interval`: How often to check the condition in seconds
            `errmsg`: Error message to include in TimeoutError

        ---
        """

        t0 = time.time()
        while not condition():
            t1 = time.time()
            if timeout and ((t1 - t0) >= timeout):
                raise TimeoutError(errmsg)

            time.sleep(interval)
    # wait_for


    def wait_for_armable(self, timeout: float | None = None) -> None:
        """
        ### Wait for the vehicle to become armable.

        If timeout is nonzero, raise a TimeoutError if the vehicle
        is not armable after timeout seconds.

        ---

        Args:
            `timeout`: Maximum time to wait in seconds. None means wait forever

        ---
        """

        def check_armable() -> bool:
            return self.is_armable

        self.wait_for(check_armable, timeout=timeout)
    # wait_for_armable


    def arm(self, wait: bool = True, timeout: float | None = None) -> None:
        """
        ### Arm the vehicle.

        If wait is True, wait for arm operation to complete before
        returning.  If timeout is nonzero, raise a TimeoutError if the
        vehicle has not armed after timeout seconds.

        ---

        Args:
            `wait`: Whether to wait for the operation to complete
            `timeout`: Maximum time to wait in seconds

        ---
        """

        self.armed = True

        if wait:
            self.wait_for(lambda: self.armed, timeout=timeout,
                          errmsg='failed to arm vehicle')
    # arm


    def disarm(self, wait: bool = True, timeout: float | None = None) -> None:
        """
        ### Disarm the vehicle.

        If wait is True, wait for disarm operation to complete before
        returning.  If timeout is nonzero, raise a TimeoutError if the
        vehicle has not disarmed after timeout seconds.

        ---

        Args:
            `wait`: Whether to wait for the operation to complete
            `timeout`: Maximum time to wait in seconds

        ---
        """
        self.armed = False

        if wait:
            self.wait_for(lambda: not self.armed, timeout=timeout,
                          errmsg='failed to disarm vehicle')
    # disarm


    def wait_for_mode(self, mode: VehicleMode | str, timeout: float | None = None) -> None:
        """
        ### Set the flight mode and wait for it to change.

        If timeout is nonzero, raise a TimeoutError if the flight mode
        hasn't changed after timeout seconds.

        ---

        Args:
            `mode`: The target flight mode
            `timeout`: Maximum time to wait in seconds

        ---
        """

        if not isinstance(mode, VehicleMode):
            mode = VehicleMode(mode)

        self.mode = mode

        self.wait_for(lambda: self.mode.name == mode.name,
                      timeout=timeout,
                      errmsg='failed to set flight mode')
    # wait_for_mode


    def wait_for_alt(
        self, 
        alt: float, 
        epsilon: float = 0.1, 
        rel: bool = True, 
        timeout: float | None = None
    ) -> None:
        """
        ### Wait for the vehicle to reach the specified altitude.

        Wait for the vehicle to get within epsilon meters of the
        given altitude.  If rel is True (the default), use the
        global_relative_frame. If rel is False, use the global_frame.
        If timeout is nonzero, raise a TimeoutError if the specified
        altitude has not been reached after timeout seconds.

        ---

        Args:
            `alt`: Target altitude in meters
            `epsilon`: Tolerance in meters
            `rel`: Use relative altitude if True, absolute if False
            `timeout`: Maximum time to wait in seconds

        ---
        """

        def get_alt() -> float | None:
            if rel:
                alt = self.location.global_relative_frame.alt
            else:
                alt = self.location.global_frame.alt

            return alt

        def check_alt() -> bool:
            cur = get_alt()
            if cur is None:
                return False
            delta = abs(alt - cur)

            return (
                (delta < epsilon) or
                (cur > alt > start) or
                (cur < alt < start)
            )

        start = get_alt()

        self.wait_for(
            check_alt,
            timeout=timeout,
            errmsg='failed to reach specified altitude')
    # wait_for_alt


    def wait_simple_takeoff(
        self, 
        alt: float | None = None, 
        epsilon: float = 0.1, # within 10%
        timeout: float | None = None
    ) -> None:
        """
        ### Take off and wait to reach target altitude.

        ---

        Args:
            `alt`: Target altitude in meters
            `epsilon`: Tolerance in meters
            `timeout`: Maximum time to wait in seconds

        ---
        """
        self.simple_takeoff(alt)

        if alt is not None:
            self.wait_for_alt(alt, epsilon=epsilon, timeout=timeout)
    # wait_simple_takeoff


    def simple_takeoff(self, alt: float | None = None) -> None:
        """
        ### Take off and fly the vehicle to the specified altitude (in meters) and then wait for another command.

        ---

        📝
        #### NOTE:

            This function should only be used on Copter vehicles.

        ---

        The vehicle must be in GUIDED mode and armed before this is called.

        There is no mechanism for notification when the correct altitude is reached,
        and if another command arrives before that point (e.g. `simple_goto`) it will be run instead.

        ---

        ⚠️
        #### WARNING:

           Apps should code to ensure that the vehicle will reach a safe altitude before
           other commands are executed. A good example is provided in the guide topic `guide/taking_off`.

        ---

        Args:
            `alt`: Target height, in metres.

        ---
        """
        if alt is not None:
            altitude = float(alt)
            if math.isnan(altitude) or math.isinf(altitude):
                raise ValueError("Altitude was NaN or Infinity. Please provide a real number")
            self._master.mav.command_long_send(
                0, 
                0, 
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, 
                altitude
            )
    # simple_takeoff


    def simple_goto(
        self, 
        location: LocationGlobal | LocationGlobalRelative, 
        airspeed: float | None = None, 
        groundspeed: float | None = None
    ) -> None:
        """
        ### Go to a specified global location (`LocationGlobal` or `LocationGlobalRelative`).

        There is no mechanism for notification when the target location is reached, and if another command arrives
        before that point that will be executed immediately.

        You can optionally set the desired airspeed or groundspeed (this is identical to setting
        `airspeed` or `groundspeed`). The vehicle will determine what speed to
        use if the values are not set or if they are both set.

        The method will change the `VehicleMode` to `GUIDED` if necessary.

        ```python
            # Set mode to guided - this is optional as the simple_goto method will change the mode if needed.
            vehicle.mode = VehicleMode("GUIDED")

            # Set the LocationGlobal to head towards
            a_location = LocationGlobal(-34.364114, 149.166022, 30)
            vehicle.simple_goto(a_location)
        ```

        ---

        Args:
            `location`: The target location (`LocationGlobal` or `LocationGlobalRelative`).
            `airspeed`: Target airspeed in m/s (optional).
            `groundspeed`: Target groundspeed in m/s (optional).

        ---
        """
        if isinstance(location, LocationGlobalRelative):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            alt = location.alt
        elif isinstance(location, LocationGlobal):
            # This should be the proper code:
            # frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            # However, APM discards information about the relative frame
            # and treats any alt value as relative. So we compensate here.
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            if not self.home_location:
                self.commands.download()
                self.commands.wait_ready()
            alt = location.alt - self.home_location.alt
        else:
            raise ValueError('Expecting location to be LocationGlobal or LocationGlobalRelative.')

        self._master.mav.mission_item_send(
            0, 0, 0, frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
            2, 0, 0,
            0, 0, 0, 
            location.lat, 
            location.lon,
            alt
        )

        if airspeed is not None:
            self.airspeed = airspeed
        if groundspeed is not None:
            self.groundspeed = groundspeed
    # simple_goto


    def send_mavlink(self, message: Any) -> None:
        """
        ### This method is used to send raw MAVLink "custom messages" to the vehicle.

        The function can send arbitrary messages/commands to the connected vehicle at any time and in any vehicle mode.
        It is particularly useful for controlling vehicles outside of missions (for example, in GUIDED mode).

        The `message_factory` is used to create messages in the appropriate format.

        For more information see the guide topic: `guided_mode_how_to_send_commands`.

        ---

        Args:
            `message`: A `MAVLink_message` instance, created using `message_factory`.
                There is need to specify the system id, component id or sequence number of messages as the API will set these appropriately.

        ---
        """
        self._master.mav.send(message)
    # send_mavlink


    @property
    def message_factory(self) -> Any:
        """
        ### Returns an object that can be used to create raw MAVLink messages that are appropriate for this vehicle.
        
        The message can then be sent using `send_mavlink(message)`.

        ---

        📝
        #### NOTE:

            Vehicles support a subset of the messages defined in the MAVLink standard. For more information
            about the supported sets see wiki topics:
            [Copter Commands in Guided Mode](http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/)
            and [Plane Commands in Guided Mode](http://dev.ardupilot.com/wiki/plane-commands-in-guided-mode/).

        ---

        All message types are defined in the central MAVLink github repository.  For example, a Pixhawk understands
        the following messages (from [pixhawk.xml](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/pixhawk.xml)):

        ```xml
            <message id="153" name="IMAGE_TRIGGER_CONTROL">
                <field type="uint8_t" name="enable">0 to disable, 1 to enable</field>
            </message>
        ```

        The name of the factory method will always be the lower case version of the message name with * _encode * appended.
        Each field in the XML message definition must be listed as arguments to this factory method.  
        
        So for this example message, the call would be:

        ```python
            msg = vehicle.message_factory.image_trigger_control_encode(True)
            vehicle.send_mavlink(msg)
        ```

        Some message types include "addressing information". If present, there is no need to specify the `target_system`
        id (just set to zero) as DroneKit will automatically update messages with the correct ID for the connected
        vehicle before sending.
        The `target_component` should be set to 0 (broadcast) unless the message is to specific component.
        CRC fields and sequence numbers (if defined in the message type) are automatically set by DroneKit and can also
        be ignored/set to zero.

        For more information see the guide topic: [Guided Mode How to Send Commands](https://dronekit.netlify.app/guide/guided_mode_how_to_send_commands).
        """
        return self._master.mav
    # message_factory


    def initialize(
        self, 
        rate: int | None = 4, 
        heartbeat_timeout: int = 30
    ) -> None:
        """
        ### Initialize the main vehicle connection.

        ---

        Args:
            `rate`: Data stream refresh rate in Hz
            `heartbeat_timeout`: Timeout for heartbeat in seconds

        ---
        """
        self._handler.start()

        # Start heartbeat polling.
        start = monotonic.monotonic()
        self._heartbeat_error = heartbeat_timeout or 0
        self._heartbeat_started = True
        self._heartbeat_lastreceived = start

        # Poll for first heartbeat.
        # If heartbeat times out, this will interrupt.
        while self._handler._alive:
            time.sleep(.1)
            if self._heartbeat_lastreceived != start:
                break
        if not self._handler._alive:
            raise APIException('Timeout in initializing connection.')

        # Register target_system now.
        self._handler.target_system = self._heartbeat_system

        # Wait until board has booted.
        while True:
            if self._flightmode not in [None, 'INITIALISING', 'MAV']:
                break
            time.sleep(0.1)

        # Initialize data stream.
        if rate is not None:
            self._master.mav.request_data_stream_send(
                0, 
                0, 
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 
                rate, 
                1
            )

        self.add_message_listener('HEARTBEAT', self.send_capabilities_request)

        # Ensure initial parameter download has started.
        while True:
            # This fn actually rate limits itself to every 2s.
            # Just retry with persistence to get our first param stream.
            self._master.param_fetch_all()
            time.sleep(0.1)
            if self._params_count > -1:
                break
    # initialize


    def send_capabilties_request(self, vehicle: 'Vehicle', name: str, m: Any) -> None:
        """
        ### An alias for send_capabilities_request.

        The word "capabilities" was misspelled in previous versions of this code. This is simply
        an alias to send_capabilities_request using the legacy name.
        """
        return self.send_capabilities_request(vehicle, name, m)
    # send_capabilties_request


    def send_capabilities_request(self, vehicle: 'Vehicle', name: str, m: Any) -> None:
        """
        ### Request an AUTOPILOT_VERSION packet
        """
        capability_msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1, 0, 0, 0, 0, 0, 0)
        vehicle.send_mavlink(capability_msg)
    # send_capabilities_request


    def play_tune(self, tune: str) -> None:
        """
        ### Play a tune on the vehicle

        ---

        Args:
            `tune`: The tune to play as a string

        ---
        """
        msg = self.message_factory.play_tune_encode(0, 0, tune)
        self.send_mavlink(msg)
    # play_tune


    def wait_ready(self, *types: str | bool, **kwargs: Any) -> bool:
        """
        ### Waits for specified attributes to be populated from the vehicle (values are initially `None`).

        This is typically called "behind the scenes" to ensure that `connect` does not return until
        attributes have populated (via the `wait_ready` parameter). You can also use it after connecting to
        wait on a specific value(s).

        There are two ways to call the method:

        ```python
            #Wait on default attributes to populate
            vehicle.wait_ready(True)

            #Wait on specified attributes (or array of attributes) to populate
            vehicle.wait_ready('mode','airspeed')
        ```

        Using the `wait_ready(True)` waits on `parameters`, `gps_0`,
        `armed`, `mode`, and `attitude`. In practice this usually
        means that all supported attributes will be populated.

        By default, the method will timeout after 30 seconds and raise an exception if the
        attributes were not populated.

        ---

        Args:
            `types`: `True` to wait on the default set of attributes, or a
                comma-separated list of the specific attributes to wait on.
            `timeout`: Timeout in seconds after which the method will raise an exception
                (the default) or return `False`. The default timeout is 30 seconds.
            `raise_exception`: If `True` the method will raise an exception on timeout,
                otherwise the method will return `False`. The default is `True` (raise exception).

        Returns:
            `True` if all attributes populated, `False` if timeout occurred and raise_exception is False

        ---
        """
        timeout = kwargs.get('timeout', 30)
        raise_exception = kwargs.get('raise_exception', True)

        # Vehicle defaults for wait_ready(True) or wait_ready()
        if list(types) == [True] or list(types) == []:
            types = self._default_ready_attrs

        if not all(isinstance(item, str) for item in types):
            raise ValueError('wait_ready expects one or more string arguments.')

        # Wait for these attributes to have been set.
        await_attributes = set(types)
        start = monotonic.monotonic()
        still_waiting_last_message_sent = start
        still_waiting_callback = kwargs.get('still_waiting_callback')
        still_waiting_message_interval = kwargs.get('still_waiting_interval', 1)

        while not await_attributes.issubset(self._ready_attrs):
            time.sleep(0.1)
            now = monotonic.monotonic()
            if now - start > timeout:
                if raise_exception:
                    raise TimeoutError(f'wait_ready experienced a timeout after {timeout} seconds.')
                else:
                    return False
            if (still_waiting_callback and
                    now - still_waiting_last_message_sent > still_waiting_message_interval):
                still_waiting_last_message_sent = now
                if still_waiting_callback:
                    still_waiting_callback(await_attributes - self._ready_attrs)

        return True
    # wait_ready


    def reboot(self) -> None:
        """
        ### Requests an autopilot reboot by sending a `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` command.
        """

        reboot_msg = self.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
            0,  # confirmation
            1,  # param 1, autopilot (reboot)
            0,  # param 2, onboard computer (do nothing)
            0,  # param 3, camera (do nothing)
            0,  # param 4, mount (do nothing)
            0, 0, 0)  # param 5 ~ 7 not used

        self.send_mavlink(reboot_msg)
    # reboot


    def send_calibrate_gyro(self) -> None:
        """
        ### Request gyroscope calibration.
        """

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            1,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)
    # send_calibrate_gyro


    def send_calibrate_magnetometer(self) -> None:
        """
        ### Request magnetometer calibration.
        """

        # ArduPilot requires the MAV_CMD_DO_START_MAG_CAL command, only present in the ardupilotmega.xml definition
        if self._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            calibration_command = self.message_factory.command_long_encode(
                self._handler.target_system, 0,  # target_system, target_component
                mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,  # command
                0,  # confirmation
                0,  # param 1, uint8_t bitmask of magnetometers (0 means all).
                1,  # param 2, Automatically retry on failure (0=no retry, 1=retry).
                1,  # param 3, Save without user input (0=require input, 1=autosave).
                0,  # param 4, Delay (seconds).
                0,  # param 5, Autoreboot (0=user reboot, 1=autoreboot).
                0,  # param 6, Empty.
                0,  # param 7, Empty.
            )
        else:
            calibration_command = self.message_factory.command_long_encode(
                self._handler.target_system, 0,  # target_system, target_component
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
                0,  # confirmation
                0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
                1,  # param 2, 1: magnetometer calibration
                0,  # param 3, 1: ground pressure calibration
                0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
                0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
                0,  # param 6, 2: airspeed calibration
                0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
            )

        self.send_mavlink(calibration_command)
    # send_calibrate_magnetometer


    def send_calibrate_accelerometer(self, simple: bool = False) -> None:
        """
        ### Request accelerometer calibration.

        ---

        Args:
            `simple`: if True, perform simple accelerometer calibration

        ---
        """

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            4 if simple else 1,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)
    # send_calibrate_accelerometer


    def send_calibrate_vehicle_level(self) -> None:
        """
        ### Request vehicle level (accelerometer trim) calibration.
        """

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            2,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)
    # send_calibrate_vehicle_level


    def send_calibrate_barometer(self) -> None:
        """
        ### Request barometer calibration.
        """

        calibration_command = self.message_factory.command_long_encode(
            self._handler.target_system, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            0,  # param 2, 1: magnetometer calibration
            1,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
        self.send_mavlink(calibration_command)
    # send_calibrate_barometer
# Vehicle



class Gimbal(object):
    """
    ### Gimbal status and control.

    An object of this type is returned by `Vehicle.gimbal`. The
    gimbal orientation can be obtained from its `roll`, `pitch` and
    `yaw` attributes.

    The gimbal orientation can be set explicitly using `rotate`
    or you can set the gimbal (and vehicle) to track a specific "region of interest" using
    `target_location`.

    ---

    📝
    #### NOTE:

        * The orientation attributes are created with values of `None`. If a gimbal is present,
          the attributes are populated shortly after initialisation by messages from the autopilot.
        * The attribute values reflect the last gimbal setting-values rather than actual measured values.
          This means that the values won't change if you manually move the gimbal, and that the value
          will change when you set it, even if the specified orientation is not supported.
        * A gimbal may not support all axes of rotation. For example, the Solo gimbal will set pitch
          values from 0 to -90 (straight ahead to straight down), it will rotate the vehicle to follow specified
          yaw values, and will ignore roll commands (not supported).

    ---
    """

    def __init__(self, vehicle: 'Vehicle') -> None:
        super(Gimbal, self).__init__()

        self._pitch: float | None = None
        self._roll: float | None = None
        self._yaw: float | None = None
        self._vehicle = vehicle

        @vehicle.on_message('MOUNT_STATUS')
        def listener(vehicle: 'Vehicle', name: str, m: Any) -> None:
            self._pitch = m.pointing_a / 100.0
            self._roll = m.pointing_b / 100.0
            self._yaw = m.pointing_c / 100.0
            vehicle.notify_attribute_listeners('gimbal', vehicle.gimbal)

        @vehicle.on_message('MOUNT_ORIENTATION')
        def listener(vehicle: 'Vehicle', name: str, m: Any) -> None:
            self._pitch = m.pitch
            self._roll = m.roll
            self._yaw = m.yaw
            vehicle.notify_attribute_listeners('gimbal', vehicle.gimbal)
    # __init__


    @property
    def pitch(self) -> float | None:
        """
        ### Gimbal pitch in degrees relative to the vehicle (see diagram for `attitude`).
        
        A value of 0 represents a camera pointed straight ahead relative to the front of the vehicle,
        while -90 points the camera straight down.

        ---

        📝
        #### NOTE:

            This is the last pitch value sent to the gimbal (not the actual/measured pitch).

        ---
        """
        return self._pitch
    # pitch


    @property
    def roll(self) -> float | None:
        """
        ### Gimbal roll in degrees relative to the vehicle (see diagram for `attitude`).

        ---

        📝
        #### NOTE:

            This is the last roll value sent to the gimbal (not the actual/measured roll).

        ---
        """
        return self._roll
    # roll


    @property
    def yaw(self) -> float | None:
        """
        ### Gimbal yaw in degrees relative to *global frame* (0 is North, 90 is West, 180 is South etc).

        ---

        📝
        #### NOTE:

            This is the last yaw value sent to the gimbal (not the actual/measured yaw).

        ---
        """
        return self._yaw
    # yaw


    def rotate(self, pitch: float, roll: float, yaw: float) -> None:
        """
        ### Rotate the gimbal to a specific vector.

        ```python
            #Point the gimbal straight down
            vehicle.gimbal.rotate(-90, 0, 0)
        ```

        ---

        Args:
            `pitch`: Gimbal pitch in degrees relative to the vehicle (see diagram for `attitude`).
                A value of 0 represents a camera pointed straight ahead relative to the front of the vehicle,
                while -90 points the camera straight down.
            `roll`: Gimbal roll in degrees relative to the vehicle (see diagram for `attitude`).
            `yaw`: Gimbal yaw in degrees relative to *global frame* (0 is North, 90 is West, 180 is South etc.)

        ---
        """
        msg = self._vehicle.message_factory.mount_configure_encode(
            0, 1,    # target system, target component
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,  #mount_mode
            1,  # stabilize roll
            1,  # stabilize pitch
            1,  # stabilize yaw
        )
        self._vehicle.send_mavlink(msg)
        msg = self._vehicle.message_factory.mount_control_encode(
            0, 1,    # target system, target component
            pitch * 100,  # pitch is in centidegrees
            roll * 100,  # roll
            yaw * 100,  # yaw is in centidegrees
            0  # save position
        )
        self._vehicle.send_mavlink(msg)
    # rotate


    def target_location(self, roi: LocationGlobal | LocationGlobalRelative) -> None:
        """
        ### Point the gimbal at a specific region of interest (ROI).

        ```python
        #Set the camera to track the current home location.
        vehicle.gimbal.target_location(vehicle.home_location)
        ```

        The target position must be defined in a `LocationGlobalRelative` or `LocationGlobal`.

        This function can be called in AUTO or GUIDED mode.

        In order to clear an ROI you can send a location with all zeros (e.g. `LocationGlobalRelative(0,0,0)`).

        ---

        Args:
            `roi`: Target location in global relative frame.

        ---
        """
        # set gimbal to targeting mode
        msg = self._vehicle.message_factory.mount_configure_encode(
            0, 1,    # target system, target component
            mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT,  # mount_mode
            1,  # stabilize roll
            1,  # stabilize pitch
            1,  # stabilize yaw
        )
        self._vehicle.send_mavlink(msg)

        # Get altitude relative to home irrespective of Location object passed in.
        if isinstance(roi, LocationGlobalRelative):
            alt = roi.alt
        elif isinstance(roi, LocationGlobal):
            if not self._vehicle.home_location:
                self._vehicle.commands.download()
                self._vehicle.commands.wait_ready()
            alt = roi.alt - self._vehicle.home_location.alt
        else:
            raise ValueError('Expecting location to be LocationGlobal or LocationGlobalRelative.')

        # set the ROI
        msg = self._vehicle.message_factory.command_long_encode(
            0, 1,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # command
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            roi.lat,
            roi.lon,
            alt
        )
        self._vehicle.send_mavlink(msg)
    # target_location


    def release(self) -> None:
        """
        ### Release control of the gimbal to the user (RC Control).

        This should be called once you've finished controlling the mount with either `rotate`
        or `target_location`. Control will automatically be released if you change vehicle mode.
        """
        msg = self._vehicle.message_factory.mount_configure_encode(
            0, 1,    # target system, target component
            mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,  # mount_mode
            1,  # stabilize roll
            1,  # stabilize pitch
            1,  # stabilize yaw
        )
        self._vehicle.send_mavlink(msg)
    # release


    def __str__(self) -> str:
        return (f"{self.__class__.__name__}: "
                f"pitch = {self.pitch}, "
                f"roll = {self.roll}, "
                f"yaw = {self.yaw}")
    # __str__
# Gimbal



class Parameters(MutableMapping, HasObservers):
    """
    ### This object is used to get and set the values of named parameters for a vehicle.
    
    See the following links for information about
    the supported parameters for each platform: [Copter Parameters](http://copter.ardupilot.com/wiki/configuration/arducopter-parameters/),
    [Plane Parameters](http://plane.ardupilot.com/wiki/arduplane-parameters/), [Rover Parameters](http://rover.ardupilot.com/wiki/apmrover2-parameters/).

    The code fragment below shows how to get and set the value of a parameter.

    ```python
        # Print the value of the THR_MIN parameter.
        print(f"Param: {vehicle.parameters['THR_MIN']}")

        # Change the parameter value to something different.
        vehicle.parameters['THR_MIN'] = 100
    ```

    It is also possible to observe parameters and to iterate the `Vehicle.parameters`.

    For more information see [Vehicle State and Parameters Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters).
    """

    def __init__(self, vehicle: 'Vehicle') -> None:
        super(Parameters, self).__init__()
        self._logger = logging.getLogger(__name__)
        self._vehicle = vehicle
    # __init__


    def __getitem__(self, name: str) -> float:
        name = name.upper()
        self.wait_ready()
        return self._vehicle._params_map[name]
    # __getitem__


    def __setitem__(self, name: str, value: float) -> None:
        name = name.upper()
        self.wait_ready()
        self.set(name, value)
    # __setitem__


    def __delitem__(self, name: str) -> None:
        raise APIException('Cannot delete value from parameters list.')
    # __delitem__


    def __len__(self) -> int:
        return len(self._vehicle._params_map)
    # __len__


    def __iter__(self) -> Iterator[str]:
        return self._vehicle._params_map.__iter__()
    # __iter__


    def get(self, name: str, wait_ready: bool = True) -> float | None:
        """
        ### Get a parameter value by name.

        ---

        Args:
            `name`: The parameter name
            `wait_ready`: Whether to wait for parameters to be downloaded first

        Returns:
            The parameter value, or None if not found

        ---
        """
        name = name.upper()
        if wait_ready:
            self.wait_ready()
        return self._vehicle._params_map.get(name, None)
    # get


    def set(self, name: str, value: float, retries: int = 3, wait_ready: bool = False) -> bool:
        """
        ### Set a parameter value.

        ---

        Args:
            `name`: The parameter name
            `value`: The new parameter value
            `retries`: Number of retries if setting fails
            `wait_ready`: Whether to wait for parameters to be downloaded first

        Returns:
            True if successful, False otherwise

        ---
        """
        if wait_ready:
            self.wait_ready()

        # TODO dumbly reimplement this using timeout loops
        # because we should actually be awaiting an ACK of PARAM_VALUE
        # changed, but we don't have a proper ack structure, we'll
        # instead just wait until the value itself was changed

        name = name.upper()
        # convert to single precision floating point number (the type used by low level mavlink messages)
        value = float(struct.unpack('f', struct.pack('f', value))[0])
        remaining = retries
        while True:
            self._vehicle._master.param_set_send(name, value)
            tstart = monotonic.monotonic()
            if remaining == 0:
                break
            remaining -= 1
            while monotonic.monotonic() - tstart < 1:
                if name in self._vehicle._params_map and self._vehicle._params_map[name] == value:
                    return True
                time.sleep(0.1)

        if retries > 0:
            self._logger.error(f"timeout setting parameter {name} to {value}")
        return False
    # set


    def wait_ready(self, **kwargs: Any) -> None:
        """
        ### Block the calling thread until parameters have been downloaded
        """
        self._vehicle.wait_ready('parameters', **kwargs)
    # wait_ready


    def add_attribute_listener(self, attr_name: str, *args: Any, **kwargs: Any) -> None:
        """
        ### Add a listener callback on a particular parameter.

        The callback can be removed using `remove_attribute_listener`.

        ---

        📝
        #### NOTE:

            The `on_attribute` decorator performs the same operation as this method, but with
            a more elegant syntax. Use `add_attribute_listener` only if you will need to remove
            the observer.

        ---

        The callback function is invoked only when the parameter changes.

        The callback arguments are:

        * `self` - the associated `Parameters`.
        * `attr_name` - the parameter name. This can be used to infer which parameter has triggered
          if the same callback is used for watching multiple parameters.
        * `msg` - the new parameter value (so you don't need to re-query the vehicle object).

        The example below shows how to get callbacks for the `THR_MIN` parameter:

        ```python
            #Callback function for the THR_MIN parameter
            def thr_min_callback(self, attr_name, value):
                print(f" PARAMETER CALLBACK: {attr_name} changed to: {value}")

            #Add observer for the vehicle's THR_MIN parameter
            vehicle.parameters.add_attribute_listener('THR_MIN', thr_min_callback)
        ```

        See [Vehicle State and Parameters Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters) for more information.

        ---

        Args:
            `attr_name`: The name of the parameter to watch (or '*' to watch all parameters).
            `args`: The callback to invoke when a change in the parameter is detected.

        ---
        """
        attr_name = attr_name.upper()
        return super(Parameters, self).add_attribute_listener(attr_name, *args, **kwargs)
    # add_attribute_listener


    def remove_attribute_listener(self, attr_name: str, *args: Any, **kwargs: Any) -> None:
        """
        ### Remove a parameter listener that was previously added using `add_attribute_listener`.

        For example to remove the `thr_min_callback()` callback function:

        ```python
            vehicle.parameters.remove_attribute_listener('thr_min', thr_min_callback)
        ```

        See [Vehicle State and Parameters Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters) for more information.

        ---

        Args:
            `attr_name`: The parameter name that is to have an observer removed (or '*' to remove an 'all attribute' observer).
            `args`: The callback function to remove.

        ---
        """
        attr_name = attr_name.upper()
        return super(Parameters, self).remove_attribute_listener(attr_name, *args, **kwargs)
    # remove_attribute_listener


    def notify_attribute_listeners(self, attr_name: str, *args: Any, **kwargs: Any) -> None:
        """
        ### Notify all listeners of a parameter change.

        ---

        Args:
            `attr_name`: The parameter name that changed

        ---
        """
        attr_name = attr_name.upper()
        return super(Parameters, self).notify_attribute_listeners(attr_name, *args, **kwargs)
    # notify_attribute_listeners


    def on_attribute(self, attr_name: str, *args: Any, **kwargs: Any) -> Callable:
        """
        ### Decorator for parameter listeners.

        ---

        📝
        #### NOTE:

            There is no way to remove a listener added with this decorator. Use
            `add_attribute_listener` if you need to be able to remove
            the `listener <remove_attribute_listener>`.

        ---

        The callback function is invoked only when the parameter changes.

        The callback arguments are:

        * `self` - the associated `Parameters`.
        * `attr_name` - the parameter name. This can be used to infer which parameter has triggered
          if the same callback is used for watching multiple parameters.
        * `msg` - the new parameter value (so you don't need to re-query the vehicle object).

        The code fragment below shows how to get callbacks for the `THR_MIN` parameter:

        ```python
            @vehicle.parameters.on_attribute('THR_MIN')
            def decorated_thr_min_callback(self, attr_name, value):
                print(f" PARAMETER CALLBACK: {attr_name} changed to: {value}")
        ```

        See [Vehicle State and Parameters Guide](https://dronekit.netlify.app/guide/vehicle_state_and_parameters) for more information.

        ---

        Args:
            `attr_name`: The name of the parameter to watch (or '*' to watch all parameters).
            `args`: The callback to invoke when a change in the parameter is detected.

        ---
        """
        attr_name = attr_name.upper()
        return super(Parameters, self).on_attribute(attr_name, *args, **kwargs)
    # on_attribute
# Parameters



class Command(mavutil.mavlink.MAVLink_mission_item_message):
    """
    ### A waypoint object.

    This object encodes a single mission item command. The set of commands that are supported
    by ArduPilot in Copter, Plane and Rover (along with their parameters) are listed in the wiki article
    MAVLink Mission Command Messages (MAV_CMD) <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`.

    For example, to create a `NAV_WAYPOINT <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_nav_waypoint>` command:

    ```python
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,-34.364114, 149.166022, 30)
    ```

    ---

    Args:
        `target_system`: This can be set to any value
            (DroneKit changes the value to the MAVLink ID of the connected vehicle before the command is sent).
        `target_component`: The component id if the message is intended for a particular component within the target system
            (for example, the camera). Set to zero (broadcast) in most cases.
        `seq`: The sequence number within the mission (the autopilot will reject messages sent out of sequence).
            This should be set to zero as the API will automatically set the correct value when uploading a mission.
        `frame`: The frame of reference used for the location parameters (x, y, z). In most cases this will be
            `mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT`, which uses the WGS84 global coordinate system for latitude and longitude, but sets altitude
            as relative to the home position in metres (home altitude = 0). For more information `see the wiki here
            <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#frames_of_reference>`.
        `command`: The specific mission command (e.g. `mavutil.mavlink.MAV_CMD_NAV_WAYPOINT`). The supported commands (and command parameters
            are listed `on the wiki <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`.
        `current`: Set to zero (not supported).
        `autocontinue`: Set to zero (not supported).
        `param1`: Command specific parameter (depends on specific `Mission Command (MAV_CMD) <http://planner.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/>`).
        `param2`: Command specific parameter.
        `param3`: Command specific parameter.
        `param4`: Command specific parameter.
        `x`: (param5) Command specific parameter used for latitude (if relevant to command).
        `y`: (param6) Command specific parameter used for longitude (if relevant to command).
        `z`: (param7) Command specific parameter used for altitude (if relevant). The reference frame for altitude depends on the `frame`.

    ---
    """
    pass
# Command



class CommandSequence(object):
    """
    ### A sequence of vehicle waypoints (a "mission").

    Operations include 'array style' indexed access to the various contained waypoints.

    The current commands/mission for a vehicle are accessed using the `Vehicle.commands` attribute.
    Waypoints are not downloaded from vehicle until `download()` is called.  The download is asynchronous;
    use `wait_ready()` to block your thread until the download is complete.
    The code to download the commands from a vehicle is shown below:

    ```python
        #Connect to a vehicle object (for example, on com14)
        vehicle = connect('com14', wait_ready=True)

        # Download the vehicle waypoints (commands). Wait until download is complete.
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
    ```

    The set of commands can be changed and uploaded to the client. The changes are not guaranteed to be complete until
    `upload()` is called.

    ```python
        cmds = vehicle.commands
        cmds.clear()
        lat = -34.364114,
        lon = 149.166022
        altitude = 30.0
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,
            lat, lon, altitude)
        cmds.add(cmd)
        cmds.upload()
    ```
    """

    def __init__(self, vehicle: 'Vehicle') -> None:
        self._vehicle = vehicle
    # __init__


    def download(self) -> None:
        """
        ### Download all waypoints from the vehicle.
        
        The download is asynchronous. Use `wait_ready()` to block your thread until the download is complete.
        """
        self.wait_ready()
        self._vehicle._ready_attrs.remove('commands')
        self._vehicle._wp_loaded = False
        self._vehicle._master.waypoint_request_list_send()
        # BIG FIXME - wait for full wpt download before allowing any of the accessors to work
    # download


    def wait_ready(self, **kwargs: Any) -> None:
        """
        ### Block the calling thread until waypoints have been downloaded.

        This can be called after `download()` to block the thread until the asynchronous download is complete.
        """
        return self._vehicle.wait_ready('commands', **kwargs)
    # wait_ready


    def clear(self) -> None:
        """
        ### Clear the command list.

        This command will be sent to the vehicle only after you call `upload()`.
        """

        # Add home point again.
        self.wait_ready()
        home = None
        try:
            home = self._vehicle._wploader.wp(0)
        except:
            pass
        self._vehicle._wploader.clear()
        if home:
            self._vehicle._wploader.add(home, comment='Added by DroneKit')
        self._vehicle._wpts_dirty = True
    # clear


    def add(self, cmd: Command) -> None:
        """
        ### Add a new command (waypoint) at the end of the command list.

        ---

        📝
        #### NOTE:

            Commands are sent to the vehicle only after you call `upload()`.

        ---

        Args:
            `cmd`: The command to be added.

        ---
        """
        self.wait_ready()
        self._vehicle._handler.fix_targets(cmd)
        self._vehicle._wploader.add(cmd, comment='Added by DroneKit')
        self._vehicle._wpts_dirty = True
    # add


    def upload(self, timeout: float | None = None) -> None:
        """
        ### Call `upload()` after adding or clearing mission commands.

        After the return from `upload()` any writes are guaranteed to have completed (or thrown an
        exception) and future reads will see their effects.

        ---

        Args:
            `timeout`: The timeout for uploading the mission. No timeout if not provided or set to None.

        ---
        """
        if self._vehicle._wpts_dirty:
            self._vehicle._master.waypoint_clear_all_send()
            start_time = time.time()
            if self._vehicle._wploader.count() > 0:
                self._vehicle._wp_uploaded = [False] * self._vehicle._wploader.count()
                self._vehicle._master.waypoint_count_send(self._vehicle._wploader.count())
                while False in self._vehicle._wp_uploaded:
                    if timeout and time.time() - start_time > timeout:
                        raise TimeoutError
                    time.sleep(0.1)
                self._vehicle._wp_uploaded = None
            self._vehicle._wpts_dirty = False
    # upload


    @property
    def count(self) -> int:
        """
        ### Return number of waypoints.

        ---

        Returns:
            The number of waypoints in the sequence.

        ---
        """
        return max(self._vehicle._wploader.count() - 1, 0)
    # count


    @property
    def next(self) -> int:
        """
        ### Get the currently active waypoint number.
        """
        return self._vehicle._current_waypoint
    # next


    @next.setter
    def next(self, index: int) -> None:
        """
        ### Set a new `next` waypoint for the vehicle.
        """
        self._vehicle._master.waypoint_set_current_send(index)
    # next.setter


    def __len__(self) -> int:
        """
        ### Return number of waypoints.

        ---

        Returns:
            The number of waypoints in the sequence.

        ---
        """
        return max(self._vehicle._wploader.count() - 1, 0)
    # __len__


    def __getitem__(self, index: int | slice) -> Command | list[Command]:
        if isinstance(index, slice):
            return [self[ii] for ii in range(*index.indices(len(self)))]
        elif isinstance(index, int):
            item = self._vehicle._wploader.wp(index + 1)
            if not item:
                raise IndexError('Index %s out of range.' % index)
            return item
        else:
            raise TypeError('Invalid argument type.')
    # __getitem__


    def __setitem__(self, index: int, value: Command) -> None:
        self._vehicle._wploader.set(value, index + 1)
        self._vehicle._wpts_dirty = True
    # __setitem__
# CommandSequence



def default_still_waiting_callback(atts: set[str]) -> None:
    """
    ### Default callback for wait_ready still waiting status.

    ---

    Args:
        `atts`: Set of attributes still waiting to be populated

    ---
    """
    logging.getLogger(__name__).debug("Still waiting for data from vehicle: %s" % ','.join(atts))
# default_still_waiting_callback



def connect(
    ip: str,
    _initialize: bool = True,
    wait_ready: bool | list[str] | None = None,
    timeout: float = 30,
    still_waiting_callback: Callable[[set[str]], None] = default_still_waiting_callback,
    still_waiting_interval: float = 1,
    status_printer: Callable[[str], None] | None = None,
    vehicle_class: type[Vehicle] | None = None,
    rate: int = 4,
    baud: int = 115200,
    heartbeat_timeout: int = 30,
    source_system: int = 255,
    source_component: int = 0,
    use_native: bool = False
) -> Vehicle:
    """
    ### Returns a `Vehicle` object connected to the address specified by string parameter `ip`.
    
    Connection string parameters (`ip`) for different targets are listed in the [Getting Started Guide](https://dronekit.netlify.app/guide/connecting_vehicle).

    The method is usually called with `wait_ready=True` to ensure that vehicle parameters and (most) attributes are available when `connect()` returns.

    ```python
        from dronekit import connect

        # Connect to the Vehicle using "connection string" (in this case an address on network)
        vehicle = connect('127.0.0.1:14550', wait_ready=True)
    ```

    ---

    Args:
        `ip`: Connection string for target address - e.g. 127.0.0.1:14550.
        `wait_ready`: If `True` wait until all default attributes have downloaded before the method returns (default is `None`). The default attributes to wait on are: `parameters`, `gps_0`, `armed`, `mode`, and `attitude`. You can also specify a named set of parameters to wait on (e.g. `wait_ready=['system_status','mode']`).
        `status_printer`: (deprecated) method of signature `def status_printer(txt)` that prints STATUS_TEXT messages from the Vehicle and other diagnostic information. By default the status information is handled by the `autopilot` logger.
        `vehicle_class`: The class that will be instantiated by the `connect()` method. This can be any sub-class of `Vehicle` (and defaults to `Vehicle`).
        `rate`: Data stream refresh rate. The default is 4Hz (4 updates per second).
        `baud`: The baud rate for the connection. The default is 115200.
        `heartbeat_timeout`: Connection timeout value in seconds (default is 30s). If a heartbeat is not detected within this time an exception will be raised.
        `source_system`: The MAVLink ID of the `Vehicle` object returned by this method (by default 255).
        `source_component`: The MAVLink Component ID fo the `Vehicle` object returned by this method (by default 0).
        `use_native`: Use precompiled MAVLink parser.

    ---

    📝
    #### NOTE:

        The returned `Vehicle` object acts as a ground control station from the perspective of the connected "real" vehicle. It will process/receive messages from the real vehicle if they are addressed to this `source_system` id. Messages sent to the real vehicle are automatically updated to use the vehicle's `target_system` id.

        It is good practice to assign a unique id for every system on the MAVLink network.
        It is possible to configure the autopilot to only respond to guided-mode commands from a specified GCS ID.

        The `status_printer` argument is deprecated. To redirect the logging from the library and from the autopilot, configure the `dronekit` and `autopilot` loggers using the Python `logging` module.

    ---

    Returns:
        A connected vehicle of the type defined in `vehicle_class` (a superclass of `Vehicle`).

    ---
    """

    from dronekit.mavlink import MAVConnection

    if not vehicle_class:
        vehicle_class = Vehicle

    handler = MAVConnection(
        ip=ip, baud=baud, source_system=source_system, 
        source_component=source_component, use_native=use_native
    )
    vehicle = vehicle_class(handler)

    if status_printer:
        vehicle._autopilot_logger.addHandler(ErrprinterHandler(status_printer))

    if _initialize:
        vehicle.initialize(rate=rate, heartbeat_timeout=heartbeat_timeout)

    if wait_ready:
        if wait_ready is True:
            vehicle.wait_ready(
                still_waiting_interval=still_waiting_interval,
                still_waiting_callback=still_waiting_callback,
                timeout=timeout
            )
        else:
            vehicle.wait_ready(*wait_ready)

    return vehicle
# connect











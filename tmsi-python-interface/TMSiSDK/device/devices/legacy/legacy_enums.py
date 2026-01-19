'''
(c) 2023 Twente Medical Systems International B.V., Oldenzaal The Netherlands

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

#######  #     #   #####   #
   #     ##   ##  #        
   #     # # # #  #        #
   #     #  #  #   #####   #
   #     #     #        #  #
   #     #     #        #  #
   #     #     #  #####    #

/**
 * @file legacy_enums.py 
 * @brief 
 * Enumerations and constants for legacy TMSi devices.
 * Based on MATLAB Library.m and TMSiHeader64.m
 */


'''

from enum import IntEnum


class TMSiConnectionType(IntEnum):
    """Connection types for TMSi devices"""
    undefined = 0
    fiber = 1
    bluetooth = 2
    usb = 3
    wifi = 4
    network = 5


class TMSiErrorCode(IntEnum):
    """Error codes from TMSi SDK"""
    unsuccessful = 256
    invalid_handle = 257
    not_implemented = 258


class TMSiStartControl(IntEnum):
    """Start control flags for recording"""
    rtc_set = 1
    alarm_record_auto_start = 2
    man_record_enable = 4
    power_on_record_auto_start = 8
    alarm_recurring = 16
    rf_auto_start = 32
    rf_timed_start = 64
    rf_recurring = 128
    man_shutdown_enable = 256


# Unit mappings from Device.m getUnit() method
UNIT_NAMES = {
    0: '?',
    1: 'V',
    2: '%',
    3: 'b/m',
    4: 'bar',
    5: 'psi',
    6: 'mH2O',
    7: 'mmHg'
}

UNIT_EXPONENTS = {
    -3: 'm',  # milli
    -6: 'u',  # micro
    -9: 'n'   # nano
}


def get_unit_name(unit_id, unit_exponent):
    """
    Get human-readable unit name from unit ID and exponent.
    Based on MATLAB Device.m getUnit() method.
    
    Args:
        unit_id: Unit identifier (0-7)
        unit_exponent: Exponent (-3, -6, -9, or 0)
    
    Returns:
        String representation of unit (e.g., 'uV', 'mV', 'V')
    """
    base_unit = UNIT_NAMES.get(unit_id, '?')
    prefix = UNIT_EXPONENTS.get(unit_exponent, '')
    return prefix + base_unit


# Channel types from MATLAB Sampler.m
CHANNEL_TYPE_EXG = 1  # EXG channels
CHANNEL_TYPE_BIP = 2  # Bipolar channels
CHANNEL_TYPE_AUX = 3  # Auxiliary channels

# Overflow value - indicates disconnected/overloaded channel
OVERFLOW_VALUE = 2147483648  # 2^31

# Default impedance limits (kOhm) from MATLAB Sampler.m
IMPEDANCE_LIMITS = {
    2: 0,
    5: 1,
    10: 2,
    20: 3,
    50: 4,
    100: 5,
    200: 6
}

#!/usr/bin/env python3
"""
Raspberry Pi 5 Power and CPU Usage Monitor
Monitors PMIC power data, CPU usage, temperature, and throttling status.
"""

import subprocess
import time
import psutil
import re
from datetime import datetime

def get_pmic_data():
    """
    Reads detailed power data from the PMIC using vcgencmd.
    Returns USB input voltage, core voltage, core current, and core power.
    """
    data = {'usb_in_v': None, 'core_v': None, 'core_a': None, 'core_w': None}
    try:
        # Run the command to get PMIC ADC readings
        result = subprocess.run(['vcgencmd', 'pmic_read_adc'],
                                capture_output=True, text=True, check=True)
        
        # Use regex to find the specific lines and extract the float values
        usb_in_v_match = re.search(r'EXT5V_V volt\(24\)=([\d.]+)V', result.stdout)
        core_v_match = re.search(r'VDD_CORE_V volt\(15\)=([\d.]+)V', result.stdout)
        core_a_match = re.search(r'VDD_CORE_A current\(7\)=([\d.]+)A', result.stdout)
        
        if usb_in_v_match:
            data['usb_in_v'] = float(usb_in_v_match.group(1))
        if core_v_match:
            data['core_v'] = float(core_v_match.group(1))
        if core_a_match:
            data['core_a'] = float(core_a_match.group(1))
            
        # Calculate power in watts (P = V * I) if both values are available
        if data['core_v'] is not None and data['core_a'] is not None:
            data['core_w'] = data['core_v'] * data['core_a']

    except (FileNotFoundError, subprocess.CalledProcessError):
        # Handle cases where vcgencmd isn't found or fails
        pass
    return data

def get_throttled_status():
    """Get throttling status from vcgencmd"""
    try:
        result = subprocess.run(['vcgencmd', 'get_throttled'], 
                                capture_output=True, text=True, check=True)
        throttled_hex = result.stdout.strip().split('=')[1]
        throttled_int = int(throttled_hex, 16)
        
        status = {
            'under_voltage': bool(throttled_int & 0x1),
            'freq_capped': bool(throttled_int & 0x2),
            'currently_throttled': bool(throttled_int & 0x4),
            'soft_temp_limit': bool(throttled_int & 0x8),
            'under_voltage_occurred': bool(throttled_int & 0x10000),
            'freq_capped_occurred': bool(throttled_int & 0x20000),
            'throttled_occurred': bool(throttled_int & 0x40000),
            'soft_temp_limit_occurred': bool(throttled_int & 0x80000),
        }
        return status, throttled_int
    except Exception:
        return None, None

def get_temperature():
    """Get CPU temperature"""
    try:
        result = subprocess.run(['vcgencmd', 'measure_temp'], 
                                capture_output=True, text=True, check=True)
        return float(result.stdout.strip().split('=')[1].replace("'C", ''))
    except Exception:
        return None

def get_cpu_usage():
    """Get overall CPU usage percentage"""
    return psutil.cpu_percent(interval=None) # Interval handled by main loop sleep

def get_memory_usage():
    """Get memory usage"""
    return psutil.virtual_memory().percent

def monitor_system(interval=1.0, log_file=None):
    """Monitor system in real-time"""
    
    log = None
    if log_file:
        log = open(log_file, 'w')
        header = ("timestamp,usb_in_v,core_v,core_a,core_w,"
                  "cpu_total,temp,memory,throttled_hex\n")
        log.write(header)
    
    try:
        # Initial call to establish baseline for cpu_percent
        psutil.cpu_percent()
        while True:
            time.sleep(interval)
            
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            # Get all metrics
            power_data = get_pmic_data()
            cpu_total = get_cpu_usage()
            temp = get_temperature()
            mem_usage = get_memory_usage()
            throttle_status, throttle_hex = get_throttled_status()
            
            # Display to console
            print(f"\r{timestamp} | ", end='')
            print(f"USB In: {power_data.get('usb_in_v', 0.0):.3f}V | ", end='')
            print(f"Core: {power_data.get('core_a', 0.0):.3f}A ({power_data.get('core_w', 0.0):.2f}W) | ", end='')
            print(f"CPU: {cpu_total:5.1f}% | ", end='')
            print(f"Temp: {temp if temp else 0.0:4.1f}°C | ", end='')
            print(f"Mem: {mem_usage:4.1f}% | ", end='')
            
            warnings = []
            if throttle_status:
                if throttle_status['under_voltage']:
                    warnings.append("⚠️ UNDER-VOLTAGE")
                if throttle_status['soft_temp_limit']:
                    warnings.append("⚠️ TEMP-LIMIT")
            
            if warnings:
                print(f" {' '.join(warnings)}", end='')
            
            print(" " * 10, end='', flush=True) # Clear remainder of line
            
            # Log to file
            if log:
                log.write(f"{timestamp},{power_data.get('usb_in_v', '')},{power_data.get('core_v', '')},"
                          f"{power_data.get('core_a', '')},{power_data.get('core_w', '')},"
                          f"{cpu_total},{temp},{mem_usage},{throttle_hex}\n")
                log.flush()
            
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")
        if log:
            log.close()
            print(f"Log saved to: {log_file}")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        if log:
            log.close()

if __name__ == "__main__":
    log_filename = f"power_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"Logging to: {log_filename}")
    monitor_system(interval=0.5, log_file=log_filename)
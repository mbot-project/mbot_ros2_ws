#!/usr/bin/env python3
"""
CPU Usage Logger - Simple logging of CPU usage and top processes
"""

import psutil
import csv
from datetime import datetime
import time
import os

def log_cpu_usage(log_file="cpu_usage.csv"):
    """Log current CPU usage with top processes"""
    # Get overall CPU usage
    cpu_percent = psutil.cpu_percent(interval=1)
    cpu_count = psutil.cpu_count()
    
    # Get top processes by CPU
    processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cpu_percent']):
        try:
            processes.append({
                'pid': proc.info['pid'],
                'name': proc.info['name'],
                'cpu_percent': proc.info['cpu_percent']
            })
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    
    # Sort by CPU usage and take top 5
    processes.sort(key=lambda x: x['cpu_percent'], reverse=True)
    top_processes = processes[:5]
    
    # Create log entry
    timestamp = datetime.now().isoformat()
    total_used = cpu_percent
    total_idle = 100 - cpu_percent
    
    # Format top processes as "name:cpu%,name:cpu%,..."
    top_procs_str = ",".join([f"{p['name']}:{p['cpu_percent']:.1f}" for p in top_processes])
    
    # Check if file exists to write header
    file_exists = os.path.isfile(log_file)
    
    with open(log_file, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(['timestamp', 'total_cpu_used', 'total_cpu_idle', 'top_processes'])
        writer.writerow([timestamp, total_used, total_idle, top_procs_str])
    
    # Print to console
    print(f"{timestamp} - CPU: {total_used:.1f}% used, {total_idle:.1f}% idle")
    print(f"  Top process: {top_processes[0]['name']} ({top_processes[0]['cpu_percent']:.1f}%)")

if __name__ == "__main__":
    import sys
    
    interval = 1  # seconds between logs
    log_file = f"data/cpu_usage_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    print(f"Logging CPU usage every {interval} seconds to {log_file}")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            log_cpu_usage(log_file)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nLogging stopped")
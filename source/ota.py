import os
import http.server
import socketserver
import threading
import requests
import time
import socket
import argparse
import shutil

# Configuration
ESP32_IP = "192.168.2.59"  # Replace with your device's IP
LOCAL_PORT = 8070           # Port to serve firmware on your PC
PROJECT_PATH = os.path.dirname(os.path.abspath(__file__))
BUILD_PATH = os.path.join(PROJECT_PATH, "build")

def get_local_ip():
    """Get the local IP address of this machine"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()

def get_binary_path(binary_path=None):
    """Find the path to the firmware binary file"""
    # If a specific binary path is provided, use it
    if binary_path and os.path.exists(binary_path):
        return binary_path
        
    # Otherwise, look in the build directory
    if not os.path.exists(BUILD_PATH):
        print(f"Build directory not found: {BUILD_PATH}")
        return None
        
    # Look for any .bin file in the build directory
    for file in os.listdir(BUILD_PATH):
        if file.endswith(".bin") and "bootloader" not in file and "partition" not in file:
            bin_path = os.path.join(BUILD_PATH, file)
            print(f"Found binary: {bin_path}")
            return bin_path
    
    print(f"Could not find binary file in {BUILD_PATH}")
    return None

def start_http_server(binary_path=None):
    """Start HTTP server to host the firmware"""
    # Find the binary
    bin_path = get_binary_path(binary_path)
    if not bin_path:
        return None, None, None
    
    # Create a temporary directory for serving only the binary
    temp_dir = os.path.join(BUILD_PATH, "ota_serve")
    os.makedirs(temp_dir, exist_ok=True)
    
    # Copy the binary to the temporary directory
    firmware_name = os.path.basename(bin_path)
    shutil.copy2(bin_path, os.path.join(temp_dir, firmware_name))
    
    # Change to the temporary directory
    os.chdir(temp_dir)
    
    # Start the server
    handler = http.server.SimpleHTTPRequestHandler
    httpd = socketserver.TCPServer(("", LOCAL_PORT), handler)
    
    server_thread = threading.Thread(target=httpd.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    
    local_ip = get_local_ip()
    print(f"HTTP server started at http://{local_ip}:{LOCAL_PORT}")
    print(f"Serving binary: {firmware_name}")
    return httpd, local_ip, firmware_name

def trigger_update(esp_ip, local_ip, local_port, firmware_name):
    """Send update notification to ESP32"""
    update_url = f"http://{esp_ip}//api/ota/update"
    firmware_url = f"http://{local_ip}:{local_port}/{firmware_name}"
    
    print(f"Triggering update on ESP32 ({esp_ip})...")
    print(f"Firmware URL: {firmware_url}")
    
    try:
        response = requests.post(
            update_url, 
            json={"firmware_url": firmware_url},
            timeout=5
        )
        if response.status_code == 200:
            print("Update triggered successfully")
            return True
        else:
            print(f"Failed to trigger update: {response.status_code}")
            return False
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to ESP32: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="ESP32 OTA Deploy Tool")
    parser.add_argument("--binary", type=str, help="Path to the binary file to deploy")
    parser.add_argument("--ip", type=str, default=ESP32_IP, help="ESP32 IP address")
    parser.add_argument("--wait", type=int, default=60, help="Seconds to wait for update to complete")
    args = parser.parse_args()
    
    # Start HTTP server
    http_server, local_ip, firmware_name = start_http_server(args.binary)
    if not http_server:
        return
    
    try:
        # Trigger the update
        if not trigger_update(args.ip, local_ip, LOCAL_PORT, firmware_name):
            return
        
        # Keep the server running while update happens
        print(f"Waiting {args.wait} seconds for ESP32 to complete the update...")
        time.sleep(args.wait)
        
    finally:
        # Shutdown the server
        http_server.shutdown()
        print("HTTP server stopped")

if __name__ == "__main__":
    main()
import requests

BASE = "http://IP_ESP32"

# avanti
print(requests.get(f"{BASE}/api/motors", params={"left": 120, "right": 120}, timeout=2).json())

# distanza
print(requests.get(f"{BASE}/api/distance", timeout=2).json())

# stop
print(requests.get(f"{BASE}/api/stop", timeout=2).json())

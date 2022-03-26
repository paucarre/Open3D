import open3d as o3d
devices = o3d.t.io.RealSenseSensor.list_devices()
print(devices)

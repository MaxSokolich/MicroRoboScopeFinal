import cv2
import EasyPySpin
import PySpin

cam = EasyPySpin.VideoCapture(0)

# ACCESS THE UNDERLYING PySpin CAMERA OBJECT
# EasyPySpin maps the 'cam' attribute to the hardware device
device = cam.cam 

# 1. DISABLE AUTO MODES (Crucial for consistent FPS)
device.AcquisitionFrameRateAuto.SetValue(PySpin.AcquisitionFrameRateAuto_Off)
device.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)

# 2. SET THE FRAME RATE
device.AcquisitionFrameRateEnabled.SetValue(True)
device.AcquisitionFrameRate.SetValue(19.0)

# 3. SET EXPOSURE TIME (Must be < 1/FPS)
# 19 FPS requires exposure < 52,600 microseconds. Let's set 40ms.
device.ExposureTime.SetValue(40000.0) 

# 4. OPTIMIZE THROUGHPUT
# 20MP generates massive data; ensure the bandwidth limit is at max
if hasattr(device, 'DeviceLinkThroughputLimit'):
    max_throughput = device.DeviceLinkThroughputLimit.GetMax()
    device.DeviceLinkThroughputLimit.SetValue(max_throughput)

# Verify settings
width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Resolution: {width}x{height} at {device.AcquisitionFrameRate.GetValue()} FPS")
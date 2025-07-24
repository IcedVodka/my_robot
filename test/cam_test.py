import pyrealsense2 as rs
import numpy as np
import cv2

# 1. 获取所有连接的 RealSense 设备
ctx = rs.context()
devices = ctx.query_devices()
serials = [dev.get_info(rs.camera_info.serial_number) for dev in devices]

pipelines = []
for serial in serials:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    pipelines.append((serial, pipeline))

try:
    while True:
        for serial, pipeline in pipelines:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # 用窗口名区分不同相机
            cv2.imshow(f'Color Stream {serial}', color_image)
            cv2.imshow(f'Depth Stream {serial}', depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    for _, pipeline in pipelines:
        pipeline.stop()
    cv2.destroyAllWindows()
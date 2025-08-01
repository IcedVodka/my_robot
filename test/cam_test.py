import pyrealsense2 as rs
import numpy as np
import cv2

# 获取所有连接的 RealSense 设备
ctx = rs.context()
devices = ctx.query_devices()
pipelines = []
serials = []

for dev in devices:
    serial = dev.get_info(rs.camera_info.serial_number)
    serials.append(serial)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    pipelines.append(pipeline)

try:
    while True:
        for i, pipeline in enumerate(pipelines):
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            # 显示窗口名带上序列号区分
            cv2.imshow(f'Color Stream {serials[i]}', color_image)
            cv2.imshow(f'Depth Stream {serials[i]}', depth_colormap)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    for pipeline in pipelines:
        pipeline.stop()
    cv2.destroyAllWindows()
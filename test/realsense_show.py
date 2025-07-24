import cv2
import time
from Robot.Reaksense_sensor import RealsenseSensor

if __name__ == "__main__":
    # 多个相机序列号，请根据实际情况填写
    CAMERA_SERIALS = [
        ("topcam", "207522073950"),
        ("handcam", "327122078945")
    ]
    
    cams = []
    for name, serial in CAMERA_SERIALS:
        cam = RealsenseSensor(name)
        cam.set_up(serial, is_depth=True)
        cam.set_collect_info(["color","depth"])
        cams.append(cam)

    print("按q退出...")
    try:
        while True:
            for cam in cams:
                data = cam.get_image_mp()
                if data is not None:
                    if "color" in data:
                        cv2.imshow(f"{cam.name}_color", cv2.cvtColor(data["color"], cv2.COLOR_RGB2BGR))
                    if "depth" in data:
                        depth = data["depth"]
                        # 归一化到0-255并转为uint8
                        depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                        depth_normalized = depth_normalized.astype('uint8')
                        # 伪彩色
                        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                        cv2.imshow(f"{cam.name}_depth", depth_colormap)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.05)
    finally:
        for cam in cams:
            cam.cleanup_mp()
        cv2.destroyAllWindows() 
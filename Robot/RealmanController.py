from Robotic_Arm.rm_robot_interface import *
import time
import numpy as np
import threading
import queue
from utils.debug_print import debug_print

# 灵巧手夹紧和松开的角度
Hand_grip_angles = [1000, 14000, 14000, 14000, 14000, 10000]
Hand_release_angles =  [4000, 17800, 17800, 17800, 17800, 10000]

Arm_init_joints = [0, 0, 90, 0, 90, 0]

class RealmanController():
    def __init__(self, name:str):
        super().__init__()
        self.name = name
        self.robot = None
        self.handle = None

    def set_up(self,rm_ip:str, port:int):
        try:
            self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self.handle = self.robot.rm_create_robot_arm(rm_ip, port)
            debug_print("RealmanController", f"机械臂ID：{self.handle.id}", "INFO")

            ret, joint_angles = self.robot.rm_get_joint_degree()
            if ret == 0:
                debug_print("RealmanController", f"当前机械臂各关节角度：{joint_angles}", "INFO")
            else:
                debug_print("RealmanController", f"读取关节角度失败，错误码：{ret}", "ERROR")      
        
        except Exception as e:
            raise ConnectionError(f"Failed to initialize robot arm: {str(e)}")
       

    def reset_zero_position(self, start_angles = None):
        """Move robot to the zero position"""
        debug_print(self.name, f"\nMoving {self.name} arm to start position...", "INFO")
        
        # Check if robot is still connected
        succ, _ = self.robot.rm_get_current_arm_state()
        if succ != 0:
            debug_print(self.name, f"Error: {self.name} arm is not connected or responding", "ERROR")
            return False
        
        # Get current joint positions
        succ, state = self.robot.rm_get_current_arm_state()
        if succ == 0:
            current_joints = state['joint']
            debug_print(self.name, f"Current {self.name} arm position: {current_joints}", "INFO")

        # Get start_angles positions
        if start_angles == None :
            succ , start_angles = self.robot.rm_get_init_pose()
            if succ != 0:
                debug_print(self.name, f"Error: {self.name} arm is not connected or responding", "ERROR")
                return False
        
        # Move to start position with error handling
        try:
            debug_print(self.name, f"Target {self.name} arm position: {start_angles}", "INFO")
            result = self.robot.rm_movej(start_angles, 20, 0, 0, 1)  # v=20%, blocking=True
            if result == 0:
                debug_print(self.name, f"Successfully moved {self.name} arm to start position", "INFO")
                # Verify current position
                succ, state = self.robot.rm_get_current_arm_state()
                if succ == 0:
                    current_joints = state['joint']
                    debug_print(self.name, f"New {self.name} arm position: {current_joints}", "INFO")
                    max_diff = max(abs(np.array(current_joints) - np.array(start_angles)))
                    if max_diff > 0.01:  # Allow small tolerance of 0.01 radians
                        debug_print(self.name, f"Warning: {self.name} arm position differs from target by {max_diff} radians", "WARNING")
                else:
                    debug_print(self.name, f"Warning: Could not verify {self.name} arm position", "WARNING")
                # Wait for system to stabilize
                debug_print(self.name, f"Waiting for {self.name} arm to stabilize...", "INFO")
                time.sleep(2)
                return True
            else:
                debug_print(self.name, f"Failed to move {self.name} arm to start position. Error code: {result}", "ERROR")
                return False
        except Exception as e:
            debug_print(self.name, f"Exception while moving {self.name} arm: {str(e)}", "ERROR")
            return False

    def get_state(self):
        # Get arm state
        succ, arm_state = self.robot.rm_get_current_arm_state()

        if succ != 0 or arm_state is None:
            raise RuntimeError("Failed to get arm state")
        state = arm_state.copy()
        return state  
    
    
    def set_arm_joints(self, joint):
        try:
            if len(joint) != 6:
                raise ValueError(f"Invalid joint length: {len(joint)}")
            success = self.robot.rm_movej_canfd(joint, False, 0, 0, 0)
            if success != 0:
                raise RuntimeError("Failed to set joint angles")
        except Exception as e:
            raise RuntimeError(f"Error moving robot: {str(e)}")
        
    def set_arm_init_joint(self):
        """
        让机械臂运动到全局变量 Arm_init_joints 所定义的位置
        """
        self.set_arm_joints(Arm_init_joints)

    def set_hand_angle(self, hand_angle, block=True, timeout=10):
        """
        设置灵巧手各自由度角度
        Args:
            hand_angle (list[int]): 手指角度数组，范围：0~1000，-1代表该自由度不执行任何操作
            block (bool): 是否阻塞，True为阻塞，False为非阻塞
            timeout (int): 阻塞模式下超时时间，单位：秒
        Returns:
            int: 状态码，0为成功，其他为失败
        """
        try:
            if not isinstance(hand_angle, (list, tuple)) or len(hand_angle) != 6:
                raise ValueError(f"Invalid hand_angle, must be list of 6 ints, got: {hand_angle}")
            tag = self.robot.rm_set_hand_follow_angle(hand_angle, block)
            if tag != 0:
                raise RuntimeError(f"Failed to set hand angle, error code: {tag}")
            return tag
        except Exception as e:
            raise RuntimeError(f"Error setting hand angle: {str(e)}")
    
    def set_hand_pos(self, hand_pos, block=True, timeout=10):
        """
        设置灵巧手位置跟随控制
        Args:
            hand_pos (list[int]): 手指位置数组，最大范围为0-65535，按照灵巧手厂商定义的角度做控制
            block (bool): 是否阻塞，True为阻塞，False为非阻塞
            timeout (int): 阻塞模式下超时时间，单位：秒
        Returns:
            int: 状态码，0为成功，其他为失败
        """
        try:
            if not isinstance(hand_pos, (list, tuple)) or len(hand_pos) != 6:
                raise ValueError(f"Invalid hand_pos, must be list of 6 ints, got: {hand_pos}")
            tag = self.robot.rm_set_hand_follow_pos(hand_pos, block)
            if tag != 0:
                raise RuntimeError(f"Failed to set hand position, error code: {tag}")
            return tag
        except Exception as e:
            raise RuntimeError(f"Error setting hand position: {str(e)}")
      
    
    def __del__(self):
        try:
            handle = self.robot.rm_delete_robot_arm()
            if handle == 0:
                debug_print("RealmanController", "\nSuccessfully disconnected from the robot arm\n", "INFO")
            else:
                debug_print("RealmanController", "\nFailed to disconnect from the robot arm\n", "WARNING")
        except Exception as e:
            debug_print("RealmanController", f"Error during disconnect in __del__: {e}", "ERROR")


class TeleoperationController:
    def __init__(self, master: RealmanController, slave: RealmanController, fps=30):
        self.master = master
        self.slave = slave
        self.fps = fps
        self._q = queue.Queue(maxsize=10)
        self._running = False
        self._master_thread = None
        self._slave_thread = None

        # 初始化时松开灵巧手
        self.release_hand(block=True)
        self.hand_state = 'open'  # 当前灵巧手状态: 'open' or 'grip'
        self.prev_hand_state = 'open'  # 上一次灵巧手状态

    def set_hand_state(self, state: str):
        """
        外部设置灵巧手状态
        :param state: 'open' 或 'grip'
        """
        if state not in ('open', 'grip'):
            raise ValueError("hand_state must be 'open' or 'grip'")
        self.hand_state = state

    def _master_collect(self):
        interval = 1.0 / self.fps
        while self._running:
            try:
                state = self.master.get_state()
                joint = state['joint']
                self._q.put(joint, timeout=0.1)
            except Exception as e:
                debug_print("TeleoperationController", f"[Master] Error collecting joint: {e}", "ERROR")
            time.sleep(interval)

    def _slave_play(self):
        while self._running:
            try:
                joint = self._q.get(timeout=0.5)
                self.slave.set_arm_joints(joint)
            except queue.Empty:
                pass
            except Exception as e:
                debug_print("TeleoperationController", f"[Slave] Error setting joint: {e}", "ERROR")
            # 灵巧手状态判断
            if self.hand_state != self.prev_hand_state:
                if self.hand_state == 'grip':
                    try:
                        self.grip_hand(block=True)
                        debug_print("TeleoperationController", "[Slave] 灵巧手夹紧", "INFO")
                    except Exception as e:
                        debug_print("TeleoperationController", f"[Slave] 灵巧手夹紧失败: {e}", "ERROR")
                elif self.hand_state == 'open':
                    try:
                        self.release_hand(block=True)
                        debug_print("TeleoperationController", "[Slave] 灵巧手松开", "INFO")
                    except Exception as e:
                        debug_print("TeleoperationController", f"[Slave] 灵巧手松开失败: {e}", "ERROR")
                self.prev_hand_state = self.hand_state

    def grip_hand(self, block=True):
        """
        夹紧灵巧手
        """
        return self.slave.set_hand_angle(Hand_grip_angles, block=block)

    def release_hand(self, block=True):
        """
        松开灵巧手
        """
        return self.slave.set_hand_angle(Hand_release_angles, block=block)

    def start(self):
        self._running = True
        self._master_thread = threading.Thread(target=self._master_collect)
        self._slave_thread = threading.Thread(target=self._slave_play)
        self._master_thread.start()
        self._slave_thread.start()
        debug_print("TeleoperationController", "遥操作已启动", "INFO")

    def stop(self):
        self._running = False
        if self._master_thread:
            self._master_thread.join()
        if self._slave_thread:
            self._slave_thread.join()
        debug_print("TeleoperationController", "遥操作已停止", "INFO")


   

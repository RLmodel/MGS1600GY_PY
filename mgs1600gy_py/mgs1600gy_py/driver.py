import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import String, Int64, Bool
from time import sleep
import re
from datetime import datetime
import serial

# for debug
import psutil



logger = rclpy.logging.get_logger('mgs_driver')

parse_maps = {
    # Read User Boolean Variable # 사용자 정의 변수(보류)
    # (ex B=0:0:0:0:0:...)
    # "B": [],
    
    # Read Track Detect # 감지하면 1, 못하면 0 (ex MGD=1)
    "MGD": bool,

    # Read all markers, or one of the 2 # 테스트 환경에서는 왼쪽 값만 확인 됨. 
    # (ex MGM=1:0)
    # "MGM": [],

    # Read all internal sensor values, or one of the 16 # 16개 인덱스로 첫번째 인덱스가 오른쪽, 마지막 인덱스가 왼쪽에 매칭됨. 
    # (ex MZ=24:32:45:67:89:100:...)
    "MZ": [
        ("sensor_1" , int, 1),
        ("sensor_2" , int, 2),
        ("sensor_3" , int, 3),
        ("sensor_4" , int, 4),
        ("sensor_5" , int, 5),
        ("sensor_6" , int, 6),
        ("sensor_7" , int, 7),
        ("sensor_8" , int, 8),
        ("sensor_9" , int, 9),
        ("sensor_10", int, 10),
        ("sensor_11", int, 11),
        ("sensor_12", int, 12),
        ("sensor_13", int, 13),
        ("sensor_14", int, 14),
        ("sensor_15", int, 15),
        ("sensor_16", int, 16),
    ],

    # Read selected track # 범위는 약 -100 ~ +100 LEFT가 (-) RIGHT가 (+)
    # (ex T=43)
    "T": int,

    # Read both the left and right tracks, or one of the 2
    # (ex MGT=0:0:0)
    "MGT": [int, int, int],

    # # Read User Integer Variable # 사용자 정의 변수(보류)
    # (ex VAR=0:0:0:0...)
    # "VAR": [],

    # # Read Gyroscope # 내장 IMU 사용 X
    # (ex GY=n:n:n)
    # "GY": [],

    # # Read MagSensor Status # T, MGT와 동류. 성능은 떨어짐.
    # (ex MGS=88)
    # "MGS": int,

    # # Read all markers, or one of the 2 and Cross Tape flag # 중복
    # "MGM": [],

    # # Read Tape Cross Detection # 애는 값이 안바뀜. Tape있어야 확인 할 수 있을 듯
    # (ex MGX=0)
    # "MGX": int

    # # Read Integrated Angle (degrees*10) # 내장 IMU 사용 X, 캘리 안된 데이터 나옴.
    # (ex ANG=8457:8107:8333)
    # "ANG": [],

    # 매뉴얼에 없는 쿼리. 센서가 구형인지 매뉴얼이 구형인지 모르겠음. # T, MGT와 동류.
    # (ex FS=72)
    # "FS": int,


    # 자석으로 확인한 쓸만한 데이터는 MGD MZ T MGT 정도.
}

class MgsDriver(Node):
    
    def __init__(self):
        super().__init__('mgs_driver') 

        self.pub_MGD = self.create_publisher(Bool, "/mgs_MGD", 10)
        self.pub_state = self.create_publisher(Bool, "/state_machine", 10)
        self.pub_T = self.create_publisher(Int64, "/mgs_T", 10)
        # self.pub_MZ = self.create_publisher(String, "/mgs_MZ", 10)
        # self.pub_MGT = self.create_publisher(String, "/mgs_MGT", 10)

        
        # self.file = open("/home/per/srcipt_241224", 'r')


    def Config_load(self, serial: serial.Serial):
        if self.save_config == True:
            try: 
                now = datetime.now()
                timestamp = now.strftime("%Y%m%d_%H%M%S")
                path = "/home/per/ivh_ws/src/mgs1600gy_py/log/log_{}.txt".format(timestamp)
                config_log_file = open(path, "w")
            except FileNotFoundError as e:
                logger.error("Path not exist. 경로를 다시 확인 해주세요\n"
                             "Error: %s" % e)
                return False
        
        while True:
            line = self.file.readline()
            print(line)
            serial.write(line.encode()+b'\r')
            sleep(0.01)
            if serial.readable():
                feedback = serial.read_all().decode()
            print('\r'+feedback)      
            if self.save_config == True:
                config_log_file.write(line+feedback)          
            if not line: 
                print("\nconfig set finished")
                self.file.close()
                break
                

       
    def publish_mgs(self, data):

        data = data.strip('\r')
        query_id = data[:3].strip('=0123456789-') # 2자리 쿼리 처리: = , 1자리 쿼리 처리: 0123456789
        # logger.info(f"{query_id}")
        

        if query_id not in parse_maps:
            logger.debug("query id %s is not in parse map, ignore data" % query_id)
            return False
        
        else : 
            ds = re.split(r'[=:]', data)
            if query_id == "T":
                try:
                    msg = Int64()
                    msg.data = int(ds[1])
                except IndexError as e:
                    print("[T] wrong data.\nError message: {0}.\ndata: {1}".format(e, ds))
                except ValueError as e:
                    print("[T] wrong data.\nError message: {0}.\ndata: {1}".format(e, ds))                   
                finally:
                    if msg.data > 100:
                        print("[T] data value wrong. value: %d" % msg.data)
                        msg.data = 100 
                        # return None
                    elif msg.data < -100:
                        print("[T] data value wrong. value: %d" % msg.data)
                        msg.data = -100 
                    # else:
                    self.pub_T.publish(msg)

            elif query_id == "MGD":
                try:
                    msg = Bool()
                    if ds[1] == "0":
                        msg.data = False
                    elif ds[1] == "1":
                        msg.data = True
                    else:
                        pass
                except IndexError as e:
                    print("[MGD] wrong data.\nError message: {0}.\ndata: {1}".format(e, ds))
                except ValueError as e:
                    print("[MGD] wrong data.\nError message: {0}.\ndata: {1}".format(e, ds))
                finally:
                    self.pub_state.publish(msg)
                    self.pub_MGD.publish(msg)
            
            # elif query_id == "MZ":
            #     print(data)
            #     try:
            #         msg = Bool()
            #         if ds[1] == "0":
            #             msg.data = False
            #         elif ds[1] == "1":
            #             msg.data = True
            #         else:
            #             pass
            #     except IndexError as e:
            #         print("[MGD] wrong data.\nError message: {0}.\ndata: {1}".format(e, ds))
            #     except ValueError as e:
            #         print("[MGD] wrong data.\nError message: {0}.\ndata: {1}".format(e, ds))
            #     finally:
            #         self.pub_state.publish(msg)
            #         self.pub_MGD.publish(msg)
            
            # elif query_id == "MGM":
            #     try:
            #         # logger.info(f"MGM: {ds}")
            #         msg = Mgm()
            #         if ds[1] == "0":
            #             msg.left = False
            #         elif ds[1] == "1":
            #             msg.left = True
            #         else:
            #             pass
            #         if ds[2] == "0":
            #             msg.center = False
            #         elif ds[2] == "1":
            #             msg.center = True
            #         else:
            #             pass

            #     except IndexError as e:
            #         print("[MGD] wrong data.\nError message: {0}.\ndata: {1}".format(e, ds))
            #     except ValueError as e:
            #         print("[MGD] wrong data.\nError message: {0}.\ndata: {1}".format(e, ds))
            #     finally:
            #         self.pub_MGM.publish(msg)                    
            #     pass
            else:
                pass
            
            


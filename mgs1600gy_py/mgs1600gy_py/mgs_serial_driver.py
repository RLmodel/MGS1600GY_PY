import serial
import rclpy
from mgs1600gy_py.driver import MgsDriver
import rclpy.publisher


def main(args=None):
    
    rclpy.init(args=args)
    driver = MgsDriver()
    query_0 = "# C_"                            # clear buffer
    query_1 = "?MGD\r\r?MZ\r\r?T\r\r?MGM\r\r"   # main data query
    query_2 = "# 10\r"                          # repeat last query every 10ms

    port_ = driver.declare_parameter('port', "/dev/ttyACM0").value
    baudrate_ = driver.declare_parameter('baudrate', 115200).value
    save_config_ = driver.declare_parameter('save_config', True).value
    
    # print(port_)

    try:
        MGS = serial.Serial(port_, baudrate_, timeout=3)
        driver.get_logger().info("Serial Connection done. port: {0}, baudrate: {1}".format(port_, baudrate_))
        try:
            # driver.Config_load(MGS)
            MGS.write(query_0.encode())
            MGS.write(query_1.encode())
            MGS.write(query_2.encode())
            while rclpy.ok():
                MGS.flush()
                data = MGS.read_until(b'\r')
                try: 
                    if isinstance(data, bytes):
                        data = data.decode()
                        driver.publish_mgs(data)

                except ValueError as e:
                    driver.get_logger().warn("value error. %s" % e)
                
        except Exception as e:
            driver.get_logger().error("ROS2 error: {0}".format(e))
            MGS.close()
    except serial.SerialException as ex:
        driver.get_logger().fatal("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
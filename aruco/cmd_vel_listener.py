import rospy
from geometry_msgs.msg import Twist
import pandas as pd
from datetime import datetime

class CmdVelLogger:
    def __init__(self):
        rospy.init_node('cmd_vel_logger', anonymous=True)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.data_df = pd.DataFrame(columns=['Timestamp', 'Linear Velocity', 'Angular Velocity'])

    def callback(self, data):
        # 현재 시간을 가져옵니다.
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # 받은 데이터를 DataFrame에 추가합니다.
        self.data_df = self.data_df.append({
            'Timestamp': current_time,
            'Linear Velocity': data.linear.x,
            'Angular Velocity': data.angular.z
        }, ignore_index=True)

        # 일정 데이터 수집 후 엑셀로 저장 (예: 50개 데이터마다 저장)
        if len(self.data_df) >= 50:
            filename = f'cmd_vel_data_{datetime.now().strftime("%Y%m%d%H%M%S")}.xlsx'
            self.data_df.to_excel(filename, index=False)
            print(f"Data saved to {filename}")
            self.data_df = self.data_df.iloc[0:0]  # 데이터 프레임 초기화

if __name__ == '__main__':
    logger = CmdVelLogger()
    rospy.spin()

from serialandFilter import SerialFilter
from enum import Enum

class TaskStatus(Enum):
    READY = "レディー"
    PROGRESS = "処理中"
    ERROR = "エラー"

class Type2BPVisApp:
    def __init__(self,com_port="/dev/ttyUSB0",baud_rate=3000000):
        self.serialFliter = SerialFilter(com_port=com_port,baud_rate=baud_rate)
        self.status = TaskStatus.READY
        
    def startVis(self):
        if not self.status == TaskStatus.PROGRESS:
            while True:
                data_frame = self.serialFliter._read_uwb_data_from_serial()
                print(data_frame)
        else:
            print("すでに計測を開始しております。")
    
    def stopVis(self):
        if self.status == TaskStatus.PROGRESS:
            self.status == TaskStatus.READY
            print("処理を中断しました。")
        else:
            print("処理は既に停止中です。")
    
    def disconnectSerial(self):
        if self.status == TaskStatus.PROGRESS:
            print("処理を停止してから、接続解除をしてください。")
        elif self.status == TaskStatus.READY:
            self.serialFliter.disconnect_serial()
            print("安全に取り外しが完了しました。")
    
if __name__ == "__main__":
    runner = Type2BPVisApp(com_port="/dev/ttyUSB0",baud_rate=3000000)
    runner.startVis()
            
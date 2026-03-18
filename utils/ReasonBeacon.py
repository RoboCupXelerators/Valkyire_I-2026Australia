import socket
import os
import threading
import subprocess,re
from ReasonData import logger,QkJson

class MisakaNetwork:
    def __init__(self):
        self.cfg = QkJson()
        result = subprocess.run(['ifconfig','wlan0'], capture_output=True, text=True)
        # if result.returncode == 0:
            # 严格验证IP地址范围（0-255）
        ip_pattern = r'inet ((?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?))'
        match = re.search(ip_pattern, result.stdout)
        if match:
            logger.info(f"IP地址: {match.group(1).upper()}")
            self.cfg.write("WIFI", "SelfIP", match.group(1).upper())
        self.LocalIP = match.group(1).upper()
        self.RemoteIP = self.cfg.read("WIFI","RemoteIP")
        self.Port = self.cfg.read("WIFI","Port")
        self.ListeningPort = self.cfg.read("WIFI","Port")
        self.MessageCache = None
        self.ReceiveDataThread = threading.Thread(target=self.ReceiveData)
        self.ReceiveDataThread.daemon = True  # 设置为守护线程，主线程结束时自动结束
        self.ReceiveDataThread.start()
        self.warned = False


    def Send(self,Data:str): #发送数据
        ClientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            ClientSocket.connect((self.RemoteIP,self.Port))
            self.warned = False
        except ConnectionRefusedError as e:
            if not self.warned:
                self.warned = True
                logger.warning(e)
            return False
        ClientSocket.sendall(Data.encode('utf-8'))
        ClientSocket.close()

    def ReceiveData(self): #接收数据
        ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ServerSocket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        ServerSocket.bind((self.LocalIP,self.ListeningPort))
        ServerSocket.listen(50)
        while(1):
            ClientSocket,ClientAddr = ServerSocket.accept()
            while(1):
                Data = ClientSocket.recv(1024)
                if not Data:
                    break
                Data = Data.decode('UTF-8')
                self.MessageCache = Data
            ClientSocket.close()
    
    def GetReceivedData(self):
        return self.MessageCache
    
if __name__ == "__main__":
    a = MisakaNetwork()
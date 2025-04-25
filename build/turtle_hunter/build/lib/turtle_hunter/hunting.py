import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill , Spawn
from std_srvs.srv import Empty


class Kurban:
    def __init__(
            self,
            name: str,
            x: float,
            y: float 
                 ):
        self.name: str = name
        self.pose: Pose = Pose()
        self.pose.x = x
        self.pose.y = y



class MyNode(Node):
    def __init__(self):
        super().__init__("hunting")
        self.avci_pose : Pose = Pose()
        self.kurbanlar: list[Kurban] = []
        self.kurban_isim_sayaci: int = 0
        self.pubs = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.create_subscription(Pose,"turtle1/pose",self.pose_cb,10)
        self.cli_kill = self.create_client(Kill , "kill")
        self.cli_spawn = self.create_client(Spawn , "spawn")
        self.cli_clear = self.create_client(Empty , "clear")
        self.create_timer(1.5 , self.kurban_yarat)
        self.create_timer(10.5,self.temizle)


        while not self.cli_kill.wait_for_service(0.5):
            self.get_logger().warn("kill servisi bekleniyor")
            
        while not self.cli_spawn.wait_for_service(0.5):
            self.get_logger().warn("spawn servisi bekleniyor")

        # self.kurban_yarat()
        
######################## ANA METOT ########################

    def ana_dongu(self):
        while True:
            rclpy.spin_once(node = self , timeout_sec = 0.5)
            self.hunt()

    def hunt(self):
        
        # ölüm var mı ?
        # en yakın hedefi sec
        target = self.hedef_sec()
        # hedef yoksa döndğr baba
        if target == -1:
            return
        # varsa git avla 
        self.kovala(target)

        self.olum_var_mi()


######################## KONTROL METOTLARI ########################
    def kovala(self,hedef_indis:int):
        mesafe = self.mesafe_hesaplama(hedef_indis)
        aci = self.aci_hesapla(hedef_indis)

        linear = max(0.5 , mesafe*1.2)
        angular = aci * 3.0
        self.pub_cmd_vel(linear,angular)
    
    def hedef_sec(self)-> int:
        en_yakin = 100000.0
        target = -1
        for i in range(len(self.kurbanlar)):
            mesafe = self.mesafe_hesaplama(i)
            if mesafe < en_yakin:
                en_yakin = mesafe
                target = i
        return target
    

    def olum_var_mi(self):
        oldurulecekler: list[Kurban] = []
        
        for i in range(len(self.kurbanlar)):
            if self.mesafe_hesaplama(i)<1.0:
                oldurulecekler.append(self.kurbanlar[i])
        for i in oldurulecekler:
            self.call_kill(i.name)
            self.kurbanlar.remove(i)


    def kurban_yarat(self):
        if len(self.kurbanlar)>=5:
            return
        name = f"kurban{self.kurban_isim_sayaci}"
        x = random.random() * 10 + 2
        y = random.random() * 10 + 2

        self.call_spawn(name,x,y)

        kurban = Kurban(name,x,y)
        self.kurbanlar.append(kurban)
        self.kurban_isim_sayaci += 1


######################## DİGER ########################

    def temizle(self):
        request = Empty.Request()
        self.cli_clear.call_async(request)

######################## HESAPLAMALAR ########################

    def mesafe_hesaplama(self,kurban_index: int) -> float:
        kurban = self.kurbanlar[kurban_index]
        delta_x = kurban.pose.x - self.avci_pose.x
        delta_y = kurban.pose.y - self.avci_pose.y

        distance = math.sqrt(delta_x**2 + delta_y**2)
        return distance
    
    def aci_hesapla(self,kurban_index: int) -> float:
        kurban = self.kurbanlar[kurban_index]
        
        delta_x = kurban.pose.x - self.avci_pose.x
        delta_y = kurban.pose.y - self.avci_pose.y
        
        kurban_aci = math.atan2(delta_y,delta_x)
        delta_aci = kurban_aci-self.avci_pose.theta
        print(f"ilkk {delta_aci}")
        if delta_aci > math.pi : 
            delta_aci -= 2*math.pi
        elif delta_aci < -math.pi: 
            delta_aci += 2*math.pi
        print(f"ikinci  {delta_aci}")
        return delta_aci

######################## ROS METOTLARI ########################
    def pose_cb(self , msg: Pose):
        self.avci_pose = msg


    def pub_cmd_vel(self , linear:float ,angular:float):
        
        msg=Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pubs.publish(msg)
    
    def call_kill(self,name: str):
        request = Kill.Request()
        request.name=name
        self.cli_kill.call_async(request)

    def call_spawn(self,name: str,x: float , y: float):
        request = Spawn.Request()
        request.name=name
        request.x=x
        request.y=y
        self.cli_spawn.call_async(request)

def main():
    rclpy.init()
    node = MyNode()
    node.ana_dongu()
    rclpy.shutdown()

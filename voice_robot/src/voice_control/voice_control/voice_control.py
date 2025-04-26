import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import speech_recognition as sr

class VoiceControl(Node):
    def __init__(self):
        super().__init__('voice_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.recognizer = sr.Recognizer()

        # İlk olarak ortam gürültüsünü ölç ve ayarla
        with sr.Microphone(device_index=4) as source:
            self.get_logger().info("⏳ Ortam gürültüsüne adapte olunuyor...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)

        # Mikrofon objesini başlat
        self.microphone = sr.Microphone(device_index=4)
        self.timer = self.create_timer(0.2, self.listen_callback)

        self.get_logger().info("✅ VoiceControl node başlatıldı (device_index=4)")

    def listen_callback(self):
        with self.microphone as source:
            self.get_logger().info("🔊 Dinliyorum...")
            try:
                audio = self.recognizer.listen(source, phrase_time_limit=3)
            except Exception as e:
                self.get_logger().warn(f"🎤 Dinleme hatası: {e}")
                return

        try:
            text = self.recognizer.recognize_google(audio, language='tr-TR').lower()
            self.get_logger().info(f"🗣️ Tanındı: “{text}”")
        except sr.UnknownValueError:
            self.get_logger().info("❓ Ses anlaşılamadı.")
            return
        except sr.RequestError as e:
            self.get_logger().error(f"🌐 API hatası: {e}")
            return

        twist = Twist()
        # İleri / Geri / Dur
        if 'ileri' in text:
            twist.linear.x = 0.2
        elif 'geri' in text:
            twist.linear.x = -0.2
        elif 'dur' in text:
            twist.linear.x = 0.0
        # Sağa / Sola dönüş
        elif 'sağa' in text or 'saga' in text:
            twist.angular.z = -0.5
        elif 'sola' in text:
            twist.angular.z = 0.5
        else: return

        self.pub.publish(twist)
        self.get_logger().info(
            f"▶️ Yayınlandı → linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔴 KeyboardInterrupt ile çıkılıyor")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

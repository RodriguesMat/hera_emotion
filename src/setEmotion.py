#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from hera_msgs.srv import SetEmotion

class EmotionNode(Node):
    def __init__(self):
        super().__init__('emotion_node')
        self.emotion_pub = self.create_publisher(String, '/hera/emotion', 10)
        self.srv = self.create_service(SetEmotion, 'setEmotion', self.set_emotion)

        self.emotions_validas = {
            'alegria': 'alegria',
            'surpresa': 'surpresa',
            'nojo': 'nojo',
            'angustia': 'angústia',
            'angústia': 'angústia',
            'pensativa': 'pensativa',
            
            'neutra': 'neutra',
            'desviando': 'desviando',
            'navegando': 'navegando',
            'encontrado': 'encontrado',
            'procurando': 'procurando'
        }

        self.get_logger().info('HERA Emotion Service pronto em /hera/say_emotion')

    def set_emotion(self, request: SetEmotion.Request, response: SetEmotion.Response):
        emotion_raw = request.emotion.strip().lower()
        if emotion_raw not in self.emotions_validas:
            response.success = False
            response.message = (
                f"Emoção '{emotion_raw}' inválida. "
                f"Use uma das: {list(self.emotions_validas.keys())}"
            )
            self.get_logger().warn(response.message)
            return response
          
        emotion = self.emotions_validas[emotion_raw]
        msg = String()
        msg.data = emotion
        self.emotion_pub.publish(msg)

        self.get_logger().info(f"Emoção definida: {emotion}")

        response.success = True
        response.message = f"Emoção '{emotion}' publicada em /hera/emotion."
        return response

if __name__ == '__main__':
    rclpy.init(args=None)
    emotion_node = EmotionNode()
    try:
        rclpy.spin(emotion_node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down...")

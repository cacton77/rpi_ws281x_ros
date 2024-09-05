import time
import sys
import signal
import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from inspection_msgs.msg import PixelStrip

import board
import neopixel_spi as neopixel

class PixelStripNode(Node):

    def __init__(self):
        super().__init__('pixel_strip')
        self.sub = self.create_subscription(
            PixelStrip, 'pixel_strip', self.set_pixels_callback, 10)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('led_count', 1),
                ('led_pin', 18),
                ('led_freq_hz', 800000),
                ('led_dma', 10),
                ('led_brightness', 255),
                ('led_invert', False),
                ('led_channel', 0)
            ]
        )

        self.init_strip()

        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_strip(self):

        led_count = self.get_parameter('led_count').value
        led_pin = self.get_parameter('led_pin').value
        led_freq_hz = self.get_parameter('led_freq_hz').value
        led_dma = self.get_parameter('led_dma').value
        led_invert = self.get_parameter('led_invert').value
        led_brightness = self.get_parameter('led_brightness').value
        led_channel = self.get_parameter('led_channel').value

        spi_bus_frequency = 6400000
        if led_freq_hz == 800000:
            spi_bus_frequency = 6400000
        elif led_freq_hz == 400000:
            spi_bus_frequency = 3200000

        spi = board.SPI()

        self.pixels = neopixel.NeoPixel_SPI(spi=spi,
                                       n=led_count,
                                       bpp=3,
                                       brightness=1.0,
                                       auto_write=False,
                                       pixel_order=neopixel.RGB,
                                       frequency=spi_bus_frequency)


    def set_pixels_callback(self, msg):
        self.get_logger().info('Message received...')
        for i, pixel_color in enumerate(msg.pixel_colors):
            r, g, b = int(pixel_color.r), int(pixel_color.g), int(pixel_color.b)
            self.get_logger().info(f'r: {r}, g: {g}, b: {b}')
            self.pixels[i] = (r, g, b)

        self.pixels.show()

    def on_shutdown(self):
        self.get_logger().info('Node is shutting down, turning off leds...')
        self.pixels.fill(0)

        self.pixels.show()


def main():
    rclpy.init()

    pixel_strip_node = PixelStripNode()

    def shutdown_hook(signum, frame):
        pixel_strip_node.on_shutdown()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_hook)
    signal.signal(signal.SIGTERM, shutdown_hook)

    try:
        rclpy.spin(pixel_strip_node)
    except KeyboardInterrupt:
        pass
    finally:
        pixel_strip_node.on_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

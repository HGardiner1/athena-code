#!/usr/bin/env python3
"""
ROS2 node that acquires raw spectra from the PDA and publishes RamanSpectrum messages.
Adapt _acquire_spectrum() to your actual hardware/radio interface.
"""
import rclpy
from rclpy.node import Node
from raman_msgs.msg import RamanSpectrum
import numpy as np


class RamanDriverNode(Node):
    def __init__(self):
        super().__init__('raman_driver_node')

        # Parameters
        self.declare_parameter('acquisition_rate_hz', 1.0)
        self.declare_parameter('integration_time_ms', 100.0)
        self.declare_parameter('laser_wavelength_nm', 785.0)
        self.declare_parameter('spectrometer_id', 'pda_spectrometer')
        self.declare_parameter('num_photodiodes', 3648)
        self.declare_parameter('wavenumber_min', 200.0)
        self.declare_parameter('wavenumber_max', 3500.0)

        rate = self.get_parameter('acquisition_rate_hz').value
        self.integration_time = self.get_parameter('integration_time_ms').value
        self.laser_wl = self.get_parameter('laser_wavelength_nm').value
        self.spec_id = self.get_parameter('spectrometer_id').value
        self.num_pd = self.get_parameter('num_photodiodes').value
        wn_min = self.get_parameter('wavenumber_min').value
        wn_max = self.get_parameter('wavenumber_max').value

        # Pixel-to-wavenumber calibration
        # TODO: Replace with actual calibration (polynomial or lookup table)
        self.wavenumber_axis = np.linspace(wn_min, wn_max, self.num_pd)

        # Publisher
        self.pub = self.create_publisher(RamanSpectrum, '/raman/raw_spectrum', 10)

        # Timer-driven acquisition
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.acquire_and_publish)

        self.get_logger().info(
            f'RamanDriverNode started: {self.num_pd} photodiodes, '
            f'{rate} Hz, integration={self.integration_time} ms'
        )

    def _acquire_spectrum(self) -> np.ndarray:
        """
        Acquire a single spectrum from hardware. Returns intensity array.

        TODO: Replace with actual acquisition:
          - Send trigger command to PDA over CAN/radio
          - Read back pixel intensity values
          - Reassemble CAN frames into full array

        Current implementation: simulated spectrum with 0-255 uint8 range for testing.
        """
        intensities = np.random.randint(0, 256, self.num_pd).astype(np.float64)

        # Simulated Lorentzian peaks (scaled to 0-255 range)
        for center_wn, amp, width in [
            (1001, 200, 8), (1031, 120, 10), (1155, 80, 12),
            (1450, 60, 15), (1602, 150, 10), (2852, 100, 20),
            (2904, 130, 18), (3054, 70, 22),
        ]:
            intensities += amp * (width ** 2) / (
                (self.wavenumber_axis - center_wn) ** 2 + width ** 2
            )

        # Clamp to 0-255 range
        intensities = np.clip(intensities, 0, 255)
        return intensities

    def acquire_and_publish(self):
        intensities = self._acquire_spectrum()

        msg = RamanSpectrum()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'raman_probe'
        msg.wavenumber_axis = self.wavenumber_axis.tolist()
        msg.intensities = intensities.tolist()
        msg.spectrometer_id = self.spec_id
        msg.integration_time_ms = self.integration_time
        msg.laser_wavelength_nm = self.laser_wl
        msg.accumulations = 1

        self.pub.publish(msg)
        self.get_logger().debug('Published raw spectrum')


def main(args=None):
    rclpy.init(args=args)
    node = RamanDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
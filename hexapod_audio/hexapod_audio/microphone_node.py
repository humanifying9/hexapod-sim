"""
microphone_node.py - capture stage for the hexapod's voice pipeline.

Publishes:
  /hexapod/audio_raw     (std_msgs/msg/UInt8MultiArray) - raw 16-bit PCM frames
  /hexapod/sound_direction (std_msgs/msg/Float32)        - azimuth to loudest source
  /hexapod/noise_level    (std_msgs/msg/Float32)         - RMS amplitude

This is the capture layer underneath the Whisper STT / command-parsing
pipeline described in the project README (STT consumes /hexapod/audio_raw;
this node doesn't do any recognition itself). Raw PCM uses a plain
UInt8MultiArray rather than audio_common_msgs/AudioData because
audio_common_msgs isn't available for ROS2 Jazzy via apt (only released for
the newer Kilted distro as of this writing) - build it from source if you
want the richer message type later.

In simulation (use_hardware: false, the default), publishes a synthetic
440Hz tone plus a low-amplitude noise floor so the rest of the pipeline can
be exercised without a real microphone. On real hardware, set use_hardware
true and provide the input device (needs the `pyaudio` package, which is
NOT currently installed - see the fallback behaviour below).
"""

import math
import struct
import time as time_module

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8MultiArray


class MicrophoneNode(Node):

    def __init__(self):
        super().__init__('hexapod_microphone')

        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('frame_size', 512)
        self.declare_parameter('use_hardware', False)
        self.declare_parameter('device', 'default')

        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.frame_size = self.get_parameter('frame_size').value
        self.use_hardware = self.get_parameter('use_hardware').value
        self.device = self.get_parameter('device').value

        self.audio_pub = self.create_publisher(UInt8MultiArray, '/hexapod/audio_raw', 10)
        self.direction_pub = self.create_publisher(Float32, '/hexapod/sound_direction', 10)
        self.noise_pub = self.create_publisher(Float32, '/hexapod/noise_level', 10)

        self._stream = None
        self._pyaudio = None
        self._phase = 0.0
        self._timer = None

        if self.use_hardware:
            try:
                self._open_hardware_stream()
                self._timer = self.create_timer(
                    self.frame_size / self.sample_rate, self._publish_hardware_frame)
                self.get_logger().info(f'Microphone node started (hardware) on {self.device}')
            except Exception as e:
                self.get_logger().warn(
                    f'Hardware microphone unavailable ({e}); falling back to synthetic tone')
                self.use_hardware = False
                self._start_synthetic()
        else:
            self._start_synthetic()

    def _start_synthetic(self):
        self._timer = self.create_timer(
            self.frame_size / self.sample_rate, self._publish_synthetic_frame)
        self.get_logger().info('Microphone node started (synthetic tone)')

    def _publish_synthetic_frame(self):
        t = time_module.time()

        samples = []
        for i in range(self.frame_size):
            sample = (
                0.3 * math.sin(2 * math.pi * 440.0 * (self._phase + i) / self.sample_rate)
                + 0.02 * math.sin(2 * math.pi * 0.7 * t)
            )
            samples.append(max(-1.0, min(1.0, sample)))
        self._phase += self.frame_size / self.sample_rate

        pcm = struct.pack(f'{self.frame_size}h', *(int(s * 32767) for s in samples))
        rms = math.sqrt(sum(s * s for s in samples) / len(samples))
        direction = 90.0 * math.sin(t * 0.2)  # slowly sweeping -90..+90 deg

        self._publish_audio(pcm, rms, direction)

    def _publish_hardware_frame(self):
        data = self._stream.read(self.frame_size, exception_on_overflow=False)
        if data:
            samples = struct.unpack(f'{self.frame_size}h', data)
            rms = math.sqrt(sum(s * s for s in samples) / len(samples)) / 32768.0
            direction = 0.0  # true direction needs a mic array - single-channel can't localize
            self._publish_audio(data, rms, direction)

    def _publish_audio(self, pcm_bytes, rms, direction):
        audio_msg = UInt8MultiArray()
        audio_msg.data = list(pcm_bytes)
        self.audio_pub.publish(audio_msg)

        noise_msg = Float32()
        noise_msg.data = rms
        self.noise_pub.publish(noise_msg)

        dir_msg = Float32()
        dir_msg.data = direction
        self.direction_pub.publish(dir_msg)

    def _open_hardware_stream(self):
        """Open the ALSA / PulseAudio capture device via PyAudio."""
        import pyaudio
        self._pyaudio = pyaudio.PyAudio()
        self._stream = self._pyaudio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.frame_size,
            input_device_index=None if self.device == 'default' else int(self.device),
        )

    def destroy_node(self):
        if self._timer:
            self._timer.cancel()
        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
        if self._pyaudio:
            self._pyaudio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

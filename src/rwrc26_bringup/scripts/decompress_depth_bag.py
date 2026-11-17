#!/usr/bin/env python3
"""Write a copy of a bag with compressedDepth topics turned back into raw depth images.

Each <topic>/compressedDepth (sensor_msgs/CompressedImage from
compressed_depth_image_transport) becomes <topic> (sensor_msgs/Image) with the same
header and bag time. Every other topic is copied unchanged, including the latched
/tf_static connection, so the output can be replayed directly. Run where rosbag,
OpenCV and NumPy are available (e.g. inside the ROS container):

  rosrun rwrc26_bringup decompress_depth_bag.py input.bag output.bag

Only the default PNG codec is supported. The input is only read and an existing
output is never overwritten.
"""
import argparse
from pathlib import Path
import struct
import sys

import cv2
import numpy as np

SUFFIX = '/compressedDepth'
HEADER = struct.Struct('<iff')  # compression_common.h ConfigHeader: format, depthParam[2]


def decode(msg):
    """Return (encoding, ndarray) for one compressedDepth CompressedImage."""
    encoding, _, codec = msg.format.partition(';')
    encoding = encoding.strip()
    codec = codec.split()
    if not codec or codec[0] != 'compressedDepth':
        raise ValueError('not a compressedDepth message: %r' % msg.format)
    if len(codec) > 1 and codec[1] != 'png':
        raise ValueError('codec %r is not supported; record with the png format' % codec[1])
    data = bytes(msg.data)
    if len(data) <= HEADER.size:
        raise ValueError('compressedDepth message too short')
    _, quant_a, quant_b = HEADER.unpack_from(data)
    image = cv2.imdecode(np.frombuffer(data, np.uint8, offset=HEADER.size), cv2.IMREAD_UNCHANGED)
    if image is None or image.dtype != np.uint16 or image.ndim != 2:
        raise ValueError('PNG payload is not a single-channel 16-bit image')
    if encoding in ('16UC1', 'mono16'):
        return encoding, image
    if encoding == '32FC1':  # Inverse-depth quantization used for float depth.
        depth = np.full(image.shape, np.nan, np.float32)
        valid = image != 0
        depth[valid] = quant_a/(image[valid].astype(np.float32)-quant_b)
        return encoding, depth
    raise ValueError('unsupported depth encoding %r' % encoding)


def to_image(msg):
    from sensor_msgs.msg import Image
    encoding, array = decode(msg)
    array = np.ascontiguousarray(array, array.dtype.newbyteorder('<'))
    return Image(header=msg.header, height=array.shape[0], width=array.shape[1],
                 encoding=encoding, is_bigendian=0, step=array.shape[1]*array.itemsize,
                 data=array.tobytes())


def main(argv=None):
    import rosbag
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--topic', action='append', default=[],
                        help='compressedDepth topic to convert (repeatable; default: all of them)')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output already exists: %s' % args.output)
    if args.output.resolve() == args.input.resolve():
        parser.error('output must differ from input')

    with rosbag.Bag(str(args.input)) as source:
        topics = source.get_type_and_topic_info().topics
        found = sorted(t for t, info in topics.items()
                       if t.endswith(SUFFIX) and info.msg_type == 'sensor_msgs/CompressedImage')
        selected = args.topic or found
        missing = sorted(set(selected)-set(found))
        if missing:
            parser.error('not a compressedDepth topic in the bag: '+', '.join(missing))
        if not selected:
            parser.error('no compressedDepth topics in the bag')
        renamed = {t: t[:-len(SUFFIX)] for t in selected}
        clash = sorted(set(renamed.values()) & set(topics))
        if clash:
            parser.error('raw topic already in the bag: '+', '.join(clash))

        counts = dict.fromkeys(selected, 0)
        try:
            with rosbag.Bag(str(args.output), 'w') as target:
                for topic, msg, stamp, header in source.read_messages(raw=True, return_connection_header=True):
                    if topic in renamed:
                        compressed = msg[4]()
                        compressed.deserialize(msg[1])
                        target.write(renamed[topic], to_image(compressed), stamp)
                        counts[topic] += 1
                    else:
                        target.write(topic, msg, stamp, raw=True, connection_header=header)
        except BaseException:
            args.output.unlink(missing_ok=True)  # Never leave a partial bag behind.
            raise
    for topic, count in counts.items():
        print('%s -> %s: %d images' % (topic, renamed[topic], count))
    return 0


if __name__ == '__main__':
    sys.exit(main())

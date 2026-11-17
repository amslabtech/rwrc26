#!/usr/bin/env python3
"""Decode compressedDepth payloads laid out like compressed_depth_image_transport (no ROS needed)."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/decompress_depth_bag.py'
spec = importlib.util.spec_from_file_location('decompress_depth_bag', SCRIPT)
tool = importlib.util.module_from_spec(spec); spec.loader.exec_module(tool)


def message(encoding, png_image, quant=(0., 0.), codec='png'):
    ok, png = cv2.imencode('.png', png_image)
    assert ok
    data = tool.HEADER.pack(0, *quant)+png.tobytes()
    return SimpleNamespace(format=encoding+'; compressedDepth '+codec, data=data)


class DecompressDepthTest(unittest.TestCase):
    def test_16bit_depth_is_restored_exactly(self):
        depth = np.random.default_rng(0).integers(0, 65535, (48, 64), dtype=np.uint16)
        encoding, restored = tool.decode(message('16UC1', depth))
        self.assertEqual(encoding, '16UC1')
        self.assertEqual(restored.dtype, np.uint16)
        np.testing.assert_array_equal(restored, depth)

    def test_float_depth_uses_inverse_depth_quantization(self):
        a, b = 100., 10.  # Arbitrary quantization parameters stored in the header.
        depth = np.random.default_rng(1).uniform(.3, 8., (48, 64)).astype(np.float32)
        inverse = np.round(a/depth+b).astype(np.uint16)
        inverse[0] = 0  # Invalid pixels.
        encoding, restored = tool.decode(message('32FC1', inverse, (a, b)))
        self.assertEqual((encoding, restored.dtype), ('32FC1', np.float32))
        self.assertTrue(np.isnan(restored[0]).all())
        np.testing.assert_allclose(restored[1:], a/(inverse[1:].astype(np.float32)-b), rtol=1e-6)

    def test_unsupported_payloads_are_rejected(self):
        depth = np.zeros((4, 4), np.uint16)
        for msg in (message('16UC1', depth, codec='rvl'),
                    SimpleNamespace(format='16UC1; png compressed ', data=b'x'*32),
                    SimpleNamespace(format='16UC1; compressedDepth png', data=b'x'*8),
                    message('16UC1', np.zeros((4, 4), np.uint8)),
                    message('8UC1', depth)):
            with self.subTest(format=msg.format), self.assertRaises(ValueError):
                tool.decode(msg)


if __name__ == '__main__': unittest.main()

"""Gate frame depth must not localize the hole's distant background."""
import unittest
import numpy as np
from utils.depth import associate_frame_depth


class GateDepthTest(unittest.TestCase):
    def test_thin_frame_and_center_occluder(self):
        depth = np.full((100, 100), 15., np.float32)
        depth[20:81, 20:22] = 10.
        depth[20:81, 79:81] = 10.
        depth[20:22, 20:81] = 10.
        depth[30:70, 48:52] = 6.  # flare occludes the hole, not the frame
        matrix = np.array([[100., 0, 50], [0, 100., 50], [0, 0, 1.]])
        self.assertEqual(associate_frame_depth(depth, (50, 50, 60, 60), matrix), (0., 0., 10.))

    def test_missing_depth(self):
        self.assertIsNone(associate_frame_depth(np.full((100, 100), np.nan),
                                               (50, 50, 60, 60), np.eye(3)))


if __name__ == '__main__':
    unittest.main(verbosity=2)

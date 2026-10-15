import os
import sys
import threading
from types import SimpleNamespace
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from path_history import PathHistory


def pose(x):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0, z=0)))


class PathHistoryTests(unittest.TestCase):
    def test_history_is_bounded_and_keeps_latest_points(self):
        history = PathHistory(max_points=3)
        for x in range(10000):
            history.append(pose(x))
        self.assertEqual([p.pose.position.x for p in history.snapshot()], [9997, 9998, 9999])

    def test_distance_filter_and_snapshot_isolation(self):
        history = PathHistory()
        self.assertTrue(history.append(pose(0)))
        self.assertFalse(history.append(pose(0.01)))
        self.assertTrue(history.append(pose(0.1)))
        snapshot = history.snapshot()
        snapshot.clear()
        self.assertEqual(len(history.snapshot()), 2)

    def test_rejects_invalid_capacity(self):
        with self.assertRaises(ValueError):
            PathHistory(max_points=0)

    def test_concurrent_snapshot_and_append(self):
        history = PathHistory(max_points=100)
        def append():
            for x in range(5000):
                history.append(pose(x))
        thread = threading.Thread(target=append)
        thread.start()
        while thread.is_alive():
            snapshot = history.snapshot()
            self.assertLessEqual(len(snapshot), 100)
            xs = [p.pose.position.x for p in snapshot]
            self.assertEqual(xs, sorted(xs))
        thread.join()
        self.assertEqual(len(history.snapshot()), 100)


if __name__ == '__main__':
    unittest.main()

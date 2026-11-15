"""Prediction moments must not depend on straight-motion subdivision."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from placenav.goal_evaluation import GoalEvaluationConfig, evaluate_goal
from placenav.place_recognition.odometry_motion import OdometryMotionModel
from placenav.subgoal_selector import ProgressSubgoalSelector
from placenav.topomap_odometry import OdomSample


class MotionConsistencyTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'nodes.json'
        self.names = [str(i)+'.jpg' for i in range(4)]
        self.path.write_text(json.dumps(dict(schema_version=1,
            units=dict(position='m', angle='rad'), child_frame_id='base_link',
            nodes=[dict(id=i, image=name, pose=dict(x=10*i, y=0, yaw=0))
                   for i, name in enumerate(self.names)])))

    def tearDown(self):
        self.temp.cleanup()

    def model(self, position, noise=0):
        model = OdometryMotionModel(self.path, self.names, noise=noise)
        model.initialize(np.ones(4)/4)
        model.probability.fill(0)
        model.probability[np.argmin(abs(model.grid_s-position))] = 1
        model.predict(self.sample(0))
        return model

    @staticmethod
    def sample(t, x=0):
        return OdomSample(int((10+t)*1e9), (x, 0, 0), 'odom', 'base_link')

    @staticmethod
    def moments(model):
        mean = float(model.grid_s @ model.probability)
        within = getattr(model, 'position_variance', np.zeros_like(model.grid_s))
        variance = float(((model.grid_s-mean)**2 + within) @ model.probability)
        return mean, variance

    def test_zero_noise_subcell_motion_does_not_diffuse(self):
        for count in [1, 10, 100]:
            model = self.model(15)
            for i in range(1, count+1):
                model.predict(self.sample(i/count, i/count))
            mean, variance = self.moments(model)
            self.assertAlmostEqual(mean, 16, places=9)
            self.assertLess(variance, 1e-12)

    def test_process_variance_depends_on_distance_not_update_count(self):
        for count in [1, 10, 100]:
            with self.subTest(count=count):
                model = self.model(15, noise=.1)
                for i in range(1, count+1):
                    model.predict(self.sample(i/count, i/count))
                mean, variance = self.moments(model)
                self.assertAlmostEqual(mean, 16, places=9)
                self.assertAlmostEqual(variance, .01, places=9)

    def test_passing_map_end_is_not_arrival_and_can_return(self):
        model = self.model(29)
        model.predict(self.sample(3, 3))
        self.assertAlmostEqual(self.moments(model)[0], 32)
        self.assertAlmostEqual(model.probability.sum(), 1)
        model.observe(np.ones(4))
        self.assertEqual(model.progress_probabilities(3)['near'], 0)
        self.assertEqual(evaluate_goal(model, 3, GoalEvaluationConfig())['arrival_score'], 0)
        for mode in ['nearest', 'passage']:
            selector = ProgressSubgoalSelector(model, 3, mode=mode)
            for stamp in range(1, 5):
                self.assertFalse(selector.update(3, stamp)[1])
        model.predict(self.sample(6, .8))
        self.assertAlmostEqual(self.moments(model)[0], 29.8)
        self.assertAlmostEqual(model.progress_probabilities(3)['near'], 1)

    def test_passing_map_start_does_not_accumulate_at_start(self):
        model = self.model(1)
        model.predict(self.sample(3, -3))
        self.assertAlmostEqual(self.moments(model)[0], -2)
        self.assertEqual(model.progress_probabilities(0)['near'], 0)
        model.predict(self.sample(6, -.7))
        self.assertAlmostEqual(self.moments(model)[0], .3)
        self.assertAlmostEqual(model.progress_probabilities(0)['near'], 1)

    def test_backward_and_forward_travel_accumulates_uncertainty(self):
        model = self.model(15, noise=.1)
        for i in range(1, 101):
            model.predict(self.sample(i/100, .01 if i % 2 else 0))
        mean, variance = self.moments(model)
        self.assertAlmostEqual(mean, 15, places=9)
        self.assertAlmostEqual(variance, .01, places=9)

    def test_reinitialize_clears_subcell_history_and_overflow(self):
        model = self.model(29, noise=.1)
        model.predict(self.sample(3, 3))
        self.assertGreater(evaluate_goal(model, 3, GoalEvaluationConfig())['outside_mass'], .9)
        model.initialize(np.ones(4)/4)
        self.assertTrue(np.all(model.position_variance == 0))
        np.testing.assert_allclose(model.node_belief(), np.ones(4)/4)
        self.assertEqual(model.probability[0], 0)
        self.assertEqual(model.probability[-1], 0)



if __name__ == '__main__':
    unittest.main()

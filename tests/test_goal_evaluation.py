"""Physical windows, conditional yaw and diagnostic-only score invariants."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from placenav.goal_evaluation import GoalEvaluationConfig, evaluate_goal, node_position_score
from placenav.place_recognition.odometry_motion import OdometryMotionModel
from placenav.topomap_odometry import OdomSample


class GoalEvaluationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.config = GoalEvaluationConfig()

    def tearDown(self):
        self.temp.cleanup()

    def model(self, positions=(0., 2., 4., 6.)):
        names = [str(i)+'.jpg' for i in range(len(positions))]
        path = Path(self.temp.name)/'nodes.json'
        path.write_text(json.dumps(dict(schema_version=1,
            units=dict(position='m', angle='rad'), child_frame_id='base_link',
            nodes=[dict(id=i, image=name, pose=dict(x=x, y=0, yaw=0))
                   for i, (x, name) in enumerate(zip(positions, names))])))
        model = OdometryMotionModel(path, names, noise=0)
        model.initialize(np.ones(len(names))/len(names))
        return model

    def mass(self, model, *points):
        model.probability.fill(0)
        for position, weight in points:
            index = np.argmin(np.abs(model.grid_s-position))
            model.probability[index] += weight

    def test_radius_is_independent_of_node_density(self):
        results = []
        for positions in [(0, 2, 4, 6), (0, .1, .2, 1, 2, 2.1, 2.2, 4, 6)]:
            model = self.model(positions)
            self.mass(model, (1.6, .2), (2.1, .5), (4.5, .3))
            results.append(evaluate_goal(model, positions.index(2), self.config))
        for name in ['position_score', 'arrival_score', 'neighborhood_mass',
                     'local_passage_score', 'local_passage_heading_score']:
            self.assertAlmostEqual(results[0][name], results[1][name])
        self.assertAlmostEqual(results[0]['arrival_score'], .7)
        self.assertAlmostEqual(results[0]['local_passage_score'], .5)

    def test_heading_changes_arrival_but_not_position_or_passage(self):
        model = self.model();self.mass(model, (2.1, 1))
        model.heading_offset.fill(math.pi/2)
        result = evaluate_goal(model, 1, self.config)
        self.assertEqual(result['position_score'], 1)
        self.assertEqual(result['arrival_score'], 0)
        self.assertEqual(result['local_passage_score'], 1)
        self.assertEqual(result['local_passage_heading_score'], 0)
        self.assertTrue(result['heading_reference_assumed'])

    def test_node_position_requires_target_ownership_in_dense_turn(self):
        model = self.model((0, .2, .4, 3.9, 7.4))
        self.mass(model, (0, .3), (.2, .4), (.4, .3))
        self.assertEqual(evaluate_goal(model, 1, self.config)['position_score'], 1)
        self.assertAlmostEqual(node_position_score(model, 1, .5), .4)
        model.heading_offset.fill(math.pi)
        self.assertAlmostEqual(node_position_score(model, 1, .5), .4)

    def test_node_position_integrates_variance_and_does_not_renormalize(self):
        model = self.model();self.mass(model, (2, .4), (6, .6))
        self.assertEqual(node_position_score(model, 1, .5), .4)
        index = int(np.argmin(np.abs(model.grid_s - 2)))
        model.position_variance[index] = 1
        expected = .4 * math.erf(.5 / math.sqrt(2))
        self.assertAlmostEqual(node_position_score(model, 1, .5), expected)

    def test_node_position_uses_joint_mass_instead_of_marginal_product(self):
        model = self.model();self.mass(model, (2, .5), (4, .5))
        model.position_variance[model.probability > 0] = 4
        near = model.interval_weights(1.5, 2.5)
        expected = float((model.probability * near)[model.owner == 1].sum())
        self.assertAlmostEqual(node_position_score(model, 1, .5), expected)
        marginal_product = float(model.probability @ near) * model.node_belief()[1]
        self.assertGreater(expected, marginal_product)

    def test_heading_uses_recorded_goal_yaw_and_wraps_at_pi(self):
        model = self.model();self.mass(model, (2, 1))
        model.node_yaw[1] = math.radians(179)
        model.heading_offset.fill(math.radians(-179))
        result = evaluate_goal(model, 1, self.config)
        self.assertEqual(result['arrival_score'], 1)
        self.assertAlmostEqual(result['map_yaw_error'], math.radians(2))

    def test_far_mass_is_not_renormalized_into_neighborhood(self):
        model = self.model();self.mass(model, (2, .4), (6, .6))
        result = evaluate_goal(model, 1, self.config)
        self.assertEqual(result['arrival_score'], .4)
        self.assertEqual(result['neighborhood_mass'], .4)
        self.assertEqual(result['map_s'], 6)
        self.assertEqual(result['local_passage_score'], .4)

    def test_evaluation_does_not_change_posterior_or_motion_state(self):
        model = self.model()
        model.predict(OdomSample(1, (0, 0, 0), 'odom', 'base_link'))
        model.predict(OdomSample(2, (.3, 0, .1), 'odom', 'base_link'))
        probability, heading = model.probability.copy(), model.heading_offset.copy()
        positions, variance = model.grid_s.copy(), model.position_variance.copy()
        before = (model.last_odometry, model.last_delta, model.last_boundary_clipped_mass)
        for goal in range(4):
            evaluate_goal(model, goal, self.config)
        np.testing.assert_array_equal(model.probability, probability)
        np.testing.assert_array_equal(model.heading_offset, heading)
        np.testing.assert_array_equal(model.grid_s, positions)
        np.testing.assert_array_equal(model.position_variance, variance)
        self.assertEqual((model.last_odometry, model.last_delta, model.last_boundary_clipped_mass), before)

    def test_boundary_diagnostics_report_retained_outside_mass(self):
        model = self.model();self.mass(model, (2, 1))
        model.predict(OdomSample(1, (0, 0, 0), 'odom', 'base_link'))
        model.predict(OdomSample(2, (10, 0, 0), 'odom', 'base_link'))
        result = evaluate_goal(model, 3, self.config)
        self.assertEqual(result['boundary_clipped_mass'], 0)
        self.assertEqual(result['outside_mass'], 1)
        self.assertEqual(result['endpoint_mass'], 0)
        self.assertEqual(result['arrival_score'], 0)
        model.predict(OdomSample(3, (10, 0, 0), 'odom', 'base_link'))
        self.assertEqual(evaluate_goal(model, 3, self.config)['boundary_clipped_mass'], 0)
        model.reset_odometry()
        self.assertEqual(model.last_boundary_clipped_mass, 0)

    def test_subcell_gaussian_is_integrated_instead_of_testing_its_mean_only(self):
        model = self.model();self.mass(model, (6, 1))
        index = int(np.argmax(model.probability))
        model.grid_s[index] = 8
        model.position_variance[index] = 4
        result = evaluate_goal(model, 3, self.config)
        self.assertAlmostEqual(result['position_score'], .1210, places=3)
        self.assertGreater(result['outside_mass'], .8)

    def test_invalid_parameters_and_missing_state_fail_clearly(self):
        for kwargs in [dict(position_tolerance=0), dict(yaw_tolerance=math.pi+.01),
                       dict(yaw_tolerance=float('nan')), dict(neighborhood_radius=float('inf')),
                       dict(position_tolerance=3, neighborhood_radius=2)]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                GoalEvaluationConfig(**kwargs)
        model = self.model()
        with self.assertRaises(ValueError):
            evaluate_goal(model, 4, self.config)
        with self.assertRaises(ValueError):
            node_position_score(model, 4, .5)
        model.probability = None
        with self.assertRaises(ValueError):
            evaluate_goal(model, 0, self.config)
        with self.assertRaises(ValueError):
            node_position_score(model, 0, .5)


if __name__ == '__main__':
    unittest.main()

import unittest
import numpy as np
from identify_clamp_causal import CausalFeatures, fit, predict


def replay(inputs):
    state = CausalFeatures()
    return [state.update(*row) for row in inputs]


class CausalTests(unittest.TestCase):
    def inputs(self, n=200):
        return [(i*.001, 20*np.sin(i*.02), 500., 400., 90., 6, 1) for i in range(n)]

    def test_future_and_truncation_invariance(self):
        rows = self.inputs()
        full = replay(rows)
        prefix = replay(rows[:100])
        changed = rows[:100] + [(r[0], -999., 0., 1., 0., 7, 2) for r in rows[100:]]
        altered = replay(changed)
        for a, b, c in zip(full[:100], prefix, altered[:100]):
            if a is None:
                self.assertIsNone(b); self.assertIsNone(c)
            else:
                np.testing.assert_array_equal(a, b)
                np.testing.assert_array_equal(a, c)

    def test_gap_nan_duplicate_and_reset(self):
        state = CausalFeatures()
        for row in self.inputs(40):
            state.update(*row)
        previous_a = state.a
        self.assertIsNone(state.update(.039, 99, 500, 400, 0, 6, 1))
        self.assertEqual(previous_a, state.a)
        self.assertIsNone(state.update(.100, 0, 500, 400, 0, 6, 1))
        self.assertIsNone(state.update(.101, float("nan"), 500, 400, 0, 6, 1))
        self.assertIsNone(state.update(.102, 1, 500, 400, 0, 6, 1))
        state.reset()
        self.assertIsNone(state.update(0, 0, 500, 400, 0, 5, 1))

    def test_stage_timer(self):
        state = CausalFeatures()
        for row in self.inputs(20):
            state.update(*row)
        x = state.update(.020, 1, 500, 400, 0, 7, 1)
        self.assertEqual(x[12], 0.)
        self.assertEqual(x[11], 1.)

    def test_intercept_survives_scaling(self):
        data = []
        for i, x in enumerate(replay(self.inputs())):
            if x is not None:
                data.append(dict(x=x, raw=np.array([2., -3.]), baseline=np.zeros(2),
                                 record="synthetic", cycle=1))
        model = fit(data)
        np.testing.assert_allclose(predict(model, data[-1]["x"]), [2., -3.], atol=1e-8)
        self.assertEqual(model["center"][0], 0.)
        self.assertEqual(model["scale"][0], 1.)


if __name__ == "__main__":
    unittest.main()

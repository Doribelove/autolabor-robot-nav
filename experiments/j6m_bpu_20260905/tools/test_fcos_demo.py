import unittest
import numpy as np
from decode_fcos_demo import decode, score_candidates, sigmoid, suppress_candidates, SIZES


def blank():
    return [np.full((80 if i < 5 else 4 if i < 10 else 1, SIZES[i % 5], SIZES[i % 5]),
                    1.0 if 5 <= i < 10 else -30.0, np.float32) for i in range(15)]


class DecoderTest(unittest.TestCase):
    def test_vectorized_nms_preserves_stable_per_class_greedy_results(self):
        random = np.random.default_rng(39)
        starts = random.uniform(0, 100, (300, 2))
        boxes = np.column_stack((starts, starts + random.uniform(1, 150, (300, 2))))
        candidates = [(float(random.choice([0.3, 0.5, 0.9])), int(random.integers(0, 4)), box)
                      for box in boxes]
        for threshold in (0.2, 0.6, 0.9):
            for maximum in (1, 10, 100):
                expected = []
                for score, label, box in sorted(candidates, key=lambda item: -item[0]):
                    keep = True
                    for previous in expected:
                        if previous['class_id'] != label:
                            continue
                        other = np.array(previous['bbox_model_px'])
                        overlap = np.prod(np.maximum(0, np.minimum(box[2:], other[2:]) - np.maximum(box[:2], other[:2])))
                        union = np.prod(box[2:] - box[:2]) + np.prod(other[2:] - other[:2]) - overlap
                        if overlap / max(union, 1e-12) > threshold:
                            keep = False
                            break
                    if keep:
                        expected.append(dict(class_id=label, confidence=score, bbox_model_px=box.tolist()))
                        if len(expected) == maximum:
                            break
                self.assertEqual(suppress_candidates(list(candidates), threshold, maximum), expected)

    def test_sparse_scoring_exactly_matches_dense_including_threshold_ties(self):
        random = np.random.default_rng(17)
        for threshold in (1e-30, 0.01, 0.2, 0.25, 0.5, 0.95, 0.99999999):
            for size in (7, 14, 112):
                logits = random.normal(-3, 5, (80, size, size)).astype(np.float32)
                centers = random.normal(-2, 5, (1, size, size)).astype(np.float32)
                # Saturation and exact 0.5*0.5 boundary, as well as random data.
                logits[:, 0, :3] = [-100, 0, 100]
                centers[:, 0, :3] = [-100, 0, 100]
                dense = sigmoid(logits) * sigmoid(centers)
                expected = np.nonzero(dense >= threshold)
                actual = score_candidates(logits, centers, threshold)
                for got, wanted in zip(actual[:3], expected):
                    np.testing.assert_array_equal(got, wanted)
                np.testing.assert_array_equal(actual[3], dense[expected])

    def test_empty(self):
        self.assertEqual(decode(blank()), [])

    def test_box_and_class(self):
        tensors = blank()
        tensors[0][3, 10, 20] = 10
        tensors[10][0, 10, 20] = 10
        result = decode(tensors)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["class_id"], 3)
        np.testing.assert_allclose(result[0]["bbox_model_px"], [156, 76, 172, 92])

    def test_nonfinite(self):
        tensors = blank()
        tensors[8][0, 0, 0] = np.nan
        with self.assertRaises(ValueError):
            decode(tensors)

    def test_wrong_shape(self):
        tensors = blank()
        tensors[0] = tensors[0][:2]
        with self.assertRaises(ValueError):
            decode(tensors)

    def test_negative_distance(self):
        tensors = blank()
        tensors[0][0, 0, 0] = tensors[10][0, 0, 0] = 10
        tensors[5][0, 0, 0] = -1
        with self.assertRaises(ValueError):
            decode(tensors)

    def test_preserve_different_classes(self):
        tensors = blank()
        tensors[0][:2, 10, 10] = tensors[10][0, 10, 10] = 10
        self.assertEqual(len(decode(tensors)), 2)

    def test_nms_and_limit(self):
        tensors = blank()
        tensors[0][0, 10, 10:12] = tensors[10][0, 10, 10:12] = 10
        tensors[5][:, 10, 10:12] = 10
        self.assertEqual(len(decode(tensors)), 1)
        with self.assertRaises(ValueError):
            decode(tensors, maximum=0)


if __name__ == "__main__":
    unittest.main()

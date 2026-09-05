from __future__ import annotations

import unittest

import numpy as np

from reference_code.rebar_inspector_s11_s22_reference import (
    ContinuousSweepAssembler,
    DualReflectionPoint,
    PersistentDualReflectionAcquirer,
    SweepConfig,
    build_dual_reflection_osl,
    decode_s11_s22_receivers,
)


def _forward(gamma: np.ndarray, d: np.ndarray, t: np.ndarray, m: np.ndarray) -> np.ndarray:
    return d + gamma * t / (1.0 - gamma * m)


class _FakeTransport:
    def __init__(self, events):
        self.events = list(events)
        self.open_count = 0
        self.start_count = 0
        self.idle_count = 0
        self.closed = False

    def open(self):
        self.open_count += 1

    def set_idle(self):
        self.idle_count += 1

    def start_continuous(self, config, stages_bitmap):
        self.start_count += 1
        self.config = config
        self.stages_bitmap = stages_bitmap

    def read_events(self, timeout_ms):
        result, self.events = self.events, []
        return result

    def close(self):
        self.closed = True


class ReferenceAcquisitionTests(unittest.TestCase):
    def test_receiver_descriptors_decode_both_reflections(self):
        point = decode_s11_s22_receivers(
            7,
            1.2e9,
            [
                (0x01, 0.5 + 0.2j),
                (0x02, 99 + 0j),      # unused stage-0 Port2
                (0x13, 1.0 + 0.0j),   # stage-0 reference
                (0x21, 99 + 0j),      # unused stage-1 Port1
                (0x22, -0.25 + 0.1j),
                (0x33, 0.5 + 0.0j),   # stage-1 reference
            ],
        )
        self.assertEqual(point.point_number, 7)
        self.assertAlmostEqual(point.s11, 0.5 + 0.2j)
        self.assertAlmostEqual(point.s22, -0.5 + 0.2j)

    def test_assembler_discards_incomplete_sweep_then_resynchronises(self):
        assembler = ContinuousSweepAssembler(3)
        self.assertIsNone(assembler.feed(DualReflectionPoint(0, 1.0, 1j, 2j)))
        self.assertIsNone(assembler.feed(DualReflectionPoint(2, 3.0, 3j, 4j)))
        self.assertIsNone(assembler.feed(DualReflectionPoint(0, 1.0, 1j, 2j)))
        self.assertIsNone(assembler.feed(DualReflectionPoint(1, 2.0, 3j, 4j)))
        sweep = assembler.feed(DualReflectionPoint(2, 3.0, 5j, 6j))
        self.assertIsNotNone(sweep)
        assert sweep is not None
        np.testing.assert_array_equal(sweep.frequency_hz, [1.0, 2.0, 3.0])
        self.assertEqual(sweep.statistics["dropped_sweeps"], 1)
        self.assertEqual(sweep.statistics["incomplete_sweeps"], 1)

    def test_session_opens_and_configures_once_for_multiple_sweeps(self):
        events = []
        for sweep_number in range(2):
            for point in range(3):
                events.append(
                    DualReflectionPoint(
                        point,
                        float(point + 1),
                        complex(sweep_number, point),
                        complex(-sweep_number, point),
                    )
                )
        transport = _FakeTransport(events)
        session = PersistentDualReflectionAcquirer(transport)
        session.open()
        session.configure(SweepConfig(1, 3, 3, 10_000))
        first = session.acquire_one()
        second = session.acquire_one()
        session.close()
        self.assertEqual((first.trace_index, second.trace_index), (0, 1))
        self.assertEqual(transport.open_count, 1)
        self.assertEqual(transport.start_count, 1)
        self.assertTrue(transport.closed)


class ReferenceCalibrationTests(unittest.TestCase):
    def test_dual_osl_recovers_two_independent_channels(self):
        frequency = np.linspace(0.5e9, 2.5e9, 11)
        x = np.linspace(0.0, 1.0, frequency.size)
        d11 = 0.02 + 0.01j * x
        t11 = 0.91 - 0.03j * x
        m11 = 0.05 + 0.02j * x
        d22 = -0.015 + 0.008j * x
        t22 = 0.86 + 0.04j * x
        m22 = -0.03 + 0.015j * x
        standards = (np.ones_like(x), -np.ones_like(x), np.zeros_like(x))

        p1 = [_forward(gamma, d11, t11, m11) for gamma in standards]
        p2 = [_forward(gamma, d22, t22, m22) for gamma in standards]
        profile = build_dual_reflection_osl(
            frequency,
            p1_open_s11=p1[0],
            p1_short_s11=p1[1],
            p1_load_s11=p1[2],
            p2_open_s22=p2[0],
            p2_short_s22=p2[1],
            p2_load_s22=p2[2],
        )

        true_s11 = 0.25 * np.exp(1j * np.linspace(0.0, 0.8, frequency.size))
        true_s22 = -0.35 * np.exp(-1j * np.linspace(0.0, 0.5, frequency.size))
        measured = np.stack(
            (_forward(true_s11, d11, t11, m11), _forward(true_s22, d22, t22, m22))
        )
        corrected = profile.correct(frequency, measured)
        np.testing.assert_allclose(corrected[0], true_s11, rtol=1e-11, atol=1e-11)
        np.testing.assert_allclose(corrected[1], true_s22, rtol=1e-11, atol=1e-11)


if __name__ == "__main__":
    unittest.main()


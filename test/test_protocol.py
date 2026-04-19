import unittest

from sensor_sync.protocol import (
    build_mode_command,
    build_status_summary,
    normalize_sync_mode,
    parse_state_line,
)


class ProtocolTests(unittest.TestCase):
    def test_normalize_sync_mode_accepts_aliases(self):
        self.assertEqual(normalize_sync_mode("gps"), "gps_pps")
        self.assertEqual(normalize_sync_mode("INTERNAL"), "internal_timer")
        self.assertEqual(normalize_sync_mode("external_trigger"), "external_trigger")

    def test_build_mode_command(self):
        self.assertEqual(build_mode_command("gps_pps"), "MODE GPS_PPS")
        self.assertEqual(build_mode_command("internal"), "MODE INTERNAL")

    def test_parse_state_line(self):
        parsed = parse_state_line(
            "STATE mode=gps_pps running=1 fps=30.00 pps_locked=1 pps_age_ms=24 "
            "stereo_frames=120 pps_count=4"
        )
        self.assertEqual(parsed["mode"], "gps_pps")
        self.assertTrue(parsed["running"])
        self.assertAlmostEqual(parsed["fps"], 30.0)
        self.assertTrue(parsed["pps_locked"])
        self.assertEqual(parsed["pps_age_ms"], 24)
        self.assertEqual(parsed["stereo_frames"], 120)
        self.assertEqual(parsed["pps_count"], 4)

    def test_build_status_summary(self):
        summary = build_status_summary(
            connected=True,
            running=False,
            mode="gps",
            fps=10.0,
            pps_locked=False,
            pps_age_ms=-1,
        )
        self.assertIn("connected", summary)
        self.assertIn("stopped", summary)
        self.assertIn("mode=gps_pps", summary)


if __name__ == "__main__":
    unittest.main()

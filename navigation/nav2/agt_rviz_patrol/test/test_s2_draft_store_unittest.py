"""Humble-independent tests of the S2 local-only scratch file, run with unittest."""
import json
import os
import tempfile
import unittest
from pathlib import Path

from agt_rviz_patrol.draft_store import load_draft, save_draft
from agt_rviz_patrol.route_editor import RouteEditor


CONTEXT = {'map_id': 'test-s2-map', 'map_version': 'revision-1',
           'robot_profile': 'bunker_v1', 'frame': 'map',
           'map_observation': ['map', 100, 100, 0.05, 'origin-test', 123456]}


class ScratchDraftTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(dir=os.environ.get('AGT_S2_RUNS'))
        self.addCleanup(self.directory.cleanup)
        self.file = Path(self.directory.name) / 'P001_P003.json'
        self.points = [(0., 0.), (1., 0.), (1., 1.)]

    def save(self):
        editor = RouteEditor()
        editor.replace(self.points, 0)
        editor.replace([(0., 0.), (1.1, 0.), (1., 1.)], editor.revision)
        return editor, save_draft(str(self.file), editor.points, editor.revision, CONTEXT)

    def test_three_point_roundtrip_restart_and_undo_redo(self):
        editor, token = self.save()
        points, revision, returned = load_draft(str(self.file), CONTEXT)
        self.assertEqual(returned, token)
        restarted = RouteEditor()
        restarted.restore(points, revision)
        self.assertEqual(restarted.points, editor.points)
        self.assertEqual(restarted.revision, editor.revision)
        self.assertEqual(restarted.history, [])
        restarted.replace([(0., 0.), (1.2, .2), (1., 1.)], revision)
        restarted.undo()
        self.assertEqual(restarted.points, editor.points)
        restarted.redo()
        self.assertEqual(restarted.points[1], (1.2, .2))
        self.assertEqual(json.loads(self.file.read_text())['state'], 'DRAFT')
        self.assertNotIn('route_manifest_sha256', self.file.read_text())

    def test_wrong_map_version_profile_observation_and_frame_rejected(self):
        self.save()
        for field, bad_value in [('map_id', 'wrong-map'), ('map_version', 'v2'),
                                 ('robot_profile', 'yhs_unverified'), ('frame', 'odom'),
                                 ('map_observation', ['different-grid'])]:
            context = dict(CONTEXT, **{field: bad_value})
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'mismatch'):
                load_draft(str(self.file), context)

    def test_unsupported_version_state_and_fields_fail_closed(self):
        self.save()
        for field, value in [('format_version', 2), ('state', 'READY'),
                             ('unexpected', 'present')]:
            doc = json.loads(self.file.read_text())
            doc[field] = value
            self.file.write_text(json.dumps(doc))
            with self.subTest(field=field), self.assertRaises(ValueError):
                load_draft(str(self.file), CONTEXT)
            self.file.unlink()
            self.save()

    def test_corrupted_json_and_noncanonical_points_rejected(self):
        self.file.write_text('{broken-json')
        with self.assertRaisesRegex(ValueError, 'invalid draft JSON'):
            load_draft(str(self.file), CONTEXT)
        self.file.unlink()
        self.save()
        doc = json.loads(self.file.read_text())
        doc['points'].append(doc['points'][-1])
        self.file.write_text(json.dumps(doc))
        with self.assertRaisesRegex(ValueError, 'collapsed duplicate'):
            load_draft(str(self.file), CONTEXT)

    def test_unknown_overwrite_and_external_change_rejected(self):
        editor, token = self.save()
        with self.assertRaisesRegex(ValueError, 'exists/changed'):
            save_draft(str(self.file), editor.points, editor.revision, CONTEXT)
        newer = save_draft(str(self.file), editor.points, editor.revision, CONTEXT, token)
        self.assertEqual(newer, token)
        self.file.write_text(self.file.read_text()+' ')
        with self.assertRaisesRegex(ValueError, 'exists/changed'):
            save_draft(str(self.file), editor.points, editor.revision, CONTEXT, token)

    def test_nonfinite_duplicate_and_too_long_rejected(self):
        for bad in [[(float('nan'), 0.)], [(0., 0.), (0., 0.)],
                    [(0., 0.), (201., 0.)]]:
            with self.subTest(points=bad), self.assertRaises(ValueError):
                save_draft(str(self.file), bad, 1, CONTEXT)
            self.assertFalse(self.file.exists())

    def test_symlink_and_relative_path_rejected(self):
        with self.assertRaisesRegex(ValueError, 'absolute'):
            save_draft('relative.json', self.points, 1, CONTEXT)
        other = Path(self.directory.name) / 'other.json'
        other.write_text('sensitive')
        self.file.symlink_to(other)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            load_draft(str(self.file), CONTEXT)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            save_draft(str(self.file), self.points, 1, CONTEXT)
        self.assertEqual(other.read_text(), 'sensitive')

    def test_restore_validation_no_mutation(self):
        e = RouteEditor()
        e.replace(self.points, 0)
        with self.assertRaises(ValueError):
            e.restore([(float('inf'), 0)], 88)
        self.assertEqual(e.points, self.points)
        self.assertEqual(e.revision, 1)

    def test_save_empty_after_clear_is_valid(self):
        e = RouteEditor()
        e.replace(self.points, 0)
        e.replace([], 1)
        save_draft(str(self.file), e.points, e.revision, CONTEXT)
        self.assertEqual(load_draft(str(self.file), CONTEXT)[:2], ([], 2))


if __name__ == '__main__':
    unittest.main()

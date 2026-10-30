"""Reject architectural regressions with small independent source fixtures."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_layers import inspect


class LayerTests(unittest.TestCase):
    def inspect_sources(self, sources, **policy):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name, text in sources.items():
                path = root / 'include/motion' / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
            return inspect(root, {'allowed_dependencies': {'core': [], 'math': ['core']}, **policy})

    def test_valid_lower_dependency(self):
        report = self.inspect_sources({'core/types.hpp': '', 'math/a.hpp': '#include <motion/core/types.hpp>'})
        self.assertFalse(report['errors'])
        self.assertEqual(report['layers']['math'], ['core'])

    def test_inactive_preprocessor_branch_does_not_hide_a_reverse_dependency(self):
        report = self.inspect_sources({'math/a.hpp': '', 'core/types.hpp': '#if 0\n#include "motion/math/a.hpp"\n#endif'})
        self.assertTrue(any('Forbidden core -> math' in message for message in report['errors']))

    def test_same_layer_include_cycle_is_rejected(self):
        report = self.inspect_sources({'math/a.hpp': '#include "motion/math/b.hpp"',
                                       'math/b.hpp': '#include "motion/math/a.hpp"'})
        self.assertTrue(any('file cycle' in message for message in report['errors']))

    def test_compatibility_facade_cannot_bypass_layer_rules(self):
        report = self.inspect_sources({'core/types.hpp': '',
                                       'math/old.hpp': '#include "motion/core/types.hpp"',
                                       'math/a.hpp': '#include "motion/math/old.hpp"'},
                                      compatibility_headers={'math/old.hpp': 'core/types.hpp'})
        self.assertTrue(any('compatibility header' in message for message in report['errors']))

    def test_runtime_closure_rejects_a_transitive_planning_dependency(self):
        report = self.inspect_sources({'core/types.hpp': '', 'math/a.hpp': '#include "motion/core/types.hpp"'},
                                      runtime_entries=['math/a.hpp'], runtime_allowed_layers=['math'])
        self.assertTrue(any('Runtime entry' in message for message in report['errors']))

    def test_cmake_component_cycle_is_rejected_even_without_header_cycles(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / 'components.cmake'
            path.write_text('set(MOTION_KNOWN_COMPONENTS core math)\nset(motion_deps_core math)\nset(motion_deps_math core)\n')
            report = inspect(root, {'allowed_dependencies': {'core': [], 'math': ['core']},
                                    'component_file': 'components.cmake'})
            self.assertTrue(any('CMake component cycle' in message for message in report['errors']))


if __name__ == '__main__':
    unittest.main()

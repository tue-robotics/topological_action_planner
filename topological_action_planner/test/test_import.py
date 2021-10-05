import unittest


class TestImport(unittest.TestCase):
    def test_import(self) -> None:
        """
        If no exception is raised, this test will succeed
        """
        import topological_action_planner


if __name__ == '__main__':
    unittest.main()

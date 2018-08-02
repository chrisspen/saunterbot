import unittest

def calculate_foot_angle(hip_angle, body_angle):
    return (body_angle - hip_angle)

class Tests(unittest.TestCase):
    
    def test_calc(self):
        self.assertEqual(calculate_foot_angle(hip_angle=-45, body_angle=-45), 0)
        self.assertEqual(calculate_foot_angle(hip_angle=-45, body_angle=0), 45)
        self.assertEqual(calculate_foot_angle(hip_angle=0, body_angle=0), 0)
        self.assertEqual(calculate_foot_angle(hip_angle=0, body_angle=45), 45)
        self.assertEqual(calculate_foot_angle(hip_angle=0, body_angle=-45), -45)
        self.assertEqual(calculate_foot_angle(hip_angle=45, body_angle=0), -45)
        self.assertEqual(calculate_foot_angle(hip_angle=45, body_angle=45), 0)
        self.assertEqual(calculate_foot_angle(hip_angle=45, body_angle=-45), -90)

if __name__ == '__main__':
    unittest.main()

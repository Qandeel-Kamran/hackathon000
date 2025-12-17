import unittest
from conversational_ai_core.msg import AICommand


class TestAICommandMsg(unittest.TestCase):

    def test_ai_command_msg_creation(self):
        """Test creating an AICommand message"""
        msg = AICommand()

        # Test default values
        self.assertEqual(msg.command_text, '')
        self.assertEqual(msg.user_id, '')
        self.assertEqual(msg.intent, '')
        self.assertEqual(msg.parameters, [])
        self.assertEqual(msg.response_text, '')
        self.assertEqual(msg.success, False)
        self.assertEqual(msg.error_message, '')
        self.assertEqual(msg.suggested_actions, [])

    def test_ai_command_msg_assignment(self):
        """Test assigning values to AICommand message"""
        msg = AICommand()

        # Assign values
        msg.command_text = 'Move forward'
        msg.user_id = 'user123'
        msg.intent = 'motion_command'
        msg.parameters = ['forward', '1.0']
        msg.response_text = 'Moving forward'
        msg.success = True
        msg.error_message = 'No error'
        msg.suggested_actions = ['stop', 'turn']

        # Verify values
        self.assertEqual(msg.command_text, 'Move forward')
        self.assertEqual(msg.user_id, 'user123')
        self.assertEqual(msg.intent, 'motion_command')
        self.assertEqual(msg.parameters, ['forward', '1.0'])
        self.assertEqual(msg.response_text, 'Moving forward')
        self.assertEqual(msg.success, True)
        self.assertEqual(msg.error_message, 'No error')
        self.assertEqual(msg.suggested_actions, ['stop', 'turn'])


if __name__ == '__main__':
    unittest.main()
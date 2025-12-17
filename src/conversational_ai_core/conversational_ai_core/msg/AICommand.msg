# Custom message for AI-robot communication
# Represents a natural language command from the conversational AI system

string command_text
string user_id
builtin_interfaces/Time timestamp
string intent
string[] parameters
---
# Response from the system
string response_text
bool success
string error_message
string[] suggested_actions
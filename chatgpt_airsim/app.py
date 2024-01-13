from flask import Flask, render_template, request, jsonify
from airsim_wrapper import *
import openai
import re
import json

app = Flask(__name__)

with open("config.json", "r") as f:
    config = json.load(f)
with open("system_prompts/airsim_basic.txt", "r") as f:
    sysprompt = f.read()
chat_history = [
    {"role": "system", "content": sysprompt},
    {"role": "user", "content": "move 10 units up"},
    {
        "role": "assistant",
        "content": """```python
aw.fly_to([aw.get_drone_position()[0], aw.get_drone_position()[1], aw.get_drone_position()[2]+10])
```

This code uses the `fly_to()` function to move the drone to a new position that is 10 units up from the current position. It does this by getting the current position of the drone using `get_drone_position()` and then creating a new list with the same X and Y coordinates, but with the Z coordinate increased by 10. The drone will then fly to this new position using `fly_to()`."""
    }
]

aw = AirSimWrapper()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/ask', methods=['POST'])
def ask():
    user_input = request.form['user_input']
    response = get_chatbot_response(user_input)
    code = extract_python_code(response)
    if code is not None:
        exec(code)
    return jsonify({'response': response})

def get_chatbot_response(user_input):
    chat_history.append({'role': 'user', 'content': user_input})
    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=chat_history,
        temperature=0
    )
    chat_history.append({'role': 'assistant', 'content': completion.choices[0].message.content})
    return chat_history[-1]['content']

code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
def extract_python_code(content):
    code_blocks = code_block_regex.findall(content)
    if code_blocks:
        full_code = "\n".join(code_blocks)
        if full_code.startswith("python"):
            full_code = full_code[7:]
        return full_code
    else:
        return None

if __name__ == '__main__':
    app.run(debug=True)

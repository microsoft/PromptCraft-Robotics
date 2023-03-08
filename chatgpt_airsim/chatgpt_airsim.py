import openai
import re
import argparse
from airsim_wrapper import *
import math
import numpy as np
import os
import json
import time

parser = argparse.ArgumentParser()
parser.add_argument("--prompt", type=str, default="prompts/airsim_basic.txt")
parser.add_argument("--sysprompt", type=str, default="system_prompts/airsim_basic.txt")
args = parser.parse_args()

with open("config.json", "r") as f:
    config = json.load(f)

print("Initializing ChatGPT...")
openai.api_key = config["OPENAI_API_KEY"]

with open(args.sysprompt, "r") as f:
    sysprompt = f.read()

chat_history = [
    {
        "role": "system",
        "content": sysprompt
    }
]


def ask(prompt):
    chat_history.append(
        {
            "role": "user",
            "content": prompt,
        }
    )
    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=chat_history,
    )
    chat_history.append(
        {
            "role": "assistant",
            "content": completion.choices[0].message.content,
        }
    )
    return chat_history[-1]["content"]


print(f"Done.")

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


class colors:  # You may need to change color settings
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"


print(f"Initializing AirSim...")
aw = AirSimWrapper()
print(f"Done.")

with open(args.prompt, "r") as f:
    prompt = f.read()

ask(prompt)
print("Welcome to the AirSim chatbot! I am ready to help you with your AirSim questions and commands.")

while True:
    question = input(colors.YELLOW + "AirSim> " + colors.ENDC)

    if question == "!quit" or question == "!exit":
        break

    if question == "!clear":
        os.system("cls")
        continue

    response = ask(question)

    print(f"\n{response}\n")

    code = extract_python_code(response)
    if code is not None:
        print("Please wait while I run the code in AirSim...")
        exec(extract_python_code(response))
        print("Done!\n")

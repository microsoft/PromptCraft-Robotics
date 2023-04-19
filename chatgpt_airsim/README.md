# ChatGPT-AirSim Interface

[![Demonstration video](https://user-images.githubusercontent.com/2274262/220220258-db492b6b-2a69-48ef-94a2-208c783f0b8a.png)](https://www.youtube.com/watch?v=iE5tZ6_ZYE8)

These scripts allow users to interact with and command a drone inside a [Microsoft AirSim](https://github.com/microsoft/AirSim) simulation using ChatGPT. ChatGPT is given access to a high-level function library containing some primitives for navigation, environmental information etc. which in turn maps to several internal AirSim functions. This mapping is done through an AirSim wrapper, which can be seen in `airsim_wrapper.py`. An example of a carefully crafted prompt (which also contains the high level function details) can be seen in `prompts/airsim_basic.txt`. The core logic of the interface is in `chatgpt_airsim.py`.

For details on how to use AirSim and its original API, please visit the [AirSim documentation](https://microsoft.github.io/AirSim/).

## Getting started (Windows only - for now)
- As a prerequisite, you need to create an OpenAI account that allows access to ChatGPT through the OpenAI API. You can do so by visiting https://platform.openai.com and signing up for an account.
- Set up the conda environment  
```
conda env create -f environment.yml
```
- Activate the environment and install the AirSim client.
```
conda activate chatgpt
pip install airsim
```
- Set up an API key by visiting https://platform.openai.com/account/api-keys. Copy the API key and paste it in the `OPENAI_API_KEY` field of `config.json`.
- Download the AirSim zip file from [Releases](https://github.com/microsoft/PromptCraft-Robotics/releases), and unzip the package.
- Copy `settings.json` to `C:\Users\<username>\Documents\AirSim\`.
  
## Running the interface
- Run the AirSim simulation as `.\run.bat` from the sim folder.
- Once simulation is up and running, run `python chatgpt_airsim.py` from this repo.

The interface shows an `AirSim>` prompt when it is ready to take user questions/commands. The commands are passed to ChatGPT and once the response is received (please note that the response could take some time to show up depending on ChatGPT response time), the code blocks within the response are automatically extracted and run in AirSim. 

We emphasize that this is meant to be a sample environment and interface aimed at getting started with ChatGPT and robotics applications inside a simulation sandbox. ChatGPT generated code could result in unexpected behavior or crashes, especially as the interface is written in a way that it executes the commands directly. That said, we do encourage users to test out the capabilities of the model, and to contribute new prompting techniques, additional wrapper functions, or any other enhancements to this interface.

We encourage users to keep evaluating new methods of accessing the ChatGPT service such as the Azure OpenAI service as appropriate.

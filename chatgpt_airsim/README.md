# ChatGPT-AirSim Interface

<a href="https://www.youtube.com/watch?v=iE5tZ6_ZYE8
" target="_blank">Demonstration Video</a>

These scripts allow users to interact with and command a drone inside a [Microsoft AirSim](https://github.com/microsoft/AirSim) simulation using ChatGPT. ChatGPT is given access to a high-level function library containing some primitives for navigation, environmental information etc. which in turn maps to several internal AirSim functions. This mapping is done through an AirSim wrapper, which can be seen in `airsim_wrapper.py`. An example of a carefully crafted prompt (which also contains the high level function details) can be seen in `prompts/airsim_basic.txt`. The core logic of the interface is in `chatgpt_airsim.py`.

For details on how to use AirSim and its original API, please visit the [AirSim documentation](https://microsoft.github.io/AirSim/).

## Getting started (Windows only - for now)
- As a prerequisite, you need to create an OpenAI account the allows access ChatGPT through the chat.openai.com website.
- Set up the conda environment  
`conda env create -f environment.yml`
- Set up the access token such that the ChatGPT interface can communicate with the model.   
Navigate to https://chat.openai.com/api/auth/session, and copy the `accessToken`. Paste it in the `access_token` field of `config.json`. 
- Download the AirSim binary from [Releases](https://github.com/microsoft/PromptCraft-Robotics/releases), and unzip the package.
- Copy `settings.json` to `C:\Users\<username>\Documents\AirSim\`.
  
## Running the interface
- Run the AirSim simulation through `run.bat`.
- Once the ChatGPT wrapper is set up, run `python chatgpt_airsim.py`

The interface shows an `AirSim>` prompt when it is ready to take user questions/commands. The commands are passed to ChatGPT and once the response is received (please note that the response could take some time to show up depending on ChatGPT response time), the code blocks within the response are automatically extracted and run in AirSim. 

We emphasize that this is meant to be a sample environment and interface aimed at getting started with ChatGPT and robotics applications inside a simulation sandbox. ChatGPT generated code could result in unexpected behavior or crashes, especially as the interface is written in a way that it executes the commands directly. That said, we do encourage users to test out the capabilities of the model, and to contribute new prompting techniques, additional wrapper functions, or any other enhancements to this interface.

Additionally, methods of accessing the ChatGPT service (such as the revchatGPT package used in this interface) could change over time. We encourage users to keep evaluating new methods including official API access or the Azure OpenAI service as appropriate.

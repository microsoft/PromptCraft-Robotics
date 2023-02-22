# PromptCraft-Robotics

The PromptCraft-Robotics repository serves as a community for people to test and share interesting prompting examples for large language models (LLMs) within the robotics domain. We also provide a sample [robotics simulator](https://github.com/microsoft/PromptCraft-Robotics/tree/main/chatgpt_airsim) (built on Microsoft AirSim) with ChatGPT integration for users to get started.

We currently focus on OpenAI's [ChatGPT](https://openai.com/blog/chatgpt/), but we also welcome examples from other LLMs (for example open-sourced models or others with API access such as [GPT-3](https://openai.com/api/) and Codex).

Users can contribute to this repository by submitting interesting prompt examples to the [Discussions](https://github.com/microsoft/PromptCraft-Robotics/discussions) section of this repository. A prompt can be submitted within different robotics categories such as [Manipulation](https://github.com/microsoft/PromptCraft-Robotics/discussions/categories/llm-manipulation), [Home Robotics](https://github.com/microsoft/PromptCraft-Robotics/discussions/categories/llm-home-robots), [Physical Reasoning](https://github.com/microsoft/PromptCraft-Robotics/discussions/categories/llm-physical-reasoning), among many others.
Once submitted, the prompt will be reviewed by the community (upvote your favorites!) and added to the repository by a team of admins if it is deemed interesting and useful.
We encourage users to submit prompts that are interesting, fun, or useful. We also encourage users to submit prompts that are not necessarily "correct" or "optimal" but are interesting nonetheless.

We encourage prompt submissions formatted as markdown, so that they can be easily transferred to the main repository. Please specify which LLM you used, and if possible provide other visuals of the model in action such as videos and pictures.

## Paper, videos and citations

Blog post: <a href="https://aka.ms/ChatGPT-Robotics" target="_blank">aka.ms/ChatGPT-Robotics</a>

Paper: <a href="https://www.microsoft.com/en-us/research/uploads/prod/2023/02/ChatGPT___Robotics.pdf" target="_blank">ChatGPT for Robotics: Design Principles and Model Abilities

Video: <a href="https://youtu.be/NYd0QcZcS6Q" target="_blank">https://youtu.be/NYd0QcZcS6Q</a>

If you use this repository in your research, please cite the following paper:

```
@techreport{vemprala2023chatgpt,
author = {Vemprala, Sai and Bonatti, Rogerio and Bucker, Arthur and Kapoor, Ashish},
title = {ChatGPT for Robotics: Design Principles and Model Abilities},
institution = {Microsoft},
year = {2023},
month = {February},
url = {https://www.microsoft.com/en-us/research/publication/chatgpt-for-robotics-design-principles-and-model-abilities/},
number = {MSR-TR-2023-8},
}
```

## ChatGPT Prompting Guides & Examples

The list below contains links to the different robotics categories and their corresponding prompt examples. We welcome contributions to this repository to add more robotics categories and examples. Please submit prompt examples to the [Discussions](https://github.com/microsoft/PromptCraft-Robotics/discussions) page, or submit a pull request with your category and examples.

* Embodied agent 
  * [ChatGPT - Habitat, closed loop object navigation 1](examples/embodied_agents/visual_language_navigation_1.md)
  * [ChatGPT - Habitat, closed loop object navigation 2](examples/embodied_agents/visual_language_navigation_2.md)
  * [ChatGPT - AirSim, object navigation using RGBD](examples/embodied_agents/airsim_objectnavigation.md)
* Aerial robotics
  * [ChatGPT - Real robot: Tello deployment](examples/aerial_robotics/tello_example.md) | [Video Link](https://youtu.be/i5wZJFb4dyA)
  * [ChatGPT - AirSim turbine Inspection](examples/aerial_robotics/airsim_turbine_inspection.md) | [Video Link](https://youtu.be/38lA3U2J43w)
  * [ChatGPT - AirSim solar panel Inspection](examples/aerial_robotics/airsim_solarpanel_inspection.md)
  * [ChatGPT - AirSim obstacle avoidance](examples/aerial_robotics/airsim_obstacleavoidance.md) | [Video Link](https://youtu.be/Vn6NapLlHPE)
* Manipulation
  * [ChatGPT - Real robot: Picking, stacking, and building the MSFT logo](examples/manipulation/pick_stack_msft_logo.md) | [Video Link](https://youtu.be/wLOChUtdqoA)
  * [ChatGPT - Manipulation tasks](examples/manipulation/manipulation_zeroshot.md)
* Spatial-temporal reasoning
  * [ChatGPT - Visual servoing with basketball](examples/spatial_temporal_reasoning/visual_servoing_basketball.md)


## ChatGPT + Robotics Simulator

We provice a sample [AirSim](https://github.com/microsoft/AirSim) environment for users to test their ChatGPT prompts. The environment is a binary containing a sample inspection environment with assets such as wind turbines, electric towers, solar panels etc. The environment comes with a drone and interfaces with ChatGPT such that users can easily send commands in natural language. [[Simulator Link]](chatgpt_airsim/README.md)

We welcome contributions to this repository to add more robotics simulators and environments. Please submit a pull request with your simulator and environment.

## Related resources

Beyond the prompt examples here, we leave useful and related links to the use of large language models below:

* [Read about the OpenAI APIs](https://openai.com/api/)
* [Azure OpenAI service](https://azure.microsoft.com/en-us/products/cognitive-services/openai-service)
* [OPT language model](https://huggingface.co/docs/transformers/model_doc/opt)

## Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.opensource.microsoft.com.

When you submit a pull request, a CLA bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., status check, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

## Trademarks

This project may contain trademarks or logos for projects, products, or services. Authorized use of Microsoft 
trademarks or logos is subject to and must follow 
[Microsoft's Trademark & Brand Guidelines](https://www.microsoft.com/en-us/legal/intellectualproperty/trademarks/usage/general).
Use of Microsoft trademarks or logos in modified versions of this project must not cause confusion or imply Microsoft sponsorship.
Any use of third-party trademarks or logos are subject to those third-party's policies.

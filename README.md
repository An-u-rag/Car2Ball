# Car2BALL

## Prerequisites:

1. [Unity](https://unity.com/download) 2022.3 or later
2. Python 3.10.13  
3. MLAgents release-20 branch
4. mlagents and mlagents-envs python packages

### For easy installation of python related stuff - use conda and install a new environment with the environment.yml file in this repo. This installs required versions of python, pytorch and mlagents.

### To continue with Unity mlagents package installation: 

You can follow the official ml-agents setup guide from unity [here](https://unity-technologies.github.io/ml-agents/Installation/). You can skip all the python related stuff as you already have the required conda environment. However note that the python version has to be 3.10.13 instead of 3.10.12. The ml-agents repository that you clone should be --branch release_20 instead of --branch release_21 (due to some compatibility issues.) 

```
git clone --branch release_20 https://github.com/Unity-Technologies/ml-agents.git
```

Then add the path to com.unity.ml-agents/package.json and com.unity.ml-agents.extension/package.json to Unity package manager as mentioned in the guide.

After this, your setup should be done!

## This is the main guide I followed for this specific example [here](https://unity-technologies.github.io/ml-agents/Learning-Environment-Create-New/)

### Pretrained model can be found under Assets/RollerBall.onnx

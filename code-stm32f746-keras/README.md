STM32F746 code for Keras
----

> This code is part of the post series in [here](https://www.stupid-projects.com/machine-learning-on-embedded-part-1/) and specifically from the 3rd article [here](https://www.stupid-projects.com/machine-learning-on-embedded-part-2/). In general, try to follow the instructions in that post

## Details
In this project I've used the CubeMX to import a Keras model using the X-CUBE-AI plugin for CubeMX.
For more details how to do that you can check this video [here](https://www.youtube.com/watch?v=grgNXdkmzzQ).

In my case I didn't use any compression on the weights as there is enough RAM and flash in the MCU remaining to fit the small code and the weights.

The use of this project is to upload a digit from the `

## Instructions
To build the project you need the AC6 IDE (aka SW4STM32) [here](https://www.st.com/en/development-tools/sw4stm32.html).

For the communication between the workstation and the stm32 I'm using nanopb, which is actually a lite version of the protocol buffers.

I've used Ubuntu 18.04 and there is a pre-build step that requires the protoc (compiler for protobuffers) to be installed.
For this reason, I've also included the `.bin` file so you can just flash the board without need to build the source.
For ubuntu:
```sh
sudo apt install protobuf-c-compiler python-protobuf protobuf-compiler
```

Nevertheless, you might need to generate the python client. The command to do so is:
```sh
protoc --python_out=python mnist_predict.proto
```

Also for the python nanopb client you need the pyserial
```sh
pip install pyserial
```

Anyway, there's the `create-files.sh` script in the folder that it generates both the C and python nanodb files.

## Notes
To build the code you need to generate the python cliemy


## Versions
STM32CubeIDE: 1.0.1
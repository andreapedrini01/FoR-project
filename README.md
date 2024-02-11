<p align='center'>
    <h1 align="center">Project of Fundamentals of Robotics</h1>
    <p align="center">
    Project for the Fundamental of Robotics course at the University of Trento A.Y. 2023/2024
    </p>
</p>

## Table of contents

+ [Introduction](#introduction)
+ [Project structure](#project-structure)
+ [Installation](#installation)
+ [How to run the project](#how-to-run-the-project)
  - [Setup](#setup)
  - [Running](#running)
+ [Known Issues](#known-issues)

## Introduction
A number of blocks are stored without any specific order on a stand located within the workspace of a robotic manipulator (Ur5). The manipulator is an anthropomorphic arm, with a spherical wrist and a two-fingered gripper as end-effector. Objects can belong to different classes but have a known geometry. The goal of the project is to use the manipulator to pick the objects in sequence and to position them on a different location according to a specified order. A calibrated 3D camera is used to locate the different objects and to detect their position in the stand.

## Project structure
+ **Motion planner ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)**
  - contains the catkin project for the motion planner and the task manager
  - `include` has the header files
  - `msg` has block's information
  - `source` has the source files
  - `CMakeLists.txt` is the CMake file for the project
  - `package.xml` is the package file for the project
+ **Vision ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)**
  - contains the visions scripts and weights
  - `dataset` contains the blocks' dataset
  - `models` contains the block's models
  - `scripts` has the scripts for the vision
  - `weights` has the weights of neural layers
  - `tavolo_obstacles.world` is the .world template

## Installation

## How to run the project

## Known Issues

## Contributors
<p align = 'center'>
  Lorenzo Pieropan [217869] <br>
  Federico Buzzini [227670] <br>
  Andrea Gravili [228055] <br>
  Andrea Pedrini [226607]
</p>

![image](assets/images/logo.jpg)
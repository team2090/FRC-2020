---
description: Tips for setting up your project
---

# Project Setup

## Why do we use command-based programming? \(optional\)

WPILib supports many different programming methodologies and each has their own advantages and disadvantages. These paradigms or styles for writing the robot code determines the basic structure for how our code will be organized. For our swerve drive robot in 2020 and 2019, we used "Command-Based" programming. You can learn more here: [https://bit.ly/3clkqAo](https://bit.ly/3clkqAo). The reason why we use Command-based programming is because it keeps the code clean, modular, and reusable from year to year if needed. Unlike some of the programming you may have done in FTC, Command-based programming is declarative and uses events \(e.g. button press\) to run certain parts of the code. Instead of checking for the state in a loop every iteration, you just need to connect the code you want to execute with the event handler. 

## Creating the project

This part is fairly simple; WPILib provides a utility in VSCode that creates a template project for you. They setup all the directories and essential files. You just need to add the vendor dependancies yourself, provide a team number, and name the project. 

{% hint style="danger" %}
This would be for creating a completely new project, but you could clone/fork an older project. However, WPILib does change stuff about the library each year, which can result in old code not working. They always post a page on their website about the changes. Refer to this whenever you are thinking of using old code, just to make sure the functionality is the same. Usually though, the library is similar from year-to-year and the logic \(e.g. reverse kinematics algorithm for swerve drive\) should remain the same.
{% endhint %}




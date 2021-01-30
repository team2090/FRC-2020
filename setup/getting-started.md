---
description: Basic setup for FRC programming
---

# Getting Started!

## Uninstalling Older Versions

While this step may seem like a pain now, having only one version of the software allows you to have an easier time managing the software development environment and save time in the long run, so that you can focus on programming and testing your code. 

You should be uninstalling older versions of the FRC Game Tools, FRC Radio Configuration Utility, and other third party applications such as the CTRE Phoenix Tuner. 

## Installing FRC Software

{% hint style="info" %}
Make sure to install the software on at least two computers, so that you always have a backup computer
{% endhint %}

There are two ways of installing the FRC Software: offline and online. I prefer the offline installation because it's easier to download all the necessary software into a single flash drive, which can be used to install software for each of the programming computers. However, either method of installation is correct. Make sure to read through the getting started and software setup guides because they contain important information about where to find the software and steps to install them. **Use admin when installing any software, otherwise, you could run into problems.**

## Installing Third Party Software

Third Party Software includes any tools/firmware/APIs from CTRE, REV Robotics, Kauai Labs. I would suggest putting all the firmware for the motor controls in a folder on the Desktop labeled "firmware".

* CTRE: Download the Phoenix Tuner and the most recent firmware for Talon SRXs and/or Falcon 500s
* REV Robotics: Download the SPARK MAX Client Application \(the bane of my existence\) and the most recent firmware for the Spark Max 

After installing all the firmware and software for the motor controls, you should now be ready to start setting up the code! Woo hoo! 

{% hint style="danger" %}
Make sure to keep all your firmware and software up to date \(especially throughout a robotics season since patches and updates are released\). Using different versions can cause problems that will make it difficult to debug your code or test the robot. 
{% endhint %}

{% hint style="info" %}
Labeling the computers and keeping track of which ones have admin privileges is a great way to stay organized throughout the season. Especially with multiple programmers on the team.
{% endhint %}

### Side Note: Java vs C++ \(optional\)

We use Java for our Swerve Drive code, but there you could use C++ \(WPILib supports both C++ and Java\). C++ actually offers many benefits over Java for FRC programming. C++ is generally less verbose \(in my opinion, Java is more annoying to program in\) and offers much more flexibility than Java. In C++, you can experiment more with more advanced code features and work with dynamic memory allocation. Also, header files are much easier to organize \(especially for command-based programming\). Overall, C++ generally results in more efficient code that performs better. However, using C++ has some trade-offs. Dynamic memory allocation is great, but if done improperly, you can easily cause memory leaks and corrupt parts of your memory during runtime. This can be difficult to debug \(especially if you don't have that much experience with pointers\). While Java can be more verbose and limited, it is generally safer to use since you don't have to mess with pointers. If you aren't looking to doing more advanced things with dynamic memory allocation, there really isn't that big of a difference between C++ and Java. The performance of the robot with Java is perfectly acceptable and some students may prefer OOP in Java.

**TLDR; Use whatever language you are more comfortable with \(or that more students know\)**


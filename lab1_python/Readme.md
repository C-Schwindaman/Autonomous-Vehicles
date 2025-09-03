# Lab 1: Repository Setup and Python Familiarity    <!-- omit in toc -->

## Contents   <!-- omit in toc -->
- [Python: What is expected for this course](#python-what-is-expected-for-this-course)
- [Setup](#setup)
- [Exercise 1 (23 Points)](#exercise-1-23-points)
- [Exercise 2 (7 Points)](#exercise-2-7-points)
- [Exercise 3 (0 Points, just for fun)](#exercise-3-0-points-just-for-fun)
- [Submitting this lab](#submitting-this-lab)
- [Due Date](#due-date)
___


# Python: What is expected for this course

This class will involve programming robots to perform complex tasks, and so inevitably will require programming skills. Fortunately, advanced tools (namely ROS) and modern languages (such as Python) make this fairly straight forward. All the teaching examples will use Python, and it is expected that you will can follow and program in Python yourself.

This lab assesses Chapters 1 to 16 of [Python Introduction Notes](https://github.com/dmorris0/python_intro/blob/main/README.md).  Its goal is to ensure that you are familiar with the basics of Python so that you can follow lecture examples, and that you have a working Python environment within ROS.  It is recommended that you complete this lab using the HPCC environment, although you can complete the first 2 exercises in your own local Python environment.

This assignment should not be hard, although if you are new to Python it may entail a fair bit of effort to get it working properly.  Feel free to use **online documentation, web searches and Gen-AI**, but **do not copy code from other people in the class**.  If you are unable to get a high score in this assignment, then you are strongly recommended to take a programming class before taking this course. 

# Setup

Follow the instructions in [labs_25/README.md](../README.md) to clone this repo as well as create your own `<student_repo>` with reporter permissions assigned to the TAs and instructor.

Copy the `lab1_python` folder from the `labs_25` repo into your personal `<student_repo>`.  You can modify this and submit your solution by committing and by pushing `<student_repo>`.

While not required, let's run this in a ROS Jazzy environment on HPCC.  (If you want, you can skip this and do it on your own computer.)  Make sure you have completed [Setup/HPCC](https://gitlab.msu.edu/av/autonomous-vehicles/-/blob/2025/Setup/HPCC.md) and [Setup/HPCC_ROS](https://gitlab.msu.edu/av/autonomous-vehicles/-/blob/2025/Setup/HPCC_ROS.md).  To enter your Jazzy environment type:
```
ros_shell
source ~/.rosinit
```
To activate your Python virtual environment called `av`, use:
```
act av
```
And your command line should then look something like this: 
```
(av) jazzy:user@name:~$
```  
The start of this line indicates you have activated a virtual Python environment named `av`, and are also in a ROS Jazzy-configured Ubuntu environment. 

The Python packages you will need, namely OpenCV and SciPy, are pre-installed in ROS so you do not need to install them in your virtual environment.  If you are working on your own computer, then you'll need to install these in your virtual environment.

# Exercise 1 (23 Points)

This lab comes with 
* 2 python files: [src/lab1_review.py](src/lab1_review.py) and [src/lab1_score.py](src/lab1_score.py)
* A sample image: [data/tail_lights.jpg](data/tail_lights.jpg)
* Ground truth results in folder [gt](gt)

The functions in `lab1_review.py` are incomplete, but their inputs and outputs are specified.  Your task is to complete all of the functions in this file accoring to the specifications in each function's documentation.  

The `lab1_score.py` function is provided as a means for you to score your work.  You can run it by first `cd` to the lab1_python folder:
```
cd lab1_python
```
And then run it with:
```python
python src/lab1_score.py
```
Or else run it directly in Visual Studio Code, which enables you to step through the code and figure out why your code isn't giving you the expected results.  

If you run this without making any changes to `lab1_review.py`, you should get a series of test failures that end with:
```
----------------------------------------------------------------------
Ran 23 tests in 0.086s

FAILED (failures=14, errors=9)
```
Also, you should see the raw color image with the back of the truck in a window.  To close the windows, click on them and press the space bar -- don't click on the `X`.

This scoring unit test will tell you how many errors or failures there are in your `lab1_review.py` code.  Each function you get correct will reduce either the failures or errors by 1.  When there are no failures or errors reported, then likely you have got everything right -- I say likely because those are not exhaustive tests, and the grader may use different tests.  But if your functions follow the directions then you should be good.  Here is the output I got when I got all the functions working:
```
....................
----------------------------------------------------------------------
Ran 23 tests in 0.159s

OK
```

Additionally, when you have completed everything correctly, you should see three windows like this showing the detected tail lights overlaid with rectangles:
<p align="left">
<img src="data/tail_lights.png" width="300">
<img src="gt/tail_lights_mask.png" width="300">
<img src="gt/tail_lights_rectangles.png" width="300">
</p>

# Exercise 2 (7 Points)

(a) Use your cell phone to take two photos of the rear of a car, each from a different viewpoint.  Resize the photos to approximately 640 pixels across.  Add them to the `data` folder with the names: `img1.jpg` and `img2.jpg`.  Write code using the color of pixels to detect the rear red-lights (they could be on or off -- your choice) in `img1.jpg`.  Your code must be called `light_detect.py`, and be put in the `src` folder.  The following command should run your code:
```
python src/light_detect.py data/img1.jpg
```
Your code should display a mask image indicating pixels it thinks are rear-light pixels, as well as the original image with bounding boxes drawn around the detected lights.  It is fine if it detects other red lights in the scene.  If possible, avoid detecting other non-rear-light objects.

(b) Now run your code on the second image with:
```
python src/light_detect.py data/img2.jpg
```
Does it perform as well on the second image as the first?  If not why not?  Add a text-only file called `answers.txt` in the `lab1_python` folder with your answer to 2(b).  

`Hint 1`: to add images to your repo, you can clone your repo to your PC and add them there and then commit and push, and then pull on the control workstation.

`Hint 2`: Avoid putting very large files in Git repos, as this makes repos slow to update.

# Exercise 3 (0 Points, just for fun)

Let's visualize the LiDAR, camera and vehicle data from one of the Indy Autonomous Vehicle races.  You should have a symbolic link to the course data here: `~/av/data`.  Move to the `iac_bags` folder with:
```
cd ~/av/data/iac_bags
```
Then in one terminal with Jazzy configured (i.e. with `ros_shell` running), play the bag in a loop with:
```
ros2 bag play AV_2025_Bag -l
```
Then in a second terminal, again with Jazzy configured, cd to the same folder and then type:
```
rviz2 -d rviz/tracking.rviz
```
You should see the race vehicle and a few opponents in both LiDAR and cameras.  Note that the `Robot Model` is not loaded.  We'll correct this when you have a ROS workspace.

# Submitting this lab
Your code should all be inside the `<student_repo>/lab1_python` folder.  To submit do:
```
cd <student_repo>/lab1_python
git add .
git commit -m "Add a comment here"
git push
```
Do not forget to `push`, or your lab will not be submitted.

# Due Date

Due 2:30pm EDT Thursday September 4, 2024.

#!/usr/bin/env python
# coding: utf-8
import numpy as np
import cv2 as cv
import os
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass

@dataclass
class Point3D:   # We'll use this class to represent 3D points
    x: float
    y: float
    z: float
# Do not import any more packages than the above
'''
    La1 1 Assignment 
    Based on Python Introduction Notes: https://github.com/dmorris0/python_intro

    Complete the following functions by replacing the pass command and/or variables set equal to None
    Functions need to return the specified output.  In most cases only a single line of code is required.  
    To test your functions and get a score out of 23, call:
      python lab1_student_score.py
    Or run lab1_score.py in VSCode.  When you have everything correct, the output should be:
....................
----------------------------------------------------------------------
Ran 23 tests in 0.100s

OK
    Also, you'll see 3 images displayed which you can close.
'''

####################
# Chapter 4: Strings

def find_warning(message: str) -> str:    
    '''
    Returns the index of the first instance of the substring "warning" or "Warning" (or any other variation on the capitalization)
    If there is no "warning" substring, returns -1
    Hint: don't forget to use a "return" statement
    '''
    return message.lower().find("warning")

def every_third(message: str) -> str:
    '''
    Returns every third letter in message starting with the second letter
    '''
    return message[1::3]

def all_words_reverse(message: str) -> list:
    '''
    Breaks message up at each space (" ") and puts the substrings into a list in reverse order
    '''
    return message.split(' ')[::-1]
    
def half_upper_case(message: str) -> str:
    '''
    Returns new_message, where new_message has the same letters as message, but the first half
        of the letters are upper case and the rest lower case.  
        If there are an odd number of letters, round down, that is the first half will have one fewer letters
    '''
    midpoint = len(message) // 2
    first_half = message[:midpoint].upper()
    second_half = message[midpoint:].lower()
    return first_half + second_half

#############################
# Chapter 5: Numbers and Math

def c_to_f(degree_c: float) -> float:    
    '''
    Converts Celcius to Fahrenheit using the formula
    °F = °C * 9/5 + 32 
    Returns output in Fahrenheit
    '''
    return (degree_c * (9 / 5)) + 32
    
def exp_div_fun(a: int, b: int, c: int) -> int:
    '''
    Return the integer remainder you get when you multiply a times itself b times and then divide by c
    '''
    return (a ** b) % c
    
 
 #################################
# Chapter 6: Functions and Loops
    
    
def lcm(x: int, y: int) -> int:
    '''
    Return lowest common multiple of x and y
    Method: let m be the larger of x and y
    Let testval start at m and in a loop increment it by m while testval is not divisible by both x and y
    return testval
    Hint: % will be useful
    '''
    m = max(x, y)
    testval = m
    while (testval % x != 0) or (testval % y != 0):
        testval += m
    return testval              

##################################################
# Chapter 8: Conditional Logic and Control Flow

def cond_cum_sum(a: int, b: int) -> int:
    '''
    Find the cumulative sum of numbers from 0 to a-1 that are not divisible by b
    Hint: % will be useful
    '''
    total_sum = 0
    for number in range(a):
        if number % b != 0:
            total_sum += number
    return total_sum

def divide_numbers(a: float, b: float) -> float:
    ''' 
    Return a / b
    Perform exception handling for ZeroDivisionError, and in this
    case return signed infinity that is the same sign as a
    Hint: np.sign() and np.inf will be useful
    '''
    try:
        return a / b
    except ZeroDivisionError:
        return np.sign(a) * np.inf

##################################################
# Chapter 9: Tuples, Lists and Dictionaries    

def inc_list(a: int, b: int) -> list:
    '''
    Return a list of numbers that start at a and increment by 1 to b-1
    '''
    return list(range(a, b))

def make_all_lower( string_list: list ) -> list:
    ''' Use a single line of Python code for this function
        string_list: list of strings
        returns: list of same strings but all in lower case
        Hint: list comprehension
    '''
    return [s.lower() for s in string_list]

def decrement_string(mystr: str) -> str:
    ''' Use a single line of Python code for this function (hint: list comprehension)
        mystr: a string
        Return a string each of whose characters has is one ASCII value lower than in mystr
        Hint: ord() returns ASCII value, chr() converts ASCII to character, join() combines elements of a list
    '''
    return "".join([chr(ord(char) - 1) for char in mystr])

def list2dict( my_list: list ) -> dict:
    ''' 
    Return a dictionary corresponding to my_list where the keys are elements of my_list
    and the values are the square of the key
    '''
    return {item: item ** 2 for item in my_list}

def concat_tuples( tuple1: tuple, tuple2: tuple ) -> tuple:
    ''' 
    Return a tuple that concatenates tuple2 to the end of tuple1
    '''
    return tuple1 + tuple2


##################################################
# Chapter 13: Numpy 
    
def matrix_multiplication(A: np.array,B: np.array) -> np.array:
    ''' 
    A, B: numpy arrays
    Return: matrix multiplication of A and B
    '''
    return A @ B

def largest_row(M: np.array) -> np.array:
    ''' 
    M: 2D numpy array
    Return: 1D numpy array corresponding to the row with the greatest sum in M
    Hint: use np.argmax
    '''
    return M[np.argmax(M.sum(axis=1))]   

def column_scale( A: np.array, vec: np.array) -> np.array:
    '''
    A: [M x N] 2D array
    vec: lenth N array
    return [M x N] 2D array where the i'th column is the corresponding column of A * vec[i]
    Hint: use broadcasting to do this in a single line
    '''
    return A * vec

def row_add( A: np.array, vec: np.array) -> np.array:
    '''
    A: [M x N] 2D array
    vec: lenth M array
    return [M x N] 2D array where the i row is the corresponding row of A + vec[i]
    Hint: use broadcasting to do this in a single line
    '''
    return A + vec[:, np.newaxis]

##################################################
# List comprehension from Chapter 9

def points_to_array(points: list) -> np.ndarray:
    '''
    Convert a list of 3D points to a numpy array
    points: list of [x,y,z] points
    return: [Mx3] numpy array
    '''
    return np.array([[p.x, p.y, p.z] for p in points])

def array_to_points(arr_points: np.ndarray) -> list:
    '''
    Convert a numpy array of 3D points to a list of Point3D
    arr_points: [Mx3] numpy array
    return: list of Point3D
    Note that iterating over a 2D array returns rows as 1D arrays
    '''
    return [Point3D(x=row[0], y=row[1], z=row[2]) for row in arr_points]


##################################################
# Chapter 14: scipy  

def rotate_90_y(A: np.array) -> np.array:
    '''
    A: [Mx3] array of M 3D points
    return: [Mx3] corresponding to points in A rotated by 90 degrees around Y axis
    Hint: use the scipy rotation operation
    '''
    rotation_y_90 = R.from_euler('y', 90, degrees=True)
    return rotation_y_90.apply(A)


def coordinate_transform(A: np.array) -> np.array:
    '''
    A: [Mx3] array of M 3D points
    return: [Mx3] corresponding to points in A rotated by 60 degrees around Y axis, shifted by 10 in X axis and roatated by 30 degrees around Z axis
    Hint: use the scipy rotation operation
    '''
    rot1 = R.from_euler('y', 60, degrees=True)
    points_after_rot1 = rot1.apply(A)
    points_after_translation = points_after_rot1 + np.array([10, 0, 0])
    rot2 = R.from_euler('z', 30, degrees=True)
    final_points = rot2.apply(points_after_translation)
    return final_points

def transform_point_array(arr_points: np.ndarray, rotation: R, translation: np.ndarray) -> np.ndarray:
    '''
    arr_points: [Mx3] array of M 3D points
    rotation: scipy Rotation object
    translation: [3] array
    return: [Mx3] corresponding to points in arr_points rotated by rotation and then shifted by translation
    '''
    # It's a good idea to use these checks on the inputs:
    if arr_points.ndim != 2 or arr_points.shape[1] != 3:
        raise ValueError("arr_points must be an array of shape (N, 3)")
    if translation.shape != (3,):
        raise ValueError("translation must be a 3-element vector")

    rotated_points = rotation.apply(arr_points)
    return rotated_points + translation


##################################################
# Chapter 15: OpenCV

class TailLights:

    def __init__(self, impath: str):
        self.impath = impath
        self.img = cv.imread(impath)      
        if self.img is None:
            print('')
            print('*'*60)  # If we can't find the image, check impath
            print('** Current folder is:      ',os.getcwd())
            print('** Unable to read image:   ',impath)  
            print('** Pass correct path to data folder during initialization')
            print('*'*60)
            self.init_succeed = False
        else:
            self.init_succeed = True

    def find_lights_pix(self, show=False) -> np.array:
        ''' Returns a binary mask image for self.img.  This should be 1 for pixels that meet the following conditions on 
            their red, green and blue channels, and zero otherwise.
            red > 220 AND blue > 70 AND blue <= 180 AND green > 120 AND green <= 190
            Note: do NOT iterate over pixels.  
            Hint: The mask can be created in one line of Python code.  
                  Use an AND operation for logical arrays.
            show: if True, then shows the mask in a window called 'Lights Pix'
        '''
        mask = (self.img[:,:,2] > 220) & (self.img[:,:,0] > 70) & (self.img[:,:,0] <= 180) & \
               (self.img[:,:,1] > 120) & (self.img[:,:,1] <= 190)
        binary_mask = mask.astype(np.uint8)
        if show:
            cv.imshow('Lights Pix', binary_mask * 255)
        return binary_mask

    def find_lights_bbox(self) -> np.array:    
        ''' Finds bounding box around lights 
            returns [Mx4] bounding boxes, one for each light.  Each row is [left, top, width, height] in pixels
            Hint: use cv.connectedComponentsWithStats, see Python_Intro documentation
        '''
        mask = self.find_lights_pix()
        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(mask, connectivity=4)
        return stats[1:, :-1]

    def draw_lights(self, show=False) -> tuple:
        ''' Draw red rectangles with thickness 2 pixels on the image around each detected light
            Returns image with rectangles draw on it.
            show: if True, then displays output image in a window called 'Detected Lights'
        '''
        bboxes = self.find_lights_bbox()
        img_out = self.img.copy()
        for bbox in bboxes:
            x, y, w, h = bbox
            cv.rectangle(img_out, (x, y), (x + w - 1, y + h - 1), color=(0, 0, 255), thickness=2)
        if show:
            cv.imshow('Detected Lights', img_out)
        return img_out

if __name__=="__main__":

    # A simple way to debug your functions above is to create tests below.  Then
    # you can run this code directly either in VSCode or by typing: 
    # python lab1_review.py

    # For example, you could do
    print( find_warning("Here is a warning") )
